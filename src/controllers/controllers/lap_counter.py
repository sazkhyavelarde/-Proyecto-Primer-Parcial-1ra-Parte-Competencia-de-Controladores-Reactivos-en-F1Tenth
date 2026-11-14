import csv
import math
import os
import statistics
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry


# Colores ANSI para la consola
VERDE = '\033[92m'
ROJO = '\033[91m'
AMARILLO = '\033[93m'
CIAN = '\033[96m'
NEGRITA = '\033[1m'
RESET = '\033[0m'


def fmt_delta(dt):
    """Diferencia con la mejor vuelta: verde si mejora, rojo si pierde."""
    if dt is None:
        return '  --  '
    color = VERDE if dt <= 0.0 else ROJO
    return f'{color}{dt:+.2f}{RESET}'


class LapCounter(Node):

    def __init__(self):
        super().__init__('lap_counter')

        # Línea de meta y conteo
        self.declare_parameter('finish_x', 0.0)
        self.declare_parameter('finish_y_min', -1.5)
        self.declare_parameter('finish_y_max', 1.5)
        self.declare_parameter('laps', 10)
        self.declare_parameter('min_lap_seconds', 15.0)
        self.declare_parameter('rearm_distance', 1.5)

        # Extras
        self.declare_parameter('start_on_move', True)     # arrancar al moverse
        self.declare_parameter('move_threshold', 0.1)     # m/s
        self.declare_parameter('lap_length_ref', 486.0)   # m, largo de Sepang
        self.declare_parameter('n_sectors', 3)
        self.declare_parameter('log_dir', os.path.expanduser('~/f1tenth_logs'))

        def param(name):
            return self.get_parameter(name).value

        self.finish_x = float(param('finish_x'))
        self.y_min = float(param('finish_y_min'))
        self.y_max = float(param('finish_y_max'))
        self.target_laps = int(param('laps'))
        self.min_lap_seconds = float(param('min_lap_seconds'))
        self.rearm_distance = float(param('rearm_distance'))
        self.start_on_move = bool(param('start_on_move'))
        self.move_threshold = float(param('move_threshold'))
        self.n_sectors = max(1, int(param('n_sectors')))
        ref = float(param('lap_length_ref'))
        self.sector_marks = [ref * (k + 1) / self.n_sectors
                             for k in range(self.n_sectors - 1)]

        # Estado de la carrera
        self.lap_count = 0
        self.start_time = None
        self.last_crossing = None
        self.best_lap = float('inf')
        self.prev_x = None
        self.prev_y = None
        self.prev_t = None
        self.armed = True
        self.finished = False

        # Estado de la vuelta actual
        self.lap_dist = 0.0
        self.lap_vmax = 0.0
        self.sector_idx = 0
        self.sector_start = None
        self.cur_sectors = []
        self.best_sectors = [None] * self.n_sectors
        self.lap_times = []
        self.total_dist = 0.0

        self.csv_file = None
        self.csv_writer = None
        self.open_csv(param('log_dir'))

        self.sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        self.get_logger().info(f'Contador de {self.target_laps} vueltas iniciado')

    # ------------------------------------------------------------------ registro

    def open_csv(self, log_dir):
        try:
            os.makedirs(log_dir, exist_ok=True)
            path = os.path.join(log_dir, 'sesiones_sepang.csv')
            new = not os.path.exists(path)
            self.csv_file = open(path, 'a', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            if new:
                self.csv_writer.writerow(
                    ['fecha', 'vuelta', 'tiempo_s', 'distancia_m', 'vel_media_m_s',
                     'vmax_m_s'] + [f's{k + 1}_s' for k in range(self.n_sectors)])
        except OSError as e:
            self.get_logger().warn(f'No se pudo abrir el CSV de vueltas: {e}')

    # ------------------------------------------------------------------ odometría

    def odom_callback(self, msg):

        if self.finished:
            return

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if not math.isfinite(x) or not math.isfinite(y):
            return

        v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        now = time.monotonic()

        # Iniciar cronómetro (al moverse o desde la primera odometría)
        if self.start_time is None:
            if self.start_on_move and v < self.move_threshold:
                self.prev_x, self.prev_y, self.prev_t = x, y, now
                return
            self.start_time = now
            self.last_crossing = now
            self.sector_start = now
            self.prev_x, self.prev_y, self.prev_t = x, y, now
            self.get_logger().info('Cronómetro iniciado desde la salida')
            return

        step = math.hypot(x - self.prev_x, y - self.prev_y)
        self.lap_dist += step
        self.total_dist += step
        self.lap_vmax = max(self.lap_vmax, v)

        # Cambio de sector por distancia recorrida
        if self.sector_idx < len(self.sector_marks) and \
                self.lap_dist >= self.sector_marks[self.sector_idx]:
            self.cur_sectors.append(now - self.sector_start)
            self.sector_start = now
            self.sector_idx += 1

        # Rearmar al alejarse de meta
        if abs(x - self.finish_x) > self.rearm_distance:
            self.armed = True

        # Detectar cruce en sentido positivo del eje X
        crossed = self.prev_x < self.finish_x <= x

        if crossed and x != self.prev_x:
            fraction = (self.finish_x - self.prev_x) / (x - self.prev_x)
            crossing_y = self.prev_y + fraction * (y - self.prev_y)
            # instante exacto del cruce, interpolado entre los dos mensajes
            t_cross = self.prev_t + fraction * (now - self.prev_t)

            if self.armed and self.y_min <= crossing_y <= self.y_max:
                self.armed = False
                lap_time = t_cross - self.last_crossing
                if lap_time >= self.min_lap_seconds:
                    self.cur_sectors.append(t_cross - self.sector_start)
                    self.close_lap(lap_time, t_cross)

        self.prev_x, self.prev_y, self.prev_t = x, y, now

    # ------------------------------------------------------------------ vueltas

    def close_lap(self, lap_time, t_cross):
        prev_best = self.best_lap if math.isfinite(self.best_lap) else None
        delta = None if prev_best is None else lap_time - prev_best
        is_best = prev_best is None or lap_time < prev_best

        self.lap_count += 1
        self.last_crossing = t_cross
        self.best_lap = min(self.best_lap, lap_time)
        self.lap_times.append(lap_time)
        total_time = t_cross - self.start_time
        v_avg = self.lap_dist / max(lap_time, 1e-3)

        sectors = list(self.cur_sectors)
        if len(sectors) != self.n_sectors:
            sectors = [float('nan')] * self.n_sectors   # sectores incompletos
        sec_txt = []
        for k, s in enumerate(sectors):
            best = self.best_sectors[k]
            if math.isfinite(s) and (best is None or s < best):
                self.best_sectors[k] = s
                sec_txt.append(f'{VERDE}S{k + 1} {s:.2f}{RESET}')
            else:
                sec_txt.append(f'S{k + 1} {s:.2f}')

        tag = f' {VERDE}{NEGRITA}MEJOR VUELTA{RESET}' if is_best else ''
        self.get_logger().warn(
            f'VUELTA {self.lap_count}/{self.target_laps} | '
            f'Tiempo: {NEGRITA}{lap_time:.2f} s{RESET} ({fmt_delta(delta)}) | '
            f'Mejor: {self.best_lap:.2f} s | Total: {total_time:.2f} s{tag}')
        self.get_logger().info(
            f'   {" | ".join(sec_txt)} | {self.lap_dist:.1f} m | '
            f'media {v_avg:.2f} m/s | vmax {self.lap_vmax:.1f} m/s')

        if self.csv_writer is not None:
            self.csv_writer.writerow(
                [datetime.now().isoformat(timespec='seconds'), self.lap_count,
                 f'{lap_time:.3f}', f'{self.lap_dist:.2f}', f'{v_avg:.3f}',
                 f'{self.lap_vmax:.2f}'] + [f'{s:.3f}' for s in sectors])
            self.csv_file.flush()

        # Reiniciar la vuelta
        self.lap_dist = 0.0
        self.lap_vmax = 0.0
        self.sector_idx = 0
        self.sector_start = t_cross
        self.cur_sectors = []

        if self.lap_count >= self.target_laps:
            self.finished = True
            self.print_summary('CARRERA FINALIZADA', total_time)

    def print_summary(self, title, total_time, use_print=False):
        if not self.lap_times:
            return
        times = self.lap_times
        flying = times[1:] if len(times) > 1 else times   # la 1.ª sale desde parado
        best_n = times.index(self.best_lap) + 1
        mean = statistics.mean(flying)
        std = statistics.pstdev(flying) if len(flying) > 1 else 0.0
        ideal = None
        if all(s is not None for s in self.best_sectors):
            ideal = sum(self.best_sectors)

        if use_print:
            # Al cerrar con Ctrl+C ROS ya no puede publicar en /rosout: se usa print
            class _Print:
                def info(self, txt):
                    print(txt, flush=True)
            log = _Print()
        else:
            log = self.get_logger()
        line = '=' * 50
        log.info(f'{CIAN}{line}{RESET}')
        log.info(f'{CIAN}{NEGRITA}{title} - SEPANG{RESET}')
        log.info(f'{CIAN}{line}{RESET}')
        for i, t in enumerate(times, 1):
            mark = f'  {VERDE}<- mejor{RESET}' if i == best_n else ''
            log.info(f'  Vuelta {i:2d}: {t:6.2f} s  ({t - self.best_lap:+.2f}){mark}')
        log.info(f'  Mejor vuelta:        {VERDE}{self.best_lap:.2f} s{RESET} (vuelta {best_n})')
        log.info(f'  Promedio lanzadas:   {mean:.2f} s')
        log.info(f'  Consistencia (desv): {std:.2f} s')
        if ideal is not None:
            log.info(f'  Vuelta ideal:        {AMARILLO}{ideal:.2f} s{RESET} '
                     f'(suma de mejores sectores)')
        log.info(f'  Tiempo total:        {total_time:.2f} s')
        log.info(f'  Distancia total:     {self.total_dist:.1f} m')
        log.info(f'{CIAN}{line}{RESET}')

    def close(self):
        if not self.finished and self.lap_times:
            self.print_summary('CARRERA INTERRUMPIDA', self.last_crossing - self.start_time,
                               use_print=True)
        try:
            if self.csv_file is not None:
                self.csv_file.close()
        except OSError:
            pass


def main(args=None):

    rclpy.init(args=args)
    node = LapCounter()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Al cerrar con Ctrl+C el contexto de ROS puede invalidarse dentro de spin()
        if rclpy.ok():
            raise
    finally:
        node.close()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()