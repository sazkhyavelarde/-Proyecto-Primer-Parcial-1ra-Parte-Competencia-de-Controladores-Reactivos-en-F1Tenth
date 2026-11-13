import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class LapCounter(Node):

    def __init__(self):
        super().__init__('lap_counter')

        self.declare_parameter('finish_x', 0.0)
        self.declare_parameter('finish_y_min', -1.5)
        self.declare_parameter('finish_y_max', 1.5)
        self.declare_parameter('laps', 10)
        self.declare_parameter('min_lap_seconds', 15.0)
        self.declare_parameter('rearm_distance', 1.5)

        self.finish_x = float(
            self.get_parameter('finish_x').value
        )
        self.y_min = float(
            self.get_parameter('finish_y_min').value
        )
        self.y_max = float(
            self.get_parameter('finish_y_max').value
        )
        self.target_laps = int(
            self.get_parameter('laps').value
        )
        self.min_lap_seconds = float(
            self.get_parameter('min_lap_seconds').value
        )
        self.rearm_distance = float(
            self.get_parameter('rearm_distance').value
        )

        self.lap_count = 0
        self.start_time = None
        self.last_crossing = None
        self.best_lap = float('inf')

        self.prev_x = None
        self.prev_y = None

        self.armed = True
        self.finished = False

        self.sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info(
            'Contador de 10 vueltas iniciado'
        )

    def odom_callback(self, msg):

        if self.finished:
            return

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)

        if not math.isfinite(x) or not math.isfinite(y):
            return

        now = time.monotonic()

        # Iniciar cronometro desde la primera odometria
        if self.start_time is None:

            self.start_time = now
            self.last_crossing = now

            self.prev_x = x
            self.prev_y = y

            self.get_logger().info(
                'Cronometro iniciado desde la salida'
            )

            return

        # Rearmar al alejarse de meta
        if abs(x - self.finish_x) > self.rearm_distance:
            self.armed = True

        # Detectar cruce en sentido positivo del eje X
        crossed = (
            self.prev_x < self.finish_x <= x
        )

        if crossed and x != self.prev_x:

            fraction = (
                (self.finish_x - self.prev_x)
                / (x - self.prev_x)
            )

            crossing_y = (
                self.prev_y
                + fraction * (y - self.prev_y)
            )

            if (
                self.armed
                and self.y_min <= crossing_y <= self.y_max
            ):

                self.armed = False

                lap_time = now - self.last_crossing

                if lap_time >= self.min_lap_seconds:

                    self.lap_count += 1
                    self.last_crossing = now

                    self.best_lap = min(
                        self.best_lap,
                        lap_time
                    )

                    total_time = now - self.start_time

                    self.get_logger().warn(
                        f'VUELTA {self.lap_count}/'
                        f'{self.target_laps} | '
                        f'Tiempo: {lap_time:.2f} s | '
                        f'Mejor: {self.best_lap:.2f} s | '
                        f'Total: {total_time:.2f} s'
                    )

                    if self.lap_count >= self.target_laps:

                        self.finished = True

                        self.get_logger().info(
                            '=============================='
                        )
                        self.get_logger().info(
                            'CARRERA FINALIZADA'
                        )
                        self.get_logger().info(
                            f'Mejor vuelta: '
                            f'{self.best_lap:.2f} s'
                        )
                        self.get_logger().info(
                            f'Tiempo total: '
                            f'{total_time:.2f} s'
                        )
                        self.get_logger().info(
                            '=============================='
                        )

        self.prev_x = x
        self.prev_y = y


def main(args=None):

    rclpy.init(args=args)
    node = LapCounter()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()