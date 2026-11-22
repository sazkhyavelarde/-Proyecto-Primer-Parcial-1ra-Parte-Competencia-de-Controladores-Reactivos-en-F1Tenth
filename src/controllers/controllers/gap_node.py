import csv
import math
import os
import time
from datetime import datetime

import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


# ============================================================================
# Funciones puras (sin ROS) - se pueden probar por separado
# ============================================================================

def contiguous_runs(mask):
    """Devuelve una lista de tramos (inicio, fin) inclusivos donde mask es True."""
    if mask.size == 0:
        return []
    m = mask.astype(np.int8)
    d = np.diff(np.concatenate(([0], m, [0])))
    starts = np.where(d == 1)[0]
    ends = np.where(d == -1)[0] - 1
    return list(zip(starts.tolist(), ends.tolist()))


def preprocess_ranges(raw, range_min, range_max, max_range):
    """
    Limpieza del LiDAR:
      - +inf -> alcance máximo (espacio libre),
      - lecturas finitas por debajo del mínimo -> mínimo (obstáculo muy cercano, conservador),
      - NaN -> interpolación con los vecinos válidos,
      - mediana de 3 rayos: quita ruido puntual sin desplazar los bordes de pared.
    Devuelve (ranges, upper) o (None, upper) si el escaneo no es utilizable.
    """
    r = np.asarray(raw, dtype=float).copy()
    upper = float(max_range)
    if math.isfinite(range_max) and range_max > 0.0:
        upper = min(upper, float(range_max))
    lower = 0.02
    if math.isfinite(range_min) and range_min > 0.0:
        lower = max(lower, float(range_min))

    if r.size < 5:
        return None, upper

    r[np.isposinf(r)] = upper
    finite = np.isfinite(r)
    if np.count_nonzero(finite) < 5:
        return None, upper
    r[finite & (r < lower)] = lower
    if not np.all(finite):
        idx = np.arange(r.size)
        r[~finite] = np.interp(idx[~finite], idx[finite], r[finite])
    r = np.clip(r, lower, upper)

    left = np.concatenate(([r[0]], r[:-1]))
    right = np.concatenate((r[1:], [r[-1]]))
    r = np.median(np.vstack((left, r, right)), axis=0)
    return r, upper


def extend_disparities(r, angle_increment, half_width, threshold):
    """
    En cada salto brusco de distancia (borde de pared / esquina interior),
    extiende la distancia corta hacia el lado lejano tantos rayos como ocupa
    medio ancho de vehículo + margen a esa distancia. Así el GAP resultante ya
    es transitable por el coche y no se cortan vértices interiores.
    """
    out = r.copy()
    diffs = np.diff(r)
    idxs = np.where(np.abs(diffs) > threshold)[0]
    inc = abs(angle_increment)
    n_total = r.size
    for i in idxs:
        if diffs[i] > 0.0:
            d = r[i]
            n = int(math.ceil(math.atan2(half_width, max(d, 0.05)) / inc))
            a, b = i + 1, min(n_total, i + 1 + n)
        else:
            d = r[i + 1]
            n = int(math.ceil(math.atan2(half_width, max(d, 0.05)) / inc))
            a, b = max(0, i + 1 - n), i + 1
        if b > a:
            out[a:b] = np.minimum(out[a:b], d)
    return out


def apply_bubble(r, angle_increment, bubble_radius):
    """Anula los rayos alrededor del punto más cercano (burbuja de seguridad)."""
    out = r.copy()
    positive = np.where(out > 0.0, out, np.inf)
    ci = int(np.argmin(positive))
    cd = float(out[ci])
    if not math.isfinite(cd) or cd <= 0.0:
        return out, ci, 0.0
    n = int(math.ceil(math.atan2(bubble_radius, max(cd, 0.05)) / abs(angle_increment)))
    out[max(0, ci - n):min(out.size, ci + n + 1)] = 0.0
    return out, ci, cd


def select_gap_target(r, theta, prev_target, params):
    """
    Busca todos los GAPs (tramos con distancia > gap_min_depth) y elige uno con
    una puntuación adimensional:
        profundidad * (1 + w_width*ancho) * (1 - w_center*|ángulo|/pi)
        / (1 + w_change*|ángulo - ángulo_anterior|)  * (1 + histéresis si es el mismo GAP)
    Dentro del GAP elegido, el objetivo es el centro del tramo contiguo más largo
    con distancia >= depth_ratio * profundidad máxima del GAP.
    Devuelve (índice_objetivo, número_de_gaps, mismo_gap).
    """
    gaps = contiguous_runs(r > params['gap_min_depth'])
    gaps = [g for g in gaps if (g[1] - g[0] + 1) >= params['gap_min_rays']]

    if not gaps:
        return int(np.argmax(r)), 0, False

    best = None
    best_score = -np.inf
    best_same = False
    for (s, e) in gaps:
        seg = r[s:e + 1]
        dmax = float(np.max(seg))
        deep_runs = contiguous_runs(seg >= params['depth_ratio'] * dmax)
        ds, de = max(deep_runs, key=lambda t: t[1] - t[0])
        t_idx = s + (ds + de) // 2
        t_ang = float(theta[t_idx])
        width = abs(float(theta[e] - theta[s]))

        lo, hi = min(theta[s], theta[e]), max(theta[s], theta[e])
        same = prev_target is not None and lo <= prev_target <= hi

        score = dmax * (1.0 + params['w_width'] * min(width, 0.6))
        score *= max(0.05, 1.0 - params['w_center'] * abs(t_ang) / math.pi)
        if prev_target is not None:
            score /= (1.0 + params['w_change'] * abs(t_ang - prev_target))
        if same:
            score *= (1.0 + params['hysteresis'])

        if score > best_score:
            best_score = score
            best = t_idx
            best_same = same

    return int(best), len(gaps), best_same


def find_objects(raw, angles, xs, ys, upper, max_range, max_size, contrast, max_angle,
                 min_points=2, min_facing=0.0):
    """Objetos pequeños (otro coche, una caja) en el escaneo, separados de las paredes.

    Un objeto es un tramo continuo de lecturas cercanas cuyo tamaño total es menor que
    max_size, separado de lo que lo rodea, y con fondo más lejano al menos a un lado.
    Las paredes forman tramos largos, por lo que no se detectan como objetos.
    min_facing descarta trozos de pared vistos de costado: en una superficie que mira al
    LiDAR (la trasera o el lateral de un coche) los rayos caen juntos, (n_rayos * paso_angular
    * distancia) / tamaño ~ 1; en un trozo de pared casi paralelo al rayo ese valor es pequeño.
    Devuelve una lista de (indices, x_min, y_min, y_max).
    """
    sel = (np.abs(angles) <= max_angle) & (raw < max_range)
    idx = np.nonzero(sel)[0]
    objs = []
    if idx.size == 0:
        return objs
    cut = np.nonzero((np.diff(idx) > 1) | (np.abs(np.diff(raw[idx])) > 0.25))[0]
    starts = np.r_[0, cut + 1]
    ends = np.r_[cut, idx.size - 1]
    for a, b in zip(starts, ends):
        run = idx[a:b + 1]
        if run.size < max(2, min_points):
            continue
        px, py = xs[run], ys[run]
        size = math.hypot(float(np.ptp(px)), float(np.ptp(py)))
        if size > max_size:
            continue
        rmin = float(np.min(raw[run]))
        if min_facing > 0.0 and size > 1e-3:
            inc = abs(float(angles[1] - angles[0]))
            if (run.size - 1) * inc * float(np.mean(raw[run])) / size < min_facing:
                continue
        before = raw[run[0] - 1] if run[0] > 0 else upper
        after = raw[run[-1] + 1] if run[-1] + 1 < raw.size else upper
        # fondo claramente más lejano al menos a un lado (al otro lado puede haber una pared)
        if before > rmin + contrast or after > rmin + contrast:
            objs.append((run, float(np.min(px)), float(np.min(py)), float(np.max(py))))
    return objs


def arc_free_distance(xs, ys, kappa, half_width, max_len):
    """
    Longitud libre a lo largo del arco de curvatura kappa que recorrería el
    vehículo (corredor de ancho 2*half_width). Con kappa ~ 0 es el corredor recto.
    xs, ys: puntos del LiDAR en el marco del vehículo (x hacia delante).
    """
    if abs(kappa) < 1e-3:
        on = (xs > 0.0) & (np.abs(ys) < half_width)
        return float(np.min(xs[on])) if np.any(on) else float(max_len)
    R = 1.0 / kappa
    dist_c = np.hypot(xs, ys - R)
    on = np.abs(dist_c - abs(R)) < half_width
    phi = np.arctan2(xs, math.copysign(1.0, R) * (R - ys))
    on &= phi > 0.0
    if not np.any(on):
        return float(max_len)
    s = abs(R) * phi[on]
    return float(min(np.min(s), max_len))


def yaw_stability_eig(v, a, vp, g=9.81):
    """
    Parte real máxima de los autovalores del subsistema (velocidad de guiñada, deriva)
    del modelo single-track de f1tenth_gym con aceleración longitudinal a.
    > 0 significa que el coche es inestable (trompo) con esa frenada a esa velocidad.
    """
    mu, Cf, Cr = vp['mu'], vp['C_Sf'], vp['C_Sr']
    lf, lr, h, m, I = vp['lf'], vp['lr'], vp['h_cg'], vp['mass'], vp['I_z']
    L = lf + lr
    Ff = g * lr - a * h
    Fr = g * lf + a * h
    A = np.array([
        [-mu * m / (v * I * L) * (lf ** 2 * Cf * Ff + lr ** 2 * Cr * Fr),
         mu * m / (I * L) * (lr * Cr * Fr - lf * Cf * Ff)],
        [mu / (v ** 2 * L) * (Cr * Fr * lr - Cf * Ff * lf) - 1.0,
         -mu / (v * L) * (Cr * Fr + Cf * Ff)]])
    return float(np.max(np.linalg.eigvals(A).real))


def yaw_gain_ss(v, vp, g=9.81):
    """
    Ganancia de guiñada en régimen permanente r/delta del modelo single-track del gym
    (sin aceleración longitudinal). El modelo cinemático da v/L; el del gym da menos
    (subviraje), cada vez menos con la velocidad.
    """
    mu, Cf, Cr = vp['mu'], vp['C_Sf'], vp['C_Sr']
    lf, lr, h, m, I = vp['lf'], vp['lr'], vp['h_cg'], vp['mass'], vp['I_z']
    L = lf + lr
    Ff, Fr = g * lr, g * lf
    A = np.array([
        [-mu * m / (v * I * L) * (lf ** 2 * Cf * Ff + lr ** 2 * Cr * Fr),
         mu * m / (I * L) * (lr * Cr * Fr - lf * Cf * Ff)],
        [mu / (v ** 2 * L) * (Cr * Fr * lr - Cf * Ff * lf) - 1.0,
         -mu / (v * L) * (Cr * Fr + Cf * Ff)]])
    B = np.array([mu * m / (I * L) * lf * Cf * Ff, mu / (v * L) * Cf * Ff])
    return float(np.linalg.solve(A, -B)[0])


def build_brake_tables(vp, a_cap, factor, v_top, margin=0.0):
    """
    Para cada velocidad calcula la deceleración máxima que mantiene estable la guiñada
    (bisección sobre el autovalor), la multiplica por 'factor' y la limita a a_cap.
    Devuelve (v_grid, a_dec(v), S(v)) donde S(v) = integral de u/a(u) du es la distancia
    necesaria para frenar de v a 0; distancia de v1 a v2 = S(v1) - S(v2).
    """
    v_grid = np.linspace(0.0, max(v_top, 1.0) + 2.0, 121)
    a_dec = np.empty_like(v_grid)
    for i, v in enumerate(v_grid):
        if v < 0.6:
            a_dec[i] = a_cap
            continue
        lo, hi = 0.0, 9.81
        if yaw_stability_eig(v, -hi, vp) < -margin:
            a_st = hi
        elif yaw_stability_eig(v, 0.0, vp) >= -margin:
            a_st = 0.0
        else:
            for _ in range(40):
                mid = 0.5 * (lo + hi)
                if yaw_stability_eig(v, -mid, vp) < -margin:
                    lo = mid
                else:
                    hi = mid
            a_st = lo
        a_dec[i] = min(a_cap, max(0.8, factor * a_st))
    integrand = v_grid / a_dec
    S = np.concatenate(([0.0], np.cumsum(0.5 * (integrand[1:] + integrand[:-1]) * np.diff(v_grid))))
    return v_grid, a_dec, S


def lp_alpha(dt, tau):
    """Coeficiente de un filtro de primer orden con constante de tiempo tau."""
    if tau <= 1e-6:
        return 1.0
    return 1.0 - math.exp(-dt / tau)


# ============================================================================
# Nodo ROS 2
# ============================================================================

class ReactiveFollowGap(Node):

    def __init__(self):
        super().__init__('reactive_node')

        defaults = {
            # --- Velocidad ---
            'max_speed': 12.0,            # m/s, techo en rectas
            'min_speed': 2.5,             # m/s, piso en marcha normal
            'corner_min_speed': 8.0,      # m/s, velocidad objetivo en curvas abiertas
            'corner_min_sharp': 6.25,     # m/s, velocidad objetivo en horquillas (<0 = igual que corner_min_speed)
            'sharp_dist_lo': 4.0,         # m, si max(d_free, d_path) <= esto -> horquilla
            'sharp_dist_hi': 12.0,         # m, si max(d_free, d_path) >= esto -> curva abierta
            'a_lat_max': 10.0,             # m/s^2, límite de aceleración lateral (mu*g ~ 10.3 en el sim)
            'a_brake': 7.5,               # m/s^2, techo de deceleración (a baja velocidad)
            'brake_stab_factor': 1.0,     # fracción de la deceleración límite de estabilidad
            'brake_stab_margin': 0.0,     # 1/s, amortiguamiento mínimo exigido a la guiñada al frenar
            'brake_kp': 19.02,            # ganancia de frenado del gym: 10*a_max/|v_min|
            'aeb_stab_factor': 1.5,       # el AEB frena algo más fuerte que el límite, nunca a fondo
            'understeer_comp': 1.0,       # 1 = compensa todo el subviraje del modelo, 0 = nada
            'understeer_max': 2.5,        # factor máximo de compensación
            'yaw_gain': 0.14,             # rad de volante por rad/s de error de guiñada
            'yaw_corr_max': 0.08,         # rad, corrección máxima por guiñada
            'yaw_deadband': 1.0,          # rad/s, banda muerta cuando NO se frena (evita zigzag)
            'yaw_brake_on': 1.5,          # m/s^2, deceleración a partir de la cual la corrección es completa
            'yaw_washout_tau': 0.30,
            'yaw_model_delay': 0.03,      # s, retardo volante->guiñada (buffer de dirección del gym)
            'yaw_model_tau': 0.08,        # s, inercia de guiñada del modelo de referencia      # s, solo se corrigen errores rápidos (derrapes), no los permanentes
            # Parámetros del vehículo en el gym (para el cálculo de estabilidad)
            'mu': 1.0489, 'C_Sf': 4.718, 'C_Sr': 5.4562, 'lf': 0.15875, 'lr': 0.17145,
            'h_cg': 0.074, 'mass': 3.74, 'I_z': 0.04712,
            'brake_margin': 0.6,          # m, colchón en la distancia de frenado
            'turn_gain': 1.3,             # radio disponible = turn_gain * (d_path - margen)
            'accel_headroom': 3.0,        # m/s, cuánto puede el comando superar a la velocidad real
            # --- Dirección ---
            'steering_mode': 'arc',       # 'arc' (geometría de arco) o 'proportional'
            'prop_gain': 1.0,             # solo en modo proportional
            'wheelbase': 0.3302,          # m, lf + lr del sim
            'max_steering': 0.4189,       # rad, s_max del sim
            'steering_rate': 3.2,         # rad/s, sv_max del sim
            'steer_tau': 0.012,           # s, filtro de dirección
            'target_tau': 0.030,          # s, filtro del ángulo objetivo
            'ld_min': 1.2,                # m, distancia de arco mínima
            'ld_gain': 0.25,              # s, ld = ld_min + ld_gain*v
            'ld_max': 5.0,                # m
            # --- LiDAR / GAP ---
            'fov_deg': 90.0,              # grados a cada lado para buscar GAP
            'max_range': 25.0,            # m, recorte del LiDAR (el sim entrega ~30 m)
            'disparity_threshold': 0.35,  # m
            'car_half_width': 0.155,      # m (ancho 0.31 en el sim)
            'safety_margin': 0.08,        # m, margen lateral base
            'safety_margin_gain': 0.008,  # m por m/s, margen extra con la velocidad
            'bubble_radius': 0.30,        # m
            'gap_min_depth': 1.8,         # m
            'gap_min_rays': 4,
            'depth_ratio': 0.80,
            'w_width': 0.5,
            'w_center': 0.30,
            'w_change': 0.6,
            'hysteresis': 0.15,
            'free_window_deg': 3.0,
            # --- Centrado lateral (corrige la deriva en rectas) ---
            'center_gain': 1.0,           # rad equivalentes por (m de desvío / Ld)
            'center_ld_min': 1.5,         # m
            'center_ld_gain': 0.20,       # s
            'center_ld_max': 3.5,         # m
            'center_fade_deg': 20.0,      # el centrado se apaga si el objetivo del GAP supera este ángulo
            'center_max_offset': 1.0,     # m, desvío máximo considerado
            # --- Trazada: abrirse hacia el exterior antes de la curva ---
            'line_gain': -0.7,             # m de desvío por unidad de 'intensidad' de la curva que viene (0 = centro)
            'line_angle_ref_deg': 12.0,   # ángulo del GAP que se considera curva "plena"
            'line_max_offset': 0.5,       # m, desvío máximo respecto al centro
            'line_wall_keep': 0.45,       # m, distancia mínima a la pared exterior que se respeta
            'line_tau': 0.15,             # s, suavizado del desvío objetivo
            'wall_guard': 0.45,           # m, distancia lateral (desde el centro) que activa la protección
            'wall_guard_speed': 5.0,      # m/s, velocidad máxima cuando una pared está a 0.25 m       # ventana alrededor del objetivo para medir distancia libre
            # --- Seguridad ---
            'ttc_aeb': 0.20,              # s, frenado de emergencia
            'aeb_corridor_extra': 0.05,   # m, ancho extra del corredor frontal del AEB
            'stuck_front': 0.45,          # m
            'stuck_time': 0.6,            # s
            'reverse_time': 0.8,          # s
            'reverse_speed': -1.5,        # m/s
            # --- Odometría / registro ---
            'odom_topic': '/ego_racecar/odom',
            'odom_timeout': 0.25,         # s
            'log_enabled': True,
            'log_dir': '~/f1tenth_logs',
            'log_rate_hz': 50.0,
            'run_tag': 'SEPANG',
            # --- Adelantamiento (prueba con dos coches). Desactivado = comportamiento de la parte 1
            'overtake_enabled': False,
            'ot_range': 9.0,              # m, alcance de detección de objetos
            'ot_max_size': 0.85,          # m, tamaño máximo de un objeto (un coche mide ~0.6 x 0.3)
            'ot_contrast': 0.8,           # m, el fondo a ambos lados debe estar al menos esto más lejos
            'ot_min_points': 4,           # rayos mínimos de un objeto
            'ot_min_facing': 0.5,         # descarta trozos de pared vistos de costado (ver find_objects)
            'ot_margin': 0.18,            # m, separación lateral deseada al pasar
            'ot_max_target_deg': 20.0,    # solo se adelanta si el GAP está casi recto
            'ot_follow_gap': 0.8,         # m, distancia mínima al seguir a un coche
            'ot_follow_time': 0.30,       # s, distancia de seguimiento adicional por velocidad
            'ot_moving_speed': 1.5,       # m/s, a partir de aquí el objeto se considera un coche en marcha
            'ot_lane_extra': 12.0,        # m, carril libre necesario por delante del coche para pasar
            'ot_lane_time': 1.0,          # s, carril libre adicional por velocidad
            'ot_follow_decel': 3.0,       # m/s², deceleración usada para seguir
            'ot_pass_side': 1,            # lado de adelantamiento: 1 izquierda, -1 derecha, 0 automático
            'ot_pass_offset': 0.40,       # m, desvío lateral al adelantar por un lado fijo
            'ot_tau': 0.12,               # s, suavizado del desvío lateral de adelantamiento
            'ot_near_angle_deg': 110.0,   # deg, ángulo a cada lado en que se buscan coches
            'ot_near_back': -1.0,         # m, un coche hasta esta distancia detrás cuenta como "cerca"
            'ot_hold': 1.5,               # s, se mantiene el lado de paso este tiempo después de ver un coche delante
            'ot_commit': 1.0,             # m, con el otro coche a menos de esto ya se está pasando: no se aborta
            'lane_bias': 0.0,             # m, desvío lateral fijo (+ izquierda). 0 = parte 1
            # --- Obstáculos fijos. 0 = comportamiento de la parte 1
            'obs_margin_extra': 0.0,      # m, margen lateral extra mientras hay un obstáculo pequeño cerca
            'obs_range': 8.0,             # m, distancia a la que se buscan obstáculos pequeños
            'obs_angle_deg': 45.0,        # deg, ángulo a cada lado del frente donde se buscan
            'obs_hold': 0.6,              # s, tiempo que se mantiene el margen extra después de verlo
            'race_laps': 10,              # vueltas de la carrera; al completarlas el carro se detiene (0 = no detener)
            'lap_radius': 1.5,            # m, radio de la "meta" alrededor de la pose inicial
            'lap_min_distance': 80.0,     # m recorridos antes de aceptar otra vuelta
            # --- Visualización en RViz ---
            'viz_enabled': True,
            'viz_topic': '/trayectoria',
            'viz_frame': 'map',           # se reemplaza por el frame de la odometría si llega
            'viz_laser_frame': 'ego_racecar/laser',
            'viz_point_spacing': 0.30,    # m entre puntos de la línea
            'viz_rate_hz': 1.0,
            'viz_width': 0.08,            # m, grosor de la línea
            'viz_text_color': [1.0, 0.15, 0.15],  # RGB del texto (rojo)
            'viz_text_size': 1.2,         # m, alto de las letras
            'viz_text_dy': 2.5,           # m, posición del texto respecto a la salida (eje y)
            'viz_speed_colors': True,     # línea coloreada por velocidad; False = color del texto
            'viz_show_target': False,     # esfera en el punto objetivo del GAP (True para depurar)
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        self.p['steering_mode'] = str(self.p['steering_mode']).lower()
        self.p['gap_min_rays'] = int(self.p['gap_min_rays'])

        # Tablas de frenado estable (dependen solo de parámetros)
        self.v_grid, self.a_dec_tab, self.S_tab = build_brake_tables(
            self.p, self.p['a_brake'], self.p['brake_stab_factor'], self.p['max_speed'],
            self.p['brake_stab_margin'])
        us = []
        for v in self.v_grid:
            if v < 1.0:
                us.append(1.0)
            else:
                f = (v / self.p['wheelbase']) / max(yaw_gain_ss(v, self.p), 1e-3)
                f = 1.0 + self.p['understeer_comp'] * (f - 1.0)
                us.append(float(np.clip(f, 1.0, self.p['understeer_max'])))
        self.us_tab = np.array(us)
        self.get_logger().info(
            'Compensación de subviraje: ' + ' | '.join(
                f'{v:.0f} m/s -> x{np.interp(v, self.v_grid, self.us_tab):.2f}' for v in (4, 6, 8, 10, 12)))
        self.get_logger().info(
            'Frenada estable: ' + ' | '.join(
                f'{v:.0f} m/s -> {np.interp(v, self.v_grid, self.a_dec_tab):.1f} m/s2'
                for v in (4, 6, 8, 10, 12)))

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.p['odom_topic'], self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Visualización: línea de la vuelta actual coloreada por velocidad, vuelta anterior
        # en gris, texto con tiempos/distancia y el punto objetivo del GAP.
        self.viz_cur = []        # [(x, y)] solo para medir separación
        self.viz_cur_pts = []    # Point ya construidos (no se reconstruyen al publicar)
        self.viz_cur_cols = []   # ColorRGBA ya construidos
        self.viz_prev_pts = []
        self.viz_prev_dirty = False
        self.viz_target = None   # (x, y) en el frame del LiDAR
        self.viz_target_cleared = False
        self.last_lap_time = None
        self.last_lap_dist = None
        self.viz_pub = None
        if self.p['viz_enabled']:
            self.viz_pub = self.create_publisher(MarkerArray, self.p['viz_topic'], 10)
            self.create_timer(1.0 / max(self.p['viz_rate_hz'], 0.5), self._publish_viz)

        # Estado de control
        self.prev_stamp = None
        self.target_angle_f = 0.0
        self.prev_target = None
        self.y_ref = 0.0
        self.ot_prev_x = None
        self.ot_vrel = 0.0
        self.ot_side = 0
        self.ot_active = False
        self.ot_near_timer = 0.0
        self.obs_timer = 0.0
        self.steering = 0.0
        self.last_cmd_speed = 0.0

        # Estado de odometría
        self.v_real = 0.0
        self.yaw_rate = 0.0
        self.yaw_err_lp = 0.0
        self.braking = False
        self.steer_hist = []              # (tiempo, volante comandado) para el modelo de referencia
        self.r_model = 0.0
        self.last_odom_time = None
        self.pos = None
        self.warned_odom = False

        # Recuperación
        self.stuck_accum = 0.0
        self.reverse_left = 0.0

        # Vueltas
        self.start_pos = None
        self.lap_start = None
        self.race_done = False
        self.lap_dist = 0.0
        self.lap_count = 0
        self.best_lap = None
        self._reset_lap_stats()

        # Registro
        self.last_log_write = 0.0
        self.last_info = 0.0
        self.csv_file = None
        self.csv_writer = None
        self.laps_writer = None
        self.laps_file = None
        if self.p['log_enabled']:
            self._open_logs()

        self.get_logger().info(
            f"FTG {self.p['run_tag']} iniciado | max_speed={self.p['max_speed']:.1f} "
            f"a_lat={self.p['a_lat_max']:.1f} a_brake={self.p['a_brake']:.1f} "
            f"modo={self.p['steering_mode']}")

    # ------------------------------------------------------------------ util

    def _reset_lap_stats(self):
        self.lap_vmax = 0.0
        self.lap_min_clear = float('inf')
        self.lap_steer_var = 0.0
        self.lap_aeb = 0
        self.lap_reverse = 0

    def _open_logs(self):
        try:
            d = os.path.expanduser(self.p['log_dir'])
            os.makedirs(d, exist_ok=True)
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            path = os.path.join(d, f"{self.p['run_tag']}_{stamp}.csv")
            self.csv_file = open(path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                't', 'lap', 'x', 'y', 'v_real', 'v_cmd', 'v_curv', 'v_brake', 'v_turn',
                'steer_cmd', 'steer_des', 'target_ang', 'target_depth', 'd_free', 'd_path',
                'front_x', 'min_clear', 'd_left', 'd_right', 'center_corr', 'yaw_rate', 'yaw_corr', 'n_gaps', 'same_gap', 'aeb', 'mode'])
            laps_path = os.path.join(d, 'vueltas.csv')
            new = not os.path.exists(laps_path)
            self.laps_file = open(laps_path, 'a', newline='')
            self.laps_writer = csv.writer(self.laps_file)
            if new:
                self.laps_writer.writerow([
                    'fecha', 'run_tag', 'max_speed', 'a_lat_max', 'a_brake', 'steering_mode',
                    'vuelta', 'tiempo_s', 'v_max', 'min_clearance_m',
                    'variacion_total_volante_rad', 'eventos_aeb', 'reversas',
                    'distancia_m', 'vel_media_m_s'])
            self.get_logger().info(f'Registro: {path}')
        except OSError as e:
            self.get_logger().warn(f'No se pudo abrir el registro: {e}')
            self.csv_writer = None
            self.laps_writer = None

    def publish_drive(self, speed, steering):
        if not rclpy.ok():
            return
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        try:
            self.drive_pub.publish(msg)
        except Exception:
            return
        self.last_cmd_speed = float(speed)

    def _speed_estimate(self, now):
        if self.last_odom_time is not None and (now - self.last_odom_time) < self.p['odom_timeout']:
            return self.v_real
        if not self.warned_odom:
            self.get_logger().warn('Sin odometría reciente: se usa la velocidad comandada como estimación.')
            self.warned_odom = True
        return max(0.0, self.last_cmd_speed)

    # ------------------------------------------------------------- odometría

    def odom_callback(self, msg):
        now = time.monotonic()
        self.last_odom_time = now
        self.warned_odom = False
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.v_real = math.copysign(math.hypot(vx, vy), vx if abs(vx) > 1e-6 else 1.0)
        self.yaw_rate = msg.twist.twist.angular.z
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if msg.header.frame_id:
            self.p['viz_frame'] = msg.header.frame_id
        if self.p['viz_enabled'] and self.lap_start is not None:
            if not self.viz_cur or math.hypot(x - self.viz_cur[-1][0], y - self.viz_cur[-1][1]) >= self.p['viz_point_spacing']:
                self.viz_cur.append((x, y))
                self.viz_cur_pts.append(Point(x=x, y=y, z=0.05))
                if self.p['viz_speed_colors']:
                    self.viz_cur_cols.append(self._speed_color(max(0.0, self.v_real), self.p['max_speed']))
                else:
                    tc = list(self.p['viz_text_color']) + [1.0, 1.0, 1.0]
                    self.viz_cur_cols.append(ColorRGBA(r=float(tc[0]), g=float(tc[1]), b=float(tc[2]), a=1.0))

        if self.start_pos is None:
            self.start_pos = (x, y)
        if self.pos is not None:
            self.lap_dist += math.hypot(x - self.pos[0], y - self.pos[1])
        self.pos = (x, y)

        if self.race_done:
            return

        if self.lap_start is None:
            if abs(self.v_real) > 0.1:
                self.lap_start = now
                self.lap_dist = 0.0
                self._reset_lap_stats()
            return

        self.lap_vmax = max(self.lap_vmax, self.v_real)
        d_start = math.hypot(x - self.start_pos[0], y - self.start_pos[1])
        if d_start < self.p['lap_radius'] and self.lap_dist > self.p['lap_min_distance']:
            lap_time = now - self.lap_start
            self.lap_count += 1
            if self.best_lap is None or lap_time < self.best_lap:
                self.best_lap = lap_time
            v_avg = self.lap_dist / max(lap_time, 1e-3)
            self.get_logger().info(
                f'VUELTA {self.lap_count}: {lap_time:.2f} s | mejor {self.best_lap:.2f} s | '
                f'distancia {self.lap_dist:.1f} m | vel. media {v_avg:.2f} m/s | '
                f'vmax {self.lap_vmax:.1f} m/s | holgura min {self.lap_min_clear:.2f} m | '
                f'var. volante {self.lap_steer_var:.1f} rad | AEB {self.lap_aeb} | '
                f'reversas {self.lap_reverse}')
            if self.laps_writer is not None:
                self.laps_writer.writerow([
                    datetime.now().isoformat(timespec='seconds'), self.p['run_tag'],
                    self.p['max_speed'], self.p['a_lat_max'], self.p['a_brake'],
                    self.p['steering_mode'], self.lap_count, f'{lap_time:.3f}',
                    f'{self.lap_vmax:.2f}', f'{self.lap_min_clear:.3f}',
                    f'{self.lap_steer_var:.2f}', self.lap_aeb, self.lap_reverse,
                    f'{self.lap_dist:.2f}', f'{v_avg:.3f}'])
                self.laps_file.flush()
            self.last_lap_time = lap_time
            self.last_lap_dist = self.lap_dist
            self.viz_prev_pts = [Point(x=q.x, y=q.y, z=0.02) for q in self.viz_cur_pts]
            self.viz_prev_dirty = True
            self.viz_cur = []
            self.viz_cur_pts = []
            self.viz_cur_cols = []
            self.lap_start = now
            self.lap_dist = 0.0
            self._reset_lap_stats()
            if self.p['race_laps'] > 0 and self.lap_count >= self.p['race_laps']:
                self.race_done = True
                self.get_logger().warn(
                    f'CARRERA TERMINADA ({self.lap_count} vueltas, mejor {self.best_lap:.2f} s): '
                    f'deteniendo el vehículo')

    # ----------------------------------------------------------------- LiDAR

    def lidar_callback(self, msg):
        p = self.p
        now = time.monotonic()

        # dt a partir del sello del mensaje (o reloj monotónico si no hay sello)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            stamp = now
        if self.prev_stamp is None or stamp <= self.prev_stamp:
            dt = 0.01
        else:
            dt = stamp - self.prev_stamp
        self.prev_stamp = stamp
        dt = float(np.clip(dt, 0.001, 0.1))

        inc = msg.angle_increment
        if not math.isfinite(inc) or abs(inc) < 1e-8:
            self.publish_drive(0.0, 0.0)
            return

        raw, upper = preprocess_ranges(msg.ranges, msg.range_min, msg.range_max, p['max_range'])
        if raw is None:
            self.publish_drive(0.0, 0.0)
            return

        angles = msg.angle_min + np.arange(raw.size) * inc
        v = self._speed_estimate(now)
        v_pos = max(0.0, v)

        # ---------- Seguridad: corredor sobre el arco actual (AEB) y holgura mínima
        xs = raw * np.cos(angles)
        ys = raw * np.sin(angles)
        kappa_now = math.tan(self.steering) / p['wheelbase'] / float(np.interp(v_pos, self.v_grid, self.us_tab))
        front_x = arc_free_distance(xs, ys, kappa_now,
                                    p['car_half_width'] + p['aeb_corridor_extra'], upper)
        min_clear = float(np.min(raw[np.abs(angles) <= math.radians(100)]))
        if self.lap_start is not None:
            self.lap_min_clear = min(self.lap_min_clear, min_clear)

        # ---------- Recuperación anti-atasco (por tiempo y con velocidad real)
        if self.reverse_left > 0.0:
            self.reverse_left -= dt
            rev_steer = -math.copysign(0.6 * p['max_steering'], self.target_angle_f) \
                if abs(self.target_angle_f) > 0.05 else 0.0
            self.steering = rev_steer
            self.publish_drive(p['reverse_speed'], rev_steer)
            return
        if front_x < p['stuck_front'] and abs(v) < 0.2 and self.lap_start is not None \
                and not self.race_done:
            self.stuck_accum += dt
        else:
            self.stuck_accum = 0.0
        if self.stuck_accum > p['stuck_time']:
            self.stuck_accum = 0.0
            self.reverse_left = p['reverse_time']
            self.lap_reverse += 1
            self.get_logger().warn('Atasco detectado: maniobra de reversa.')
            self.publish_drive(p['reverse_speed'], 0.0)
            return

        # ---------- Objetos (otro coche, cajas): se separan de las paredes
        obj = None
        self.ot_near = False
        raw_w, xs_w, ys_w = raw, xs, ys      # solo paredes (para centrado y frenado)
        raw_ftg = raw                        # lo que ve el Follow the Gap
        if p['overtake_enabled']:
            objs = find_objects(raw, angles, xs, ys, upper, p['ot_range'], p['ot_max_size'],
                                p['ot_contrast'], math.radians(p['ot_near_angle_deg']),
                                p['ot_min_points'], p['ot_min_facing'])
            self.ot_nobj = len(objs)
            if objs:
                raw_w = raw.copy()
                for run, _, _, _ in objs:
                    raw_w[run] = upper
                xs_w = raw_w * np.cos(angles)
                ys_w = raw_w * np.sin(angles)
                self.ot_near = any(o[1] > p['ot_near_back'] for o in objs)
                ahead = [o for o in objs if o[1] > 0.2]
                if ahead:
                    obj = min(ahead, key=lambda o: o[1])
                    # velocidad del objeto estimada a partir de la variación de su distancia
                    ox = obj[1]
                    if self.ot_prev_x is not None and abs(ox - self.ot_prev_x) < 1.0:
                        self.ot_vrel += lp_alpha(dt, 0.2) * ((ox - self.ot_prev_x) / dt - self.ot_vrel)
                    else:
                        # objeto nuevo: se supone quieto hasta que su distancia muestre que avanza
                        self.ot_vrel = -v_pos
                    self.ot_prev_x = ox
                    hw0 = p['car_half_width'] + p['ot_margin']
                    moving = (v_pos + self.ot_vrel) > p['ot_moving_speed']
                    in_lane0 = obj[2] < hw0 and obj[3] > -hw0
                    # Un coche que avanza delante en el mismo carril no es un obstáculo para
                    # elegir el GAP: se sigue la pista y la velocidad se ajusta para seguirlo.
                    if moving and in_lane0 and self.ot_side == 0:
                        raw_ftg = raw.copy()
                        raw_ftg[obj[0]] = upper

            # Mantener el lado de paso un tiempo después de ver el coche: al terminar el
            # adelantamiento el otro coche queda al lado o detrás (fuera de la vista del LiDAR)
            # y no hay que volver a su carril hasta dejarlo atrás.
            if obj is not None and obj[3] > -1.0 and obj[2] < 1.0 and \
                    v_pos + self.ot_vrel > p['ot_moving_speed']:
                # solo un coche que avanza, visto delante y dentro de la pista, inicia la espera
                # (los trozos de pared quietos no)
                self.ot_near_timer = p['ot_hold']
            else:
                self.ot_near_timer = max(0.0, self.ot_near_timer - dt)
            # "cerca" solo por un coche en marcha visto hace poco; los trozos de pared sueltos
            # (por ejemplo en la horquilla) ya no desplazan la trazada
            self.ot_near = self.ot_near_timer > 0.0

        # ---------- Obstáculos fijos: margen extra solo cuando hay uno pequeño cerca.
        # Así la burbuja de seguridad es más grande al pasar junto a un obstáculo, pero
        # en las curvas cerradas (sin obstáculos) el margen es el mismo de la parte 1.
        extra = 0.0
        if p['obs_margin_extra'] > 0.0:
            near = find_objects(raw, angles, xs, ys, upper, p['obs_range'], p['ot_max_size'],
                                p['ot_contrast'], math.radians(p['obs_angle_deg']),
                                p['ot_min_points'], p['ot_min_facing'])
            # obstáculo = objeto pequeño con fondo más lejano a ambos lados (las esquinas de
            # pared solo tienen fondo lejano a un lado)
            if any(raw[o[0][0] - 1] > float(np.min(raw[o[0]])) + p['ot_contrast'] and
                   raw[o[0][-1] + 1] > float(np.min(raw[o[0]])) + p['ot_contrast']
                   for o in near if 0 < o[0][0] and o[0][-1] + 1 < raw.size):
                self.obs_timer = p['obs_hold']
            else:
                self.obs_timer = max(0.0, self.obs_timer - dt)
            if self.obs_timer > 0.0:
                extra = p['obs_margin_extra']

        # ---------- Follow the Gap
        half_w = p['car_half_width'] + p['safety_margin'] + p['safety_margin_gain'] * v_pos + extra
        ext = extend_disparities(raw_ftg, inc, half_w, p['disparity_threshold'])

        fov = math.radians(p['fov_deg'])
        fmask = np.abs(angles) <= fov
        scan = ext[fmask]
        theta = angles[fmask]
        if scan.size < 5:
            self.publish_drive(0.0, 0.0)
            return

        scan_b, _, _ = apply_bubble(scan, inc, p['bubble_radius'])

        t_idx, n_gaps, same_gap = select_gap_target(scan_b, theta, self.prev_target, p)
        target_raw = float(theta[t_idx])
        target_depth = float(scan[t_idx])

        self.prev_target = target_raw
        self.viz_target = (target_depth * math.cos(target_raw), target_depth * math.sin(target_raw))

        # ---------- Centrado lateral: distancia perpendicular a cada pared
        sin_abs = np.abs(np.sin(angles))
        side_l = (angles >= math.radians(60)) & (angles <= math.radians(120))
        side_r = (angles <= -math.radians(60)) & (angles >= -math.radians(120))
        d_l = float(np.min(raw_w[side_l] * sin_abs[side_l])) if np.any(side_l) else upper
        d_r = float(np.min(raw_w[side_r] * sin_abs[side_r])) if np.any(side_r) else upper
        offset = float(np.clip(0.5 * (d_l - d_r), -p['center_max_offset'], p['center_max_offset']))
        d_side = min(d_l, d_r)
        fade = float(np.clip(1.0 - abs(target_raw) / math.radians(p['center_fade_deg']), 0.0, 1.0))
        if d_side < p['wall_guard']:
            fade = 1.0
        ld_c = float(np.clip(p['center_ld_min'] + p['center_ld_gain'] * v_pos,
                             p['center_ld_min'], p['center_ld_max']))
        # Desvío lateral objetivo (positivo = izquierda). Con line_gain = 0 es el centro.
        # Si el GAP se inclina a la izquierda (curva a la izquierda que viene), el objetivo
        # es la derecha (exterior), y viceversa. Se limita para no acercarse a la pared exterior.
        half_track = 0.5 * (d_l + d_r)
        sev = float(np.clip(target_raw / math.radians(p['line_angle_ref_deg']), -1.0, 1.0))
        y_goal = -p['line_gain'] * sev + p['lane_bias']
        if p['overtake_enabled'] and p['ot_pass_side'] != 0 and self.ot_near:
            # adelantamiento por un lado fijo: desplazarse hacia ese lado mientras haya un
            # coche cerca (delante o al lado)
            y_goal = p['ot_pass_side'] * p['ot_pass_offset']
        y_lim = max(0.0, min(p['line_max_offset'], half_track - p['line_wall_keep']))
        y_goal = float(np.clip(y_goal, -y_lim, y_lim))
        self.y_ref += lp_alpha(dt, p['line_tau']) * (y_goal - self.y_ref)
        # offset > 0 significa que el coche está a la derecha del centro (y_car = -offset)
        lat_err = float(np.clip(self.y_ref + offset, -p['center_max_offset'], p['center_max_offset']))
        center_corr = fade * math.atan2(p['center_gain'] * lat_err, ld_c)
        target_cmd = target_raw + center_corr

        # ---------- Adelantamiento: pasar por el lado libre del objeto de adelante
        v_follow = float('inf')
        self.ot_active = False
        if obj is not None:
            _, ox, oy_min, oy_max = obj
            v_obj = max(0.0, v_pos + self.ot_vrel)
            hw = p['car_half_width'] + p['ot_margin']
            in_lane = (oy_min < hw) and (oy_max > -hw)
            need = 2.0 * hw
            room_l = d_l - oy_max
            room_r = oy_min + d_r
            straight = abs(target_raw) <= math.radians(p['ot_max_target_deg'])

            # carril de paso medido en la dirección de la pista (la del GAP), no la del coche
            ct, st_ = math.cos(target_raw), math.sin(target_raw)
            xt = xs_w * ct + ys_w * st_
            yt = -xs_w * st_ + ys_w * ct

            def lane_free(y_c):
                m = (xt > 0.3) & (np.abs(yt - y_c) < p['car_half_width'] + 0.05)
                return float(np.min(xt[m])) if np.any(m) else upper

            side = 0
            y_pass = 0.0
            if p['ot_pass_side'] == 0 and straight and max(room_l, room_r) >= need:
                side = 1 if room_l >= room_r else -1
                # histéresis: mantener el lado elegido mientras siga habiendo espacio
                if self.ot_side == -side and (room_l if self.ot_side > 0 else room_r) >= need:
                    side = self.ot_side
                y_pass = 0.5 * (oy_max + d_l) if side > 0 else 0.5 * (oy_min - d_r)
                # no adelantar si el carril de paso se acaba pronto (viene una curva)
                if lane_free(y_pass) < ox + p['ot_lane_extra'] + p['ot_lane_time'] * v_pos:
                    side = 0
            self.ot_side = side
            if side != 0:
                ld_p = float(np.clip(ox + 1.0, 2.0, 6.0))
                target_cmd = math.atan2(y_pass, ld_p)
                self.y_ref = y_pass - offset
                self.ot_active = True
            gap_need = p['ot_follow_gap'] + p['ot_follow_time'] * v_pos
            v_behind = v_obj + math.sqrt(2.0 * p['ot_follow_decel'] * max(0.0, ox - gap_need))
            if in_lane:
                v_follow = v_behind
            elif p['ot_pass_side'] != 0 and ox > p['ot_commit']:
                # Lado fijo: solo ponerse al lado del otro coche si el carril de paso sigue libre
                # lo suficiente (si viene una curva, esperar detrás y pasar en la recta siguiente).
                y_pc = p['ot_pass_side'] * p['ot_pass_offset'] + offset
                if lane_free(y_pc) < ox + p['ot_lane_extra'] + p['ot_lane_time'] * v_pos:
                    v_follow = v_behind
            elif p['ot_pass_side'] == 0 and side == 0 and ox < 1.5:
                # al lado y sin carril libre por delante: quedarse detrás
                v_follow = max(0.0, v_obj - 0.5)
            self.ot_dbg = (round(ox, 2), round(oy_min, 2), round(oy_max, 2), round(v_obj, 1),
                           side, in_lane, round(room_l, 2), round(room_r, 2))
        else:
            self.ot_prev_x = None
            self.ot_vrel = 0.0
            self.ot_side = 0
            self.ot_dbg = None

        # Distancia libre en la dirección del objetivo
        win = max(1, int(math.ceil(math.radians(p['free_window_deg']) / abs(inc))))
        d_free = float(np.min(scan[max(0, t_idx - win):min(scan.size, t_idx + win + 1)]))
        if raw_w is not raw:
            scan_w = extend_disparities(raw_w, inc, half_w, p['disparity_threshold'])[fmask]
            d_free = float(np.min(scan_w[max(0, t_idx - win):min(scan_w.size, t_idx + win + 1)]))

        # ---------- Dirección
        self.target_angle_f += lp_alpha(dt, p['target_tau']) * (target_cmd - self.target_angle_f)
        alpha = self.target_angle_f
        L = p['wheelbase']
        if p['steering_mode'] == 'proportional':
            steer_des = p['prop_gain'] * alpha
        else:
            ld = float(np.clip(p['ld_min'] + p['ld_gain'] * v_pos, p['ld_min'], p['ld_max']))
            ld = max(0.5, min(ld, target_depth))
            kappa = 2.0 * math.sin(alpha) / ld
            steer_des = math.atan(L * kappa)
        steer_des = float(np.clip(steer_des, -p['max_steering'], p['max_steering']))
        steer_ftg = steer_des

        # Realimentación de guiñada: si el coche gira más (o menos) de lo que pide el
        # volante, se corrige antes de que el deslizamiento crezca.
        yaw_corr = 0.0
        odom_ok = self.last_odom_time is not None and (now - self.last_odom_time) < p['odom_timeout']
        if odom_ok and v_pos > 1.0:
            # Guiñada de referencia = la que produciría el volante comandado hace
            # 'yaw_model_delay' s, filtrada con la inercia de guiñada. Así la corrección
            # solo ve deslizamientos reales y no los propios cambios de volante.
            t_ref = stamp - p['yaw_model_delay']
            d_old = self.steering
            for (ts, ds) in reversed(self.steer_hist):
                d_old = ds
                if ts <= t_ref:
                    break
            r_target = v_pos * math.tan(d_old) / L / float(np.interp(v_pos, self.v_grid, self.us_tab))
            self.r_model += lp_alpha(dt, p['yaw_model_tau']) * (r_target - self.r_model)
            yaw_err = self.yaw_rate - self.r_model
            self.yaw_err_lp += lp_alpha(dt, p['yaw_washout_tau']) * (yaw_err - self.yaw_err_lp)
            e_hp = yaw_err - self.yaw_err_lp
            # Frenando, el coche es inestable por naturaleza: corrección completa.
            # Sin frenar es estable: banda muerta para no generar zigzag.
            db = 0.0 if self.braking else p['yaw_deadband']
            e_db = math.copysign(max(0.0, abs(e_hp) - db), e_hp)
            yaw_corr = float(np.clip(-p['yaw_gain'] * e_db,
                                     -p['yaw_corr_max'], p['yaw_corr_max']))
        else:
            self.yaw_err_lp = 0.0
            self.r_model = v_pos * math.tan(self.steering) / L / float(np.interp(v_pos, self.v_grid, self.us_tab))
        us_f = float(np.interp(v_pos, self.v_grid, self.us_tab))
        steer_des = float(np.clip(us_f * steer_ftg + yaw_corr, -p['max_steering'], p['max_steering']))

        prev_steer = self.steering
        s_f = prev_steer + lp_alpha(dt, p['steer_tau']) * (steer_des - prev_steer)
        max_change = p['steering_rate'] * dt
        s_f = float(np.clip(s_f, prev_steer - max_change, prev_steer + max_change))
        self.steering = float(np.clip(s_f, -p['max_steering'], p['max_steering']))
        self.steer_hist.append((stamp, self.steering))
        if len(self.steer_hist) > 64:
            self.steer_hist = self.steer_hist[-64:]
        if self.lap_start is not None:
            self.lap_steer_var += abs(self.steering - prev_steer)

        # ---------- Velocidad
        # La curvatura que limita la velocidad es la que pide el GAP (más la parte del
        # volante actual que no es corrección de guiñada), no la corrección de estabilidad.
        kappa_need = max(abs(math.tan(steer_ftg)),
                         abs(math.tan((self.steering - yaw_corr) / us_f))) / L
        v_curv = math.sqrt(p['a_lat_max'] / max(kappa_need, 1e-4))
        # Distancia libre a lo largo del arco planificado (frenado predictivo):
        # si el coche aún apunta a la pared exterior, esta distancia es corta aunque
        # el objetivo del GAP ya esté girado hacia la salida.
        kappa_plan = math.tan(steer_ftg) / L
        d_path = arc_free_distance(xs_w, ys_w, kappa_plan, half_w, upper)
        vg, S = self.v_grid, self.S_tab

        def v_reach(v_end, dist):
            # velocidad máxima desde la que se puede frenar (estable) hasta v_end en 'dist'
            return float(np.interp(np.interp(v_end, vg, S) + max(0.0, dist), S, vg))

        # (a) frenar hasta corner_min antes del final del espacio libre hacia el objetivo
        # Velocidad de curva adaptativa: si tanto el camino libre hacia el objetivo como el
        # arco planificado son cortos, viene una horquilla -> velocidad de horquilla.
        cm_sharp = p['corner_min_sharp'] if p['corner_min_sharp'] > 0 else p['corner_min_speed']
        cm = float(np.interp(max(d_free, d_path), [p['sharp_dist_lo'], p['sharp_dist_hi']],
                             [cm_sharp, p['corner_min_speed']]))
        v_brake = v_reach(cm, min(d_free, d_path) - p['brake_margin'])
        # (b) llegar a una velocidad de giro sqrt(a_lat*R) con espacio para girar con radio R
        d_avail = d_path - p['brake_margin']
        v_turn = v_reach(cm, d_avail)
        for R in np.linspace(0.5, max(0.6, p['turn_gain'] * d_avail), 12):
            v_corner = math.sqrt(p['a_lat_max'] * R)
            v_turn = max(v_turn, v_reach(v_corner, d_avail - R / p['turn_gain']) if d_avail - R / p['turn_gain'] >= 0 else 0.0)

        v_guard = p['max_speed']
        if d_side < p['wall_guard']:
            frac = float(np.clip((d_side - 0.25) / max(p['wall_guard'] - 0.25, 1e-3), 0.0, 1.0))
            v_guard = p['wall_guard_speed'] + frac * (p['max_speed'] - p['wall_guard_speed'])
        v_target = min(p['max_speed'], v_curv, v_brake, v_turn, v_guard, v_follow)
        self.dbg = (v_curv, v_brake, v_turn, v_guard, d_free, d_path)
        v_target = max(p['min_speed'], v_target)
        if self.race_done:
            # Fin de carrera: frenado progresivo (limitado a la deceleración estable)
            v_target = 0.0

        aeb = False
        if v_pos > 1.0 and front_x / v_pos < p['ttc_aeb']:
            aeb = True
            v_target = 0.0
            if self.lap_start is not None:
                self.lap_aeb += 1

        a_stab = float(np.interp(v_pos, self.v_grid, self.a_dec_tab))
        if aeb:
            a_cmd = min(9.51, p['aeb_stab_factor'] * a_stab)
            speed = max(0.0, v_pos - a_cmd / p['brake_kp'])
        else:
            speed = min(v_target, v_pos + p['accel_headroom'])
            # nunca pedir más deceleración que la estable a esta velocidad
            speed = max(speed, v_pos - a_stab / p['brake_kp'])
            speed = float(np.clip(speed, 0.0, p['max_speed']))
        self.braking = aeb or ((v_pos - speed) * p['brake_kp'] > p['yaw_brake_on'])

        self.publish_drive(speed, self.steering)

        # ---------- Registro
        if self.csv_writer is not None and \
                now - self.last_log_write >= 1.0 / max(p['log_rate_hz'], 1.0):
            self.last_log_write = now
            x, y = self.pos if self.pos is not None else (float('nan'), float('nan'))
            self.csv_writer.writerow([
                f'{now:.3f}', self.lap_count + 1, f'{x:.3f}', f'{y:.3f}', f'{v:.3f}',
                f'{speed:.3f}', f'{v_curv:.3f}', f'{v_brake:.3f}', f'{v_turn:.3f}', f'{self.steering:.4f}',
                f'{steer_des:.4f}', f'{target_raw:.4f}', f'{target_depth:.2f}',
                f'{d_free:.2f}', f'{d_path:.2f}', f'{front_x:.2f}', f'{min_clear:.3f}', f'{d_l:.3f}', f'{d_r:.3f}', f'{center_corr:.4f}', f'{self.yaw_rate:.3f}', f'{yaw_corr:.4f}', n_gaps,
                int(same_gap), int(aeb), p['steering_mode']])

        if now - self.last_info >= 1.0:
            self.last_info = now
            if self.csv_file is not None:
                self.csv_file.flush()
            self.get_logger().info(
                f'v_real {v:5.2f} | v_cmd {speed:5.2f} | giro {self.steering:+.3f} | '
                f'objetivo {math.degrees(target_raw):+5.1f} deg | libre {d_free:5.1f} m | '
                f'frente {front_x:5.1f} m | izq {d_l:4.2f} der {d_r:4.2f} | gaps {n_gaps}'
                + (f' | OT objs {getattr(self, "ot_nobj", 0)} cerca {int(self.ot_near)} '
                   + (f'auto x {obj[1]:.1f} y [{obj[2]:+.2f},{obj[3]:+.2f}] v_auto {v_pos + self.ot_vrel:.1f}'
                      if obj is not None else 'auto -')
                   + f' | y_ref {self.y_ref:+.2f}' if p['overtake_enabled'] else ''))

    # --------------------------------------------------------- visualización

    @staticmethod
    def _speed_color(v, v_max):
        # azul (lento) -> verde -> amarillo -> rojo (rápido)
        t = float(np.clip(v / max(v_max, 1e-3), 0.0, 1.0))
        c = ColorRGBA()
        if t < 0.33:
            k = t / 0.33
            c.r, c.g, c.b = 0.0, k, 1.0 - k
        elif t < 0.66:
            k = (t - 0.33) / 0.33
            c.r, c.g, c.b = k, 1.0, 0.0
        else:
            k = (t - 0.66) / 0.34
            c.r, c.g, c.b = 1.0, 1.0 - k, 0.0
        c.a = 1.0
        return c

    def _base_marker(self, mid, mtype, frame):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = str(self.p['run_tag'])   # cada coche con su propio grupo de marcadores
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _publish_viz(self):
        if self.viz_pub is None or not rclpy.ok():
            return
        try:
            frame = self.p['viz_frame']
            arr = MarkerArray()

            t_pub = time.monotonic()
            if len(self.viz_cur_pts) >= 2:
                m = self._base_marker(0, Marker.LINE_STRIP, frame)
                m.scale.x = self.p['viz_width']
                m.points = list(self.viz_cur_pts)
                m.colors = list(self.viz_cur_cols)
                arr.markers.append(m)

            # La vuelta anterior se envía una sola vez (RViz la conserva)
            if self.viz_prev_dirty and len(self.viz_prev_pts) >= 2:
                m = self._base_marker(1, Marker.LINE_STRIP, frame)
                m.scale.x = 0.5 * self.p['viz_width']
                m.color.r = m.color.g = m.color.b = 0.6
                m.color.a = 0.6
                m.points = self.viz_prev_pts
                arr.markers.append(m)
                self.viz_prev_dirty = False

            if self.start_pos is not None:
                m = self._base_marker(2, Marker.TEXT_VIEW_FACING, frame)
                m.pose.position.x = self.start_pos[0]
                m.pose.position.y = self.start_pos[1] + float(self.p['viz_text_dy'])
                m.pose.position.z = 1.0
                m.scale.z = float(self.p['viz_text_size'])
                tc = list(self.p['viz_text_color']) + [1.0, 1.0, 1.0]
                m.color.r, m.color.g, m.color.b = float(tc[0]), float(tc[1]), float(tc[2])
                m.color.a = 1.0
                if self.race_done:
                    lines = [f"{self.p['run_tag']}  |  CARRERA TERMINADA"]
                else:
                    lines = [f"{self.p['run_tag']}  |  vuelta {self.lap_count + 1}"]
                if self.lap_start is not None and not self.race_done:
                    lines.append(f'actual: {time.monotonic() - self.lap_start:5.1f} s  {self.lap_dist:6.1f} m')
                if self.last_lap_time is not None:
                    lines.append(f'ultima: {self.last_lap_time:.2f} s  {self.last_lap_dist:.1f} m')
                if self.best_lap is not None:
                    lines.append(f'mejor: {self.best_lap:.2f} s')
                m.text = '\n'.join(lines)
                arr.markers.append(m)

            if self.viz_target is not None and self.p['viz_show_target']:
                m = self._base_marker(3, Marker.SPHERE, self.p['viz_laser_frame'])
                m.pose.position.x, m.pose.position.y = self.viz_target
                m.pose.position.z = 0.1
                m.scale.x = m.scale.y = m.scale.z = 0.35
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 1.0, 0.9
                arr.markers.append(m)

            if not self.p['viz_show_target'] and not self.viz_target_cleared:
                # borrar la esfera que pudo quedar en RViz de una corrida anterior
                m = self._base_marker(3, Marker.SPHERE, self.p['viz_laser_frame'])
                m.action = Marker.DELETE
                arr.markers.append(m)
                self.viz_target_cleared = True
            self.viz_pub.publish(arr)
            ms = 1000.0 * (time.monotonic() - t_pub)
            if ms > 15.0:
                self.get_logger().warn(f'Visualización lenta ({ms:.0f} ms): se reduce la frecuencia')
                self.p['viz_point_spacing'] *= 1.5
        except Exception as e:
            self.get_logger().warn(f'Visualización desactivada por error: {e}')
            self.viz_pub = None

    def close_logs(self):
        for f in (self.csv_file, self.laps_file):
            try:
                if f is not None:
                    f.flush()
                    f.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveFollowGap()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        # Al cerrar con Ctrl+C el contexto de ROS puede invalidarse dentro de spin()
        if rclpy.ok():
            raise
        _ = e
    finally:
        try:
            if rclpy.ok():
                node.publish_drive(0.0, 0.0)
        except Exception:
            pass
        node.close_logs()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()