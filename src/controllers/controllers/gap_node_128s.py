import math
import time

import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class ReactiveFollowGap(Node):

    def __init__(self):
        super().__init__('reactive_node')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        # Velocidad
        self.max_speed = 8.0
        self.min_speed = 1.2
        self.corner_speed = 2.2

        # Direccion
        self.max_steering = 0.4189
        self.steering_gain = 0.85
        self.steering_deadband = 0.018
        self.max_steering_rate = 3.2

        # Follow the Gap
        self.max_lidar_range = 12.0
        self.free_distance = 1.8
        self.vehicle_clearance = 0.42
        self.disparity_threshold = 0.65

        # Estabilidad
        self.previous_steering = 0.0
        self.previous_target = 0.0
        self.previous_speed = 0.0
        self.previous_time = None

        # Aceleracion
        self.max_acceleration = 4.5
        self.max_deceleration = 9.0

        # Diagnostico
        self.last_log = 0.0

        self.get_logger().info(
            'Follow the Gap optimizado iniciado'
        )

    def publish_drive(self, speed, steering):

        msg = AckermannDriveStamped()

        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)

        self.drive_pub.publish(msg)

    def preprocess_lidar(self, msg):

        ranges = np.asarray(
            msg.ranges,
            dtype=float
        ).copy()

        if ranges.size == 0:
            return ranges

        upper = self.max_lidar_range

        if math.isfinite(msg.range_max):
            upper = min(
                upper,
                float(msg.range_max)
            )

        lower = max(
            0.05,
            float(msg.range_min)
        )

        ranges[np.isposinf(ranges)] = upper

        invalid = (
            ~np.isfinite(ranges)
            | (ranges < lower)
        )

        ranges[invalid] = 0.0

        ranges = np.clip(
            ranges,
            0.0,
            upper
        )

        return ranges

    def extend_disparities(self, ranges, increment):

        extended = ranges.copy()

        differences = np.abs(
            np.diff(ranges)
        )

        disparities = np.where(
            differences > self.disparity_threshold
        )[0]

        for idx in disparities:

            left = ranges[idx]
            right = ranges[idx + 1]

            if left <= 0.0 or right <= 0.0:
                continue

            if left < right:
                near_idx = idx
                near_dist = left
                direction = 1
            else:
                near_idx = idx + 1
                near_dist = right
                direction = -1

            angle = math.atan2(
                self.vehicle_clearance,
                max(near_dist, 0.1)
            )

            rays = int(
                math.ceil(
                    angle / abs(increment)
                )
            )

            if direction == 1:

                start = near_idx
                end = min(
                    len(extended),
                    near_idx + rays + 1
                )

            else:

                start = max(
                    0,
                    near_idx - rays
                )

                end = near_idx + 1

            extended[start:end] = np.minimum(
                extended[start:end],
                near_dist
            )

        return extended

    def find_best_gap(self, safe, angles):

        free = safe >= self.free_distance

        edges = np.diff(
            np.r_[False, free, False].astype(int)
        )

        starts = np.where(edges == 1)[0]
        ends = np.where(edges == -1)[0]

        best_score = -float('inf')
        best_target = None
        best_depth = 0.0

        for start, end in zip(starts, ends):

            width_rays = end - start

            if width_rays < 6:
                continue

            margin = max(
                2,
                int(width_rays * 0.08)
            )

            left = start + margin
            right = end - margin

            if right <= left:
                continue

            # Centro del corredor
            center_idx = (left + right - 1) // 2

            center_angle = float(
                angles[center_idx]
            )

            # Mantener trayectoria anterior
            previous_idx = int(
                np.argmin(
                    np.abs(
                        angles[left:right]
                        - self.previous_target
                    )
                )
            ) + left

            previous_angle = float(
                angles[previous_idx]
            )

            # Objetivo combinado
            target_angle = (
                0.75 * center_angle
                + 0.25 * previous_angle
            )

            width = abs(
                angles[right - 1] - angles[left]
            )

            depth = float(
                np.percentile(
                    safe[left:right],
                    35
                )
            )

            score = (
                3.0 * width
                + 0.35 * min(depth, 10.0)
                - 1.4 * abs(target_angle)
                - 1.3 * abs(
                    target_angle - self.previous_target
                )
            )

            if score > best_score:

                best_score = score
                best_target = target_angle
                best_depth = depth

        return best_target, best_depth

    def lateral_correction(self, scan, theta):

        # Distancias laterales a izquierda y derecha
        left_mask = (
            (theta >= math.radians(65))
            & (theta <= math.radians(85))
        )

        right_mask = (
            (theta <= math.radians(-65))
            & (theta >= math.radians(-85))
        )

        left = scan[left_mask]
        right = scan[right_mask]

        left = left[left > 0.1]
        right = right[right > 0.1]

        if left.size < 3 or right.size < 3:
            return 0.0, False

        left_dist = float(np.median(left))
        right_dist = float(np.median(right))

        # Solo corregir cuando ambas paredes
        # estan suficientemente cerca
        if left_dist > 5.0 or right_dist > 5.0:
            return 0.0, False

        error = left_dist - right_dist

        correction = float(
            np.clip(
                0.045 * error,
                -0.10,
                0.10
            )
        )

        return correction, True

    def calculate_speed(
        self,
        steering,
        front_dist,
        depth,
        target
    ):

        abs_steer = abs(steering)

        # Velocidad segun direccion
        if abs_steer < 0.055:
            steering_speed = 8.0

        elif abs_steer < 0.11:
            steering_speed = 6.5

        elif abs_steer < 0.18:
            steering_speed = 5.0

        elif abs_steer < 0.27:
            steering_speed = 3.5

        else:
            steering_speed = self.corner_speed

        # Velocidad segun espacio frontal
        if front_dist < 0.8:
            distance_speed = 0.0

        elif front_dist < 1.5:
            distance_speed = 1.2

        elif front_dist < 2.5:
            distance_speed = 2.2

        elif front_dist < 4.0:
            distance_speed = 3.5

        elif front_dist < 6.0:
            distance_speed = 5.0

        elif front_dist < 8.0:
            distance_speed = 6.5

        else:
            distance_speed = self.max_speed

        # Profundidad del corredor
        # Ya no limita automaticamente a 3 m/s
        if depth < 2.5:
            gap_speed = 2.2

        elif depth < 4.0:
            gap_speed = 4.0

        elif depth < 6.0:
            gap_speed = 5.5

        else:
            gap_speed = self.max_speed

        # Reducir velocidad si el objetivo
        # exige un cambio grande de orientacion
        abs_target = abs(target)

        if abs_target > 0.55:
            target_limit = 2.5

        elif abs_target > 0.35:
            target_limit = 3.5

        elif abs_target > 0.20:
            target_limit = 5.0

        else:
            target_limit = self.max_speed

        speed = min(
            steering_speed,
            distance_speed,
            gap_speed,
            target_limit
        )

        return speed

    def lidar_callback(self, msg):

        now = time.monotonic()

        if self.previous_time is None:
            dt = 0.05
        else:
            dt = float(
                np.clip(
                    now - self.previous_time,
                    0.01,
                    0.15
                )
            )

        self.previous_time = now

        if (
            not math.isfinite(msg.angle_increment)
            or abs(msg.angle_increment) < 1e-8
        ):
            self.previous_speed = 0.0
            self.publish_drive(0.0, 0.0)
            return

        ranges = self.preprocess_lidar(msg)

        if ranges.size < 5:
            self.previous_speed = 0.0
            self.publish_drive(0.0, 0.0)
            return

        angles = (
            msg.angle_min
            + np.arange(len(ranges))
            * msg.angle_increment
        )

        # Campo frontal
        mask = np.abs(angles) <= math.radians(85)

        scan = ranges[mask].copy()
        theta = angles[mask]

        if len(scan) < 5:
            self.previous_speed = 0.0
            self.publish_drive(0.0, 0.0)
            return

        # Distancia frontal
        frontal_mask = (
            np.abs(theta) <= math.radians(12)
        )

        frontal = scan[frontal_mask]

        if frontal.size == 0:
            front_dist = 0.0

        elif np.any(frontal <= 0.05):
            front_dist = 0.0

        else:
            front_dist = float(
                np.percentile(frontal, 20)
            )

        # Disparity Extender
        safe = self.extend_disparities(
            scan,
            msg.angle_increment
        )

        # Burbuja de seguridad
        valid = np.where(
            safe > 0.05,
            safe,
            np.inf
        )

        if not np.any(np.isfinite(valid)):

            self.previous_speed = 0.0
            self.publish_drive(0.0, 0.0)
            return

        closest = int(np.argmin(valid))
        closest_distance = float(safe[closest])

        bubble_angle = math.atan2(
            self.vehicle_clearance,
            max(closest_distance, 0.1)
        )

        bubble_rays = max(
            1,
            int(
                math.ceil(
                    bubble_angle
                    / abs(msg.angle_increment)
                )
            )
        )

        start = max(
            0,
            closest - bubble_rays
        )

        end = min(
            len(safe),
            closest + bubble_rays + 1
        )

        safe[start:end] = 0.0

        # Buscar mejor corredor
        target, depth = self.find_best_gap(
            safe,
            theta
        )

        if target is None:

            self.previous_speed = 0.0

            self.publish_drive(
                0.0,
                self.previous_steering
            )

            return

        # Centrado lateral
        correction, walls_valid = (
            self.lateral_correction(
                scan,
                theta
            )
        )

        # Aplicar centrado solo cuando
        # la trayectoria es casi recta
        if walls_valid and abs(target) < 0.18:

            target = (
                0.85 * target
                + correction
            )

        # Direccion deseada
        desired_steering = float(
            np.clip(
                self.steering_gain * target,
                -self.max_steering,
                self.max_steering
            )
        )

        if abs(desired_steering) < self.steering_deadband:
            desired_steering = 0.0

        # Suavizado
        if abs(desired_steering) > 0.18:
            alpha = 0.80
        else:
            alpha = 0.45

        filtered_steering = (
            alpha * desired_steering
            + (1.0 - alpha) * self.previous_steering
        )

        max_change = (
            self.max_steering_rate * dt
        )

        steering = float(
            np.clip(
                filtered_steering,
                self.previous_steering - max_change,
                self.previous_steering + max_change
            )
        )

        steering = float(
            np.clip(
                steering,
                -self.max_steering,
                self.max_steering
            )
        )

        # Calcular velocidad
        target_speed = self.calculate_speed(
            steering,
            front_dist,
            depth,
            target
        )

        # Frenado de emergencia
        if front_dist < 0.8:
            speed = 0.0

        elif target_speed > self.previous_speed:

            speed = min(
                target_speed,
                self.previous_speed
                + self.max_acceleration * dt
            )

        else:

            speed = max(
                target_speed,
                self.previous_speed
                - self.max_deceleration * dt
            )

        speed = float(
            np.clip(
                speed,
                0.0,
                self.max_speed
            )
        )

        self.previous_speed = speed
        self.previous_steering = steering
        self.previous_target = target

        self.publish_drive(
            speed,
            steering
        )

        # Diagnostico cada segundo
        if now - self.last_log >= 1.0:

            self.last_log = now

            self.get_logger().info(
                f'VEL: {speed:.2f} m/s | '
                f'GIRO: {steering:.3f} rad | '
                f'FRONTAL: {front_dist:.2f} m | '
                f'GAP: {depth:.2f} m'
            )


def main(args=None):

    rclpy.init(args=args)

    node = ReactiveFollowGap()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.publish_drive(0.0, 0.0)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
