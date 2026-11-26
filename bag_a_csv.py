
import csv
import math
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def abrir(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                            output_serialization_format='cdr'))
    return reader


def nombre_archivo(topic):
    return topic.strip('/').replace('/', '_') + '.csv'


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    uri = os.path.expanduser(sys.argv[1].rstrip('/'))
    salida = os.path.join(uri, 'csv')
    os.makedirs(salida, exist_ok=True)

    reader = abrir(uri)
    tipos = {t.name: t.type for t in reader.get_all_topics_and_types()}
    clases = {name: get_message(tipo) for name, tipo in tipos.items()}

    archivos = {}
    escritores = {}
    cuenta = {}
    t0 = None

    def escritor(topic, encabezado):
        if topic not in escritores:
            f = open(os.path.join(salida, nombre_archivo(topic)), 'w', newline='')
            archivos[topic] = f
            escritores[topic] = csv.writer(f)
            escritores[topic].writerow(encabezado)
            cuenta[topic] = 0
        return escritores[topic]

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if t0 is None:
            t0 = t_ns
        t = (t_ns - t0) * 1e-9
        tipo = tipos[topic]
        msg = deserialize_message(data, clases[topic])

        if tipo == 'nav_msgs/msg/Odometry':
            q = msg.pose.pose.orientation
            rumbo = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            w = escritor(topic, ['t_s', 'x_m', 'y_m', 'rumbo_rad',
                                 'v_lineal_x_mps', 'v_lineal_y_mps', 'v_angular_z_radps'])
            w.writerow([f'{t:.4f}', f'{msg.pose.pose.position.x:.4f}', f'{msg.pose.pose.position.y:.4f}',
                        f'{rumbo:.5f}', f'{msg.twist.twist.linear.x:.4f}',
                        f'{msg.twist.twist.linear.y:.4f}', f'{msg.twist.twist.angular.z:.5f}'])

        elif tipo == 'ackermann_msgs/msg/AckermannDriveStamped':
            w = escritor(topic, ['t_s', 'velocidad_mps', 'angulo_direccion_rad'])
            w.writerow([f'{t:.4f}', f'{msg.drive.speed:.4f}', f'{msg.drive.steering_angle:.5f}'])

        elif tipo == 'sensor_msgs/msg/LaserScan':
            n = len(msg.ranges)
            w = escritor(topic, ['t_s', 'angulo_min_rad', 'paso_angular_rad'] +
                         [f'r{i}' for i in range(n)])
            w.writerow([f'{t:.4f}', f'{msg.angle_min:.6f}', f'{msg.angle_increment:.8f}'] +
                       [f'{r:.3f}' for r in msg.ranges])
        else:
            continue
        cuenta[topic] += 1

    for f in archivos.values():
        f.close()
    print(f'CSV guardados en {salida}')
    for topic in sorted(cuenta):
        print(f'  {nombre_archivo(topic):28s} {cuenta[topic]:7d} filas')


if __name__ == '__main__':
    main()