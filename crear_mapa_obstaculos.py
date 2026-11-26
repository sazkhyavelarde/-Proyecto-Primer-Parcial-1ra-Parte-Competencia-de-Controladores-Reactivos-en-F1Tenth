import os

import numpy as np
import yaml
from PIL import Image

# Carpeta de los mapas
MAPS = os.path.expanduser('~/F1Tenth-Repository/src/f1tenth_gym_ros/maps')
ORIGINAL = 'Sepang_map'
NUEVO = 'Sepang_obstaculos'

# Tamaño de cada obstáculo (lado del cuadrado, en metros)
LADO = 0.40

# Centro de cada obstáculo en coordenadas del mapa (metros).
# Cada obstáculo está sobre la trayectoria que sigue el carro sin obstáculos,
# desplazado 0.25 m hacia un lado (alternando): tapa la mitad de la línea del
# carro, así que el carro está obligado a desviarse para no chocar.
OBSTACULOS = [
    (0.17, -9.44),     # 1: recta larga, ~66 m desde la salida
    (-9.70, -31.77),   # 2: ~160 m
    (20.10, -10.87),   # 3: ~228 m (corrido 0.3 m: así deja un solo paso claro)
    (-25.59, 36.90),   # 4: ~362 m
    (-30.84, -2.43),   # 5: recta de meta, ~30 m antes de la línea
]


def metros_a_pixel(x, y, resolucion, origen, alto):
    """Convierte una posición del mapa (m) a fila y columna de la imagen.

    En los mapas de ROS el origen es la esquina inferior izquierda de la imagen,
    mientras que en la imagen la fila 0 es la de arriba; por eso se invierte el eje y.
    """
    col = (x - origen[0]) / resolucion
    fila = alto - 1 - (y - origen[1]) / resolucion
    return int(round(fila)), int(round(col))


def main():
    with open(os.path.join(MAPS, ORIGINAL + '.yaml')) as f:
        info = yaml.safe_load(f)
    resolucion = float(info['resolution'])
    origen = info['origin']

    img = np.array(Image.open(os.path.join(MAPS, info['image'])).convert('L'))
    alto = img.shape[0]
    medio = int(round(LADO / resolucion / 2))   # medio lado en píxeles

    for i, (x, y) in enumerate(OBSTACULOS, 1):
        fila, col = metros_a_pixel(x, y, resolucion, origen, alto)
        img[fila - medio:fila + medio + 1, col - medio:col + medio + 1] = 0   # negro = obstáculo
        print(f'Obstáculo {i}: ({x:.2f}, {y:.2f}) m -> píxel (fila {fila}, columna {col})')

    Image.fromarray(img).save(os.path.join(MAPS, NUEVO + '.png'))

    info['image'] = NUEVO + '.png'
    with open(os.path.join(MAPS, NUEVO + '.yaml'), 'w') as f:
        yaml.safe_dump(info, f, sort_keys=False)

    print(f'Listo: {NUEVO}.png y {NUEVO}.yaml en {MAPS}')


if __name__ == '__main__':
    main()