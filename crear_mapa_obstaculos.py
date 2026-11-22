#!/usr/bin/env python3
"""
crear_mapa_obstaculos.py - Crea el mapa de Sepang con 5 obstáculos fijos.

Toma el mapa original (Sepang_map.png + Sepang_map.yaml), pinta 5 obstáculos
cuadrados en posiciones de la pista dadas en metros y guarda el resultado como
Sepang_obstaculos.png + Sepang_obstaculos.yaml.

El simulador interpreta los píxeles negros como paredes, así que el LiDAR
detecta los obstáculos igual que cualquier otra pared.
"""

import os

import numpy as np
import yaml
from PIL import Image

MAPS = os.path.expanduser('~/F1Tenth-Repository/src/f1tenth_gym_ros/maps')
ORIGINAL = 'Sepang_map'
NUEVO = 'Sepang_obstaculos'

# Tamaño de cada obstáculo (lado del cuadrado, en metros)
LADO = 0.40

# Centro de cada obstáculo en coordenadas del mapa (metros)
OBSTACULOS = [
    (0.17, -9.44),     # 1: recta larga, ~66 m desde la salida
    (-9.70, -31.77),   # 2: ~160 m
    (20.18, -11.16),   # 3: ~228 m
    (-25.59, 36.90),   # 4: ~362 m
    (-30.84, -2.43),   # 5: recta de meta, ~30 m antes de la linea
]


def metros_a_pixel(x, y, resolucion, origen, alto):
    """Convierte una posición del mapa (m) a fila y columna de la imagen."""
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
    medio = int(round(LADO / resolucion / 2))

    for i, (x, y) in enumerate(OBSTACULOS, 1):
        fila, col = metros_a_pixel(x, y, resolucion, origen, alto)
        img[fila - medio:fila + medio + 1, col - medio:col + medio + 1] = 0
        print(f'Obstáculo {i}: ({x:.2f}, {y:.2f}) m -> píxel (fila {fila}, columna {col})')

    Image.fromarray(img).save(os.path.join(MAPS, NUEVO + '.png'))

    info['image'] = NUEVO + '.png'
    with open(os.path.join(MAPS, NUEVO + '.yaml'), 'w') as f:
        yaml.safe_dump(info, f, sort_keys=False)

    print(f'Listo: {NUEVO}.png y {NUEVO}.yaml en {MAPS}')


if __name__ == '__main__':
    main()
