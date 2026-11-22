import argparse
import csv
import os
import statistics
import time
from datetime import datetime

ROJO = '\033[91m'
VERDE = '\033[92m'
CIAN = '\033[96m'
NEGRITA = '\033[1m'
FIN = '\033[0m'


def leer_vueltas(path, tags, since):
    """Devuelve {tag: [fila, ...]} con las vueltas registradas desde 'since'."""
    datos = {t: [] for t in tags}
    if not os.path.exists(path):
        return datos
    with open(path, newline='') as f:
        for fila in csv.DictReader(f):
            try:
                fecha = datetime.fromisoformat(fila['fecha'])
            except (KeyError, ValueError):
                continue
            if fecha >= since and fila.get('run_tag') in datos:
                datos[fila['run_tag']].append(fila)
    return datos


def cuadro(tag, filas):
    tiempos = [float(f['tiempo_s']) for f in filas]
    lineas = []
    lineas.append(f'{NEGRITA}{CIAN}{tag}{FIN}')
    lineas.append('  Vuelta | Tiempo (s) | V media (m/s) | V max (m/s) | Holgura min (m) | AEB')
    lineas.append('  -------+------------+---------------+-------------+-----------------+----')
    mejor = min(tiempos)
    for f, t in zip(filas, tiempos):
        marca = f'{VERDE} <- mejor{FIN}' if t == mejor else ''
        aeb = int(f['eventos_aeb'])
        aeb_txt = f'{ROJO}{aeb:>3}{FIN}' if aeb > 0 else f'{aeb:>3}'
        lineas.append(
            f"  {int(f['vuelta']):>6} | {t:>10.2f} | {float(f['vel_media_m_s']):>13.2f} | "
            f"{float(f['v_max']):>11.2f} | {float(f['min_clearance_m']):>15.3f} | {aeb_txt}{marca}")
    lanzadas = tiempos[1:] if len(tiempos) > 1 else tiempos
    lineas.append('  -------+------------+---------------+-------------+-----------------+----')
    lineas.append(f'  Mejor vuelta:          {mejor:.2f} s')
    lineas.append(f'  Promedio (sin la 1):   {statistics.mean(lanzadas):.2f} s')
    if len(lanzadas) > 1:
        lineas.append(f'  Consistencia (desv.):  {statistics.stdev(lanzadas):.2f} s')
    lineas.append(f'  Tiempo total:          {sum(tiempos):.2f} s')
    lineas.append(f"  Distancia total:       {sum(float(f['distancia_m']) for f in filas):.1f} m")
    lineas.append(f"  Velocidad maxima:      {max(float(f['v_max']) for f in filas):.2f} m/s")
    lineas.append(f"  Holgura minima:        {min(float(f['min_clearance_m']) for f in filas):.3f} m")
    lineas.append(f"  Eventos AEB:           {sum(int(f['eventos_aeb']) for f in filas)}")
    lineas.append(f"  Reversas (atascos):    {sum(int(f['reversas']) for f in filas)}")
    return '\n'.join(lineas)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tags', nargs='+', required=True)
    ap.add_argument('--laps', type=int, default=10)
    ap.add_argument('--since', required=True)
    ap.add_argument('--file', default=os.path.expanduser('~/f1tenth_logs/vueltas.csv'))
    a = ap.parse_args()
    since = datetime.fromisoformat(a.since)

    while True:
        datos = leer_vueltas(a.file, a.tags, since)
        if all(len(datos[t]) >= a.laps for t in a.tags):
            break
        time.sleep(2.0)

    print('\n' + '=' * 78)
    print(f'{NEGRITA}RESUMEN DE LA CARRERA - SEPANG ({a.laps} vueltas){FIN}')
    print('=' * 78)
    for t in a.tags:
        print(cuadro(t, datos[t][:a.laps]))
        print()
    print(f'{NEGRITA}Carrera terminada.{FIN}')
    print('=' * 78, flush=True)


if __name__ == '__main__':
    main()