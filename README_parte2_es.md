# Proyecto F1TENTH – Parte 2: Competencia con obstáculos y rebase

Segunda parte del proyecto F1TENTH en el circuito de **Sepang** (ROS 2 Humble, simulador F1TENTH Gym). Se usa el mismo controlador **Follow the Gap** de la parte 1, ampliado para correr en una pista con **5 obstáculos fijos** y **un segundo vehículo** que arranca 5 m adelante y debe ser rebasado. Las dos condiciones se prueban **juntas**: dos vehículos y cinco obstáculos al mismo tiempo, durante 10 vueltas consecutivas.

**Resultado final (10 vueltas consecutivas cada vehículo):**

| Vehículo | Vueltas | Mejor vuelta | Promedio | Colisiones | AEB |
|---|---|---|---|---|---|
| Principal | 10/10 | **49.79 s** | 50.71 s | **0** | 0 |
| Oponente | 10/10 | 58.11 s | 58.27 s | **0** | 0 |

**Videos:**
- Parte 1 (circuito libre): https://youtu.be/4MpEtsWnOw8
- Parte 2 (dos vehículos y obstáculos): https://youtu.be/fDcNhzK-DyY

*English version: [README_parte2.md](README_parte2.md) · Parte 1: [README_es.md](README_es.md)*

---

## 1. Resumen

- **Un solo script** lanza toda la prueba: `./iniciar_sepang_parte2.sh`. Abre el simulador con los dos vehículos y el mapa con obstáculos, RViz, dos contadores de vueltas, los dos controladores y la grabación rosbag.
- Los **dos vehículos usan el mismo algoritmo** (`gap_node.py`). El oponente corre más lento (máximo 8.7 m/s) para que se le pueda alcanzar y rebasar.
- Cada vehículo **se detiene solo al completar 10 vueltas**. Cuando terminan los dos, el script cierra todo, imprime un cuadro resumen y convierte la grabación a CSV.
- En RViz, cada vehículo lleva junto a él un texto con su número de vuelta y sus tiempos. Al rebasar aparece un aviso **"REBASE!"** y cambia la posición (P2 → P1).

## 2. Cómo funciona

### 2.1. Obstáculos fijos
- `crear_mapa_obstaculos.py` toma `Sepang_map.png` y dibuja 5 cuadrados de 0.40 m sobre la trayectoria normal del vehículo, desplazados hacia un lado para obligarlo a esquivar. El resultado es `Sepang_obstaculos.png/.yaml`.
- El LiDAR ve los obstáculos igual que una pared, así que Follow the Gap los esquiva sin cambios en su lógica.
- Cuando se detecta un objeto pequeño (un obstáculo o el otro vehículo), se suma un **margen extra** al inflado de los obstáculos (`obs_margin_extra`). En el resto del circuito el margen es el de la parte 1, así que no se pierde tiempo.
- Para no frenar de más junto a los obstáculos, el frenado considera las paredes y solo lo que está en el corredor por donde pasa el vehículo (`obs_brake_walls`). Si el otro vehículo está cerca, se frena por todo lo que se ve.

| # | Posición (x, y) [m] | Ubicación |
|---|---|---|
| 1 | (0.17, −9.44) | Recta larga, ~66 m después de la salida |
| 2 | (−9.70, −31.77) | ~160 m |
| 3 | (20.10, −10.87) | ~228 m |
| 4 | (−25.59, 36.90) | ~362 m |
| 5 | (−30.84, −2.43) | Recta de meta, ~30 m antes de la línea |

### 2.2. El otro vehículo como obstáculo dinámico
- **Oponente:** el mismo `gap_node` con sus propios tópicos (`/opp_scan`, `/opp_drive`, `/opp_racecar/odom`), velocidad máxima 8.7 m/s y cargado a la derecha (`lane_bias = −0.4`). Arranca en (5.0, 0.0), 5 m adelante.
- **Vehículo principal:** ve al oponente con el LiDAR como un obstáculo más. Además lee la **odometría del oponente** (`/opp_racecar/odom`), que en el simulador da su posición y velocidad, para no chocarlo (`safe_rival`):
  - **En curvas y zonas de obstáculos** (menos de 12 m libres por delante), si el oponente va delante, el principal se queda unos 3 m detrás y no acelera más que él. Mientras lo sigue, los puntos del LiDAR que caen sobre el oponente no se usan para elegir el gap, para que no tape el paso entre obstáculos.
  - **En recta**, Follow the Gap ve al oponente y busca el hueco libre a su lado para rebasarlo.
  - Si el oponente va en el mismo carril justo delante, el principal limita su velocidad para no llegarle encima (`ot_speed_only`).
- Con esto, el principal rebasa al oponente al inicio y luego le saca vuelta durante la carrera sin contacto. La vuelta en la que le saca vuelta es más lenta (55.6 s), porque espera detrás de él en la zona de curvas.

### 2.3. Grabación rosbag y CSV
- Se graban `/scan`, `/ego_racecar/odom`, `/drive`, `/opp_scan`, `/opp_racecar/odom` y `/opp_drive` en `~/f1tenth_bags/PARTE2_<fecha>/`.
- Al terminar la carrera, `bag_a_csv.py` convierte cada tópico a un CSV en la carpeta `csv/` de la grabación:

| Archivo | Columnas |
|---|---|
| `ego_racecar_odom.csv`, `opp_racecar_odom.csv` | tiempo, x, y, rumbo, velocidad lineal x/y, velocidad angular z |
| `drive.csv`, `opp_drive.csv` | tiempo, velocidad comandada, ángulo de dirección |
| `scan.csv`, `opp_scan.csv` | tiempo, ángulo inicial, incremento, r0 … rN (distancias del LiDAR) |

- La conversión tarda unos minutos, porque el LiDAR tiene muchos datos. **No se debe presionar Ctrl+C mientras convierte.**
- Si la carrera se corta antes con Ctrl+C, la grabación queda sin convertir y se puede convertir después a mano (ver la sección 4).

## 3. Archivos de la parte 2

| Archivo | Función |
|---|---|
| `iniciar_sepang_parte2.sh` | Lanza la prueba completa (dos vehículos + obstáculos) |
| `crear_mapa_obstaculos.py` | Crea el mapa con los 5 obstáculos |
| `resumen_carrera.py` | Espera las 10 vueltas de cada vehículo e imprime el cuadro resumen |
| `bag_a_csv.py` | Convierte una grabación rosbag a CSV |
| `src/controllers/controllers/gap_node.py` | Controlador Follow the Gap (el mismo para los dos vehículos) |
| `src/f1tenth_gym_ros/config/sim_2robots.yaml` | Simulador con 2 vehículos y el mapa con obstáculos |
| `src/f1tenth_gym_ros/maps/Sepang_obstaculos.png/.yaml` | Mapa con obstáculos |

Todas las funciones nuevas del controlador vienen **apagadas por defecto** y se activan con parámetros en `iniciar_sepang_parte2.sh`. Por eso la parte 1 (`./iniciar_sepang.sh`) funciona igual que antes.

## 4. Compilación y ejecución

**Compilar:**
```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
colcon build --packages-select controllers f1tenth_gym_ros --symlink-install
source install/setup.bash
```

**Crear el mapa con obstáculos** (solo la primera vez):
```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
python3 crear_mapa_obstaculos.py
```

**Correr la parte 2:**
```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
source install/setup.bash
./iniciar_sepang_parte2.sh
```
El script termina solo. La carrera terminó bien cuando aparecen el cuadro resumen con **"Carrera terminada."**, después **"CSV guardados en ..."** con los 6 archivos, y vuelve el prompt de la terminal.

**Si quedaron procesos de una ejecución anterior:**
```bash
pkill -f gym_bridge; pkill -f gap_node; pkill -f lap_counter; pkill -f rviz2; pkill -f "ros2 bag"
```

**Convertir a CSV una grabación a mano:**
```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
python3 bag_a_csv.py ~/f1tenth_bags/PARTE2_<fecha>
```

**Volver a sacar el cuadro resumen:**
```bash
python3 resumen_carrera.py --tags PRINCIPAL OPONENTE --laps 10 --since 2026-11-26T17:00:00
```

## 5. Parámetros principales

| Parámetro | Valor | Descripción |
|---|---|---|
| `race_laps` | 10 | Vueltas antes de detenerse |
| `obs_margin_extra` | 0.15 m (principal) / 0.08 m (oponente) | Margen extra cerca de objetos pequeños |
| `obs_brake_walls` | true (principal) | El frenado no considera los obstáculos que quedan fuera del corredor |
| `obs_angle_deg` | 100° | Ángulo a cada lado en el que se buscan obstáculos |
| `safe_rival` | true (principal) | Usa la odometría del oponente para no chocarlo |
| `sr_straight` | 12 m | Recta libre mínima por delante para rebasar |
| `sr_gap_twisty` | 3 m | Distancia detrás del oponente en curvas y obstáculos |
| `sr_lat` | 0.8 m | Separación lateral por debajo de la cual se considera el mismo carril |
| `ot_speed_only` | true (los dos) | Limita la velocidad detrás del otro vehículo en el mismo carril |
| `max_speed` | 12.0 m/s (principal) / 8.7 m/s (oponente) | Velocidad máxima |
| `lane_bias` | −0.4 (oponente) | Desplazamiento lateral del oponente |

## 6. Resultados

| Métrica | Principal | Oponente |
|---|---|---|
| Vueltas completadas | **10/10** | **10/10** |
| Mejor vuelta | **49.79 s** | 58.11 s |
| Promedio (sin la vuelta 1) | 50.71 s | 58.27 s |
| Desviación entre vueltas | 1.85 s | 0.17 s |
| Velocidad máxima | 11.96 m/s | 8.70 m/s |
| Holgura mínima | 0.255 m | 0.254 m |
| Activaciones del AEB | **0** | **0** |
| Reversas | 0 | 0 |
| Colisiones | **0** | **0** |
| Tiempo total | 509.1 s | 583.5 s |

- La mejor vuelta del principal (49.79 s) es solo 0.7 s más lenta que la parte 1 sin obstáculos (49.09 s).
- La desviación del principal es mayor porque en una vuelta (la 8) espera detrás del oponente para sacarle vuelta de forma segura.

### Problemas encontrados y resueltos
- **Margen de seguridad en todo el circuito:** hacía chocar al vehículo en la horquilla. Se reemplazó por un margen extra que solo se aplica cerca de objetos pequeños.
- **Obstáculo con dos pasos casi iguales:** el vehículo dudaba entre los dos lados. El obstáculo 3 se movió 0.3 m para dejar un solo paso claro.
- **Frenado de más junto a los obstáculos:** el frenado ahora considera las paredes y el corredor del vehículo, no los obstáculos de los lados. Esto bajó el tiempo de vuelta en ~1 s.
- **Contacto con el oponente junto a los obstáculos:** el LiDAR a veces no reconocía al oponente como vehículo, y este tapaba el paso entre obstáculos. Se resolvió usando su odometría: en curvas y obstáculos el principal va detrás, y rebasa solo en recta.

## 7. Conclusiones

- Follow the Gap maneja obstáculos fijos sin cambios en su lógica: para el LiDAR, los obstáculos son parte de la pista.
- Aplicar el margen extra **solo cerca de objetos pequeños** mantiene casi el mismo tiempo de vuelta de la parte 1.
- Un vehículo en movimiento es más difícil que un obstáculo fijo, porque cambia de posición y puede tapar el único paso libre. Rebasar solo en recta y esperar detrás en las curvas evita los contactos.
- El cuadro resumen automático y la grabación en CSV permiten comparar corridas con las mismas métricas y justificar cada cambio con datos.
- Trabajo futuro: detectar y predecir la trayectoria del otro vehículo usando solo el LiDAR, sin su odometría.