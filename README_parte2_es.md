# Proyecto F1TENTH – Parte 2: Obstáculos y rebase

Segunda parte del proyecto F1TENTH en el circuito de **Sepang** (ROS 2 Humble, F1TENTH Gym). Se amplió el mismo controlador **Follow the Gap** de la parte 1 para resolver dos pruebas:

1. **Obstáculos fijos:** 5 obstáculos colocados en la trayectoria del vehículo; 10 vueltas consecutivas sin colisión.
2. **Dos vehículos:** un segundo robot arranca 5 m delante del vehículo principal y debe ser rebasado. Ambos robots usan el algoritmo de la parte 1. El objetivo son 10 vueltas consecutivas sin colisión.

**Resultados finales:**

| Prueba | Vueltas | Mejor vuelta | Colisiones | AEB |
|---|---|---|---|---|
| Obstáculos | 10/10 | **51.07 s** | 0 | 0 |
| Dos vehículos (principal) | 10/10 | **49.21 s** | 0 | 0 |
| Dos vehículos (oponente) | 10/10 | 57.74 s | 0 | 0 |

**Evidencia en video:**
- Prueba 1: [`evidencia/PROYECTOPARTE2_obstaculos.webm`](evidencia/PROYECTOPARTE2_obstaculos.webm)
- Prueba 2: [`evidencia/PROYECTOPARTE2_2robots.webm`](evidencia/PROYECTOPARTE2_2robots.webm)

*English version: [README_parte2.md](README_parte2.md) · Parte 1: [README_es.md](README_es.md)*

---

## 1. Descripción

La parte 1 no se reemplazó. Las dos pruebas usan el mismo nodo (`gap_node.py`) con el mismo proceso de Follow the Gap: preprocesado del LiDAR, extensión de disparidades, burbuja de seguridad, selección del gap, control de dirección y velocidad, y AEB.

Todas las funciones nuevas vienen **apagadas por defecto** y se activan solo con parámetros en el script de cada prueba. Por eso la parte 1 (`./iniciar_sepang.sh`) sigue funcionando exactamente igual que antes.

El controlador sigue siendo **reactivo**. No usa mapa, ni localización, ni trayectoria calculada de antemano. Las únicas entradas son el LiDAR y la odometría del propio vehículo.

## 2. Prueba 1 – Obstáculos fijos

### 2.1. Colocación de los obstáculos
Los obstáculos se dibujan directamente sobre una copia del mapa con `crear_mapa_obstaculos.py`:
- Toma `Sepang_map.png` y pinta 5 cuadrados de 0.40 m de lado.
- Guarda el resultado como `Sepang_obstaculos.png/.yaml`.
- El simulador interpreta los píxeles negros como paredes, así que el LiDAR ve los obstáculos igual que cualquier otra pared.

Cada obstáculo se colocó **sobre la trayectoria que sigue el vehículo sin obstáculos**, desplazado 0.25 m hacia un lado, alternando lados. Cada uno tapa la mitad de la línea normal del vehículo, así que el vehículo está obligado a esquivarlo. Sin ese desplazamiento, un obstáculo justo en el centro hacía que el vehículo dudara entre los dos lados.

| # | Posición (x, y) [m] | Ubicación |
|---|---|---|
| 1 | (0.17, −9.44) | Recta larga, ~66 m después de la salida |
| 2 | (−9.70, −31.77) | ~160 m |
| 3 | (20.18, −11.16) | ~228 m |
| 4 | (−25.59, 36.90) | ~362 m |
| 5 | (−30.84, −2.43) | Recta de meta, ~30 m antes de la línea |

### 2.2. Margen extra solo cerca de obstáculos
En un primer intento se subió el margen de seguridad global. Eso hizo que el vehículo chocara contra la pared en la horquilla, porque el espacio libre en las curvas cerradas quedaba demasiado angosto.

La solución adoptada aplica el margen extra **solo cuando se detecta un obstáculo pequeño**:
- El escaneo se divide en objetos, separados por saltos de distancia.
- Un objeto cuenta como obstáculo si es pequeño (≤ 0.85 m), está a menos de 8 m y dentro de ±45°, y el fondo detrás de él está claramente más lejos por los dos lados (contraste).
- En ese caso se suma un margen extra `obs_margin_extra = 0.08 m` al inflado, y se mantiene 0.6 s después de que el obstáculo deja de verse.
- En el circuito sin obstáculos este margen nunca se activa, así que el tiempo por vuelta no empeora.

## 3. Prueba 2 – Dos vehículos y rebase

### 3.1. Configuración
- `sim_2robots.yaml`: `num_agent: 2`. El oponente arranca en (5.0, 0.0), 5 m adelante y con el mismo rumbo.
- **Oponente:** el mismo `gap_node` con los tópicos reasignados (`/opp_scan`, `/opp_drive`, `/opp_racecar/odom`). Corre con una velocidad máxima menor (`max_speed = 8.7 m/s`, ~58 s por vuelta) y un pequeño desplazamiento lateral (`lane_bias = −0.4`), para que se le pueda alcanzar y rebasar. También usa `obs_margin_extra`, así que trata al vehículo principal como un obstáculo.
- **Vehículo principal:** el mismo `gap_node` con `overtake_enabled = true`.

### 3.2. Detección del otro vehículo
Se usa la misma detección de objetos de la prueba 1, con dos controles extra:
- **Mínimo 4 puntos y una relación "de frente" ≥ 0.5.** Esto descarta pedazos de pared vistos muy de lado, que pueden parecer pequeños pero no están de frente al vehículo.
- **Velocidad del objeto.** Se estima con qué tan rápido cambia su distancia. Un objeto nuevo se supone quieto al principio. Un objeto cuenta como carro adelante solo si se mueve a más de 1.5 m/s y está dentro de la pista.

Cuando se detecta un carro adelante, se activa un temporizador de 1.5 s. El modo "carro cerca" se mantiene mientras ese temporizador sigue corriendo, para que no se prenda y apague en cada escaneo.

### 3.3. Lógica de rebase
Mientras hay un carro cerca:
1. **Lado de rebase:** siempre por la **izquierda** (`ot_pass_side = 1`). La línea de referencia se mueve 0.40 m hacia ese lado.
2. **Quitar el carro de Follow the Gap:** el carro en movimiento se quita de la búsqueda del gap, para que no "cierre" el hueco. Para el centrado y el frenado se usa un escaneo solo con paredes.
3. **Esperar o pasar:** el vehículo solo va a rebasar cuando el carril de rebase está libre lo suficientemente lejos (distancia al carro + 12 m + 1 s × velocidad). Si no, se queda detrás a una distancia segura, con un límite de velocidad que deja espacio para frenar (0.8 m + 0.3 s × velocidad, a 3 m/s²).
4. **Compromiso:** cuando el carro está a menos de 1.0 m adelante, el vehículo ya no aborta el rebase.

En la práctica el rebase ocurre en la recta larga, unos 9 s después de la salida. Después el vehículo principal le saca vueltas al oponente varias veces sin contacto.

### 3.4. Fin de la carrera
Cada vehículo se detiene solo después de 10 vueltas (`race_laps = 10`). Cuando el vehículo principal termina, se queda parado en la pista, y el oponente sigue y lo esquiva como un obstáculo.

## 4. Herramientas agregadas en la parte 2

| Archivo | Función |
|---|---|
| `crear_mapa_obstaculos.py` | Crea el mapa con los 5 obstáculos |
| `iniciar_sepang_obstaculos.sh` | Lanza la prueba 1 |
| `iniciar_sepang_2robots.sh` | Lanza la prueba 2 (dos vehículos, dos contadores de vueltas) |
| `resumen_carrera.py` | Espera las 10 vueltas e imprime el cuadro resumen |
| `config/sim_obstaculos.yaml` | Configuración del simulador para la prueba 1 |
| `config/sim_2robots.yaml` | Configuración del simulador para la prueba 2 |

- **Selección de la configuración:** `gym_bridge_launch.py` lee la variable de entorno `F1TENTH_SIM_CONFIG`. Cada script la apunta a su propio YAML. Si no está definida, se usa el `sim.yaml` original (parte 1).
- **Contador de vueltas para los dos carros:** corre un segundo `lap_counter`, suscrito a `/opp_racecar/odom`, que guarda sus registros en `~/f1tenth_logs/oponente`.
- **RViz:** cada vehículo dibuja su propia trayectoria y su propio texto con vueltas y tiempos (el oponente en azul).
- **Resumen y cierre automáticos:** `resumen_carrera.py` lee `~/f1tenth_logs/vueltas.csv` hasta que cada vehículo completa 10 vueltas. Entonces imprime un cuadro con el tiempo, la velocidad media y máxima, la holgura mínima y los AEB de cada vuelta, y después los totales. Luego el script cierra todos los procesos solo. Con Ctrl+C también se cierra todo de forma limpia.

## 5. Parámetros principales de la parte 2

| Parámetro | Valor | Descripción |
|---|---|---|
| `obs_margin_extra` | 0.08 m | Margen extra cerca de obstáculos pequeños |
| `obs_range` / `obs_angle_deg` | 8 m / 45° | Zona de búsqueda de obstáculos |
| `obs_hold` | 0.6 s | Tiempo que se mantiene el margen extra |
| `overtake_enabled` | true (principal) | Activa el rebase |
| `ot_range` | 9 m | Distancia de búsqueda del carro |
| `ot_max_size` | 0.85 m | Tamaño máximo del objeto |
| `ot_min_points` / `ot_min_facing` | 4 / 0.5 | Filtro contra pedazos de pared |
| `ot_moving_speed` | 1.5 m/s | Velocidad mínima para contar como carro |
| `ot_hold` | 1.5 s | Duración del modo "carro cerca" |
| `ot_pass_side` / `ot_pass_offset` | izquierda / 0.40 m | Lado y desplazamiento del rebase |
| `ot_lane_extra` / `ot_lane_time` | 12 m / 1.0 s | Carril libre necesario para pasar |
| `ot_follow_gap` / `ot_follow_time` | 0.8 m / 0.30 s | Distancia de seguimiento |
| `ot_commit` | 1.0 m | Distancia desde la que ya no se aborta el rebase |
| `max_speed` (oponente) | 8.7 m/s | Velocidad máxima del oponente |
| `lane_bias` (oponente) | −0.4 | Desplazamiento lateral del oponente |
| `race_laps` | 10 | Vueltas antes de detenerse |

## 6. Estructura del proyecto (archivos de la parte 2)

```text
F1Tenth-Repository/
├── README_parte2.md
├── README_parte2_es.md
├── crear_mapa_obstaculos.py
├── iniciar_sepang_obstaculos.sh
├── iniciar_sepang_2robots.sh
├── resumen_carrera.py
├── evidencia/
│   ├── PROYECTOPARTE2_obstaculos.webm
│   └── PROYECTOPARTE2_2robots.webm
└── src/
    ├── controllers/controllers/gap_node.py
    └── f1tenth_gym_ros/
        ├── config/
        │   ├── sim_obstaculos.yaml
        │   └── sim_2robots.yaml
        ├── launch/gym_bridge_launch.py
        └── maps/
            ├── Sepang_obstaculos.png
            └── Sepang_obstaculos.yaml
```

## 7. Compilación y ejecución

```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
colcon build --packages-select controllers f1tenth_gym_ros --symlink-install
source install/setup.bash
```

**Prueba 1 – obstáculos:**
```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
./iniciar_sepang_obstaculos.sh
```

**Prueba 2 – dos vehículos:**
```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
./iniciar_sepang_2robots.sh
```

Cada script se cierra solo después de 10 vueltas e imprime el cuadro resumen. El cuadro también se puede volver a sacar a mano:
```bash
python3 resumen_carrera.py --tags OBSTACULOS --laps 10 --since 2026-11-22T13:45:00
```

## 8. Resultados

### Prueba 1 – Obstáculos (10 vueltas consecutivas)

| Métrica | Valor |
|---|---|
| Vueltas completadas | **10/10** |
| Mejor vuelta | **51.07 s** |
| Vuelta promedio | 51.17 s |
| Desviación entre vueltas | 0.08 s |
| Holgura mínima | 0.281 m |
| Activaciones del AEB | **0** |
| Reversas | 0 |
| Colisiones | **0** |

Esquivar los 5 obstáculos cuesta unos 2 s por vuelta comparado con la parte 1 (49.00 s).

### Prueba 2 – Dos vehículos (10 vueltas consecutivas cada uno)

| Métrica | Principal | Oponente |
|---|---|---|
| Vueltas completadas | **10/10** | **10/10** |
| Mejor vuelta | **49.21 s** | 57.74 s |
| Vuelta promedio | 49.57 s | 57.92 s |
| Desviación entre vueltas | 0.45 s | 0.15 s |
| Holgura mínima | 0.337 m | 0.331 m |
| Activaciones del AEB | **0** | **0** |
| Reversas | 0 | 0 |
| Colisiones | **0** | **0** |

El vehículo principal rebasa al oponente en la salida y luego le vuelve a sacar vuelta durante la carrera. Cada rebase cuesta alrededor de 1 s, lo que explica que la desviación del principal sea mayor. Una corrida anterior con el oponente a 8.0 m/s (promedio 62.10 s) también completó 10/10 vueltas sin colisión.

### Problemas encontrados y resueltos
- **Margen de seguridad global:** subir `safety_margin` hizo que el vehículo chocara en la horquilla. Se reemplazó por el margen extra que se aplica solo cerca de obstáculos.
- **Obstáculo justo en el centro de la línea:** el vehículo dudaba entre los dos lados y chocaba. Los obstáculos se movieron 0.25 m hacia un lado.
- **Paredes detectadas como carros:** pedazos de pared vistos muy de lado movían la línea del vehículo principal sin razón. El filtro de puntos y "de frente" y la condición de "objeto en movimiento" bajaron estas detecciones falsas del 41% del tiempo a 0.
- **Oponente que chocaba en la horquilla:** se debía a haber apagado su línea de carrera (`line_gain = 0`). Se volvió al valor por defecto.
- **Contacto al rebasar en la primera curva:** se agregaron la revisión del carril de rebase y el temporizador de "carro cerca", así que ahora el rebase ocurre en la recta.

## 9. Conclusiones

- Follow the Gap maneja obstáculos y otro vehículo sin mapa. Los obstáculos simplemente forman parte de la búsqueda del espacio libre.
- Un margen de seguridad aplicado en todo el circuito hace al carro más lento y menos seguro en curvas cerradas. Aplicarlo **solo cerca de obstáculos** mantiene el rendimiento de la parte 1 donde no hay nada que esquivar.
- Para rebasar, la calidad de la detección importa más que la agresividad. La mayoría de los problemas venían de paredes confundidas con carros, y se resolvieron con filtros simples de geometría y velocidad.
- Esperar a que el carril de rebase esté libre hace que el rebase ocurra en la recta y evita el contacto en las curvas.
- El cuadro resumen automático al final de cada corrida permitió comparar corridas con las mismas métricas y justificar cada cambio con datos.
- Trabajo futuro: elegir el lado de rebase de forma dinámica (izquierda o derecha) y predecir la trayectoria del otro vehículo.