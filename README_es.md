# Proyecto F1TENTH – Competencia de Controladores Reactivos

Controlador reactivo **Follow the Gap** para un vehículo autónomo F1TENTH en el circuito **Sepang**, desarrollado en ROS 2 Humble y evaluado en el simulador F1TENTH Gym.

**Resultado final:** mejor vuelta de **49.09 s**, 10 vueltas consecutivas sin colisiones y sin activaciones del frenado de emergencia.

**Video:** https://youtu.be/4MpEtsWnOw8

*English version: [README.md](README.md) · Parte 2: [README_parte2_es.md](README_parte2_es.md)*

---

## 1. Descripción del proyecto

El objetivo del proyecto es que el vehículo recorra el circuito Sepang de forma autónoma y en el menor tiempo posible, **sin colisionar**, usando únicamente las mediciones del LiDAR y la odometría.

Para ello se implementó el algoritmo **Follow the Gap**, un controlador **reactivo**: no utiliza mapa ni trayectoria precalculada, sino que en cada instante busca el espacio libre más conveniente y dirige el vehículo hacia él. Sobre esta base se incorporó un control de velocidad y de estabilidad basado en el modelo dinámico del vehículo, que fue lo que permitió reducir el tiempo de vuelta sin perder estabilidad.

Además, se desarrolló un nodo de cronometraje (`lap_counter.py`) que registra tiempos por vuelta y por sector, y genera estadísticas para comparar las pruebas.

## 2. Herramientas utilizadas

- **Ubuntu** – sistema operativo del entorno de simulación.
- **ROS 2 Humble** – comunicación entre nodos mediante tópicos.
- **Python** – lenguaje de los nodos.
- **F1TENTH Gym** – simulador del vehículo (modelo dinámico de una sola vía).
- **RViz** – visualización del vehículo, el mapa, la trayectoria y los tiempos.
- **Visual Studio Code** – editor de código.

## 3. Arquitectura

El sistema sigue el ciclo clásico de un vehículo autónomo: **percepción → planificación → control**.

| Nodo | Archivo | Suscribe | Publica | Función |
|---|---|---|---|---|
| `reactive_node` | `gap_node.py` | `/scan`, `/ego_racecar/odom` | `/drive`, `/trayectoria` | Controlador Follow the Gap |
| `lap_counter` | `lap_counter.py` | `/ego_racecar/odom` | — | Cronómetro de vueltas y sectores |

Los comandos se envían como mensajes `AckermannDriveStamped` (velocidad y ángulo de dirección).

## 4. Funcionamiento del controlador (`gap_node.py`)

El controlador se ejecuta con cada lectura del LiDAR y realiza las siguientes etapas:

### 4.1. Preprocesado del LiDAR
- Filtro de mediana para eliminar lecturas espurias.
- Interpolación de lecturas inválidas (NaN).
- Recorte al campo de visión frontal (±90°).

### 4.2. Inflado de obstáculos
- **Extensión de disparidades:** en cada borde de pared (salto brusco de distancia) se extiende el obstáculo el semiancho del vehículo más un margen. Así, cualquier espacio que queda libre es realmente transitable.
- **Burbuja de seguridad** de 0.30 m alrededor del punto más cercano.
- El margen de seguridad **aumenta con la velocidad**.

### 4.3. Selección del GAP
- Se identifican los tramos continuos de espacio libre (GAPs).
- Cada GAP recibe una puntuación adimensional según su **profundidad**, **ancho**, **alineación con el rumbo** y **cambio respecto al GAP anterior**.
- Se aplica **histéresis**: el GAP solo cambia si el nuevo es claramente mejor, lo que evita oscilaciones (zigzag).

### 4.4. Punto objetivo
- El objetivo es el **centro del tramo más profundo** del GAP elegido, no un rayo aislado, lo que da una referencia estable.
- En rectas se aplica **centrado lateral**; en curvas, una ligera **trazada interior** (máximo 0.5 m de desplazamiento y nunca a menos de 0.45 m de la pared).

### 4.5. Cálculo de la dirección
- **Geometría de arco** (modelo de bicicleta): δ = atan(L · 2·sen(α) / Ld), con distancia de mirada Ld = 1.2 m + 0.25 · v.
- **Compensación de subviraje:** a partir de la ganancia de guiñada en régimen permanente del modelo dinámico, se aumenta el ángulo de dirección con la velocidad (×1.30 a 6 m/s, ×2.22 a 12 m/s).

### 4.6. Control de velocidad
La velocidad objetivo es el **mínimo** de varios límites:
- **Por curvatura:** v = √(a_lat / κ).
- **Por frenado:** velocidad desde la que se puede frenar de forma estable dentro de la distancia libre medida **a lo largo del arco planificado**.
- **Por espacio para girar** hasta la salida de la curva.
- **Por cercanía a la pared** lateral.
- **Velocidad de curva adaptativa:** 8.0 m/s en curvas abiertas y 6.25 m/s en horquillas, según la distancia libre.

### 4.7. Estabilidad en frenada
- En el modelo del simulador, una frenada fuerte a alta velocidad transfiere carga al eje delantero y vuelve inestable la guiñada (trompo).
- Al iniciar, el nodo calcula una **tabla de deceleración estable** a partir del análisis de estabilidad del modelo dinámico, y nunca solicita más deceleración que la estable a la velocidad actual.
- **Realimentación de velocidad de guiñada** activa solo durante frenadas fuertes, comparada con un modelo de referencia que incluye el retardo de la dirección.

### 4.8. Seguridad
- **Frenado de emergencia (AEB)** por tiempo hasta colisión: si TTC < 0.20 s, frena al máximo estable.
- **Recuperación anti-atasco** mediante maniobra de reversa.
- **Fin de carrera:** al completar 10 vueltas el vehículo frena de forma progresiva y se detiene.

## 5. Contador de vueltas (`lap_counter.py`)

- Detecta el cruce de la línea de meta (x = `finish_x`, entre `finish_y_min` y `finish_y_max`) en sentido positivo, con rearme al alejarse de la línea.
- El cronómetro arranca **cuando el vehículo empieza a moverse**.
- El instante de cruce se **interpola** entre dos mensajes de odometría.
- Por vuelta: tiempo, diferencia con la mejor vuelta, distancia, velocidad media y máxima, y **3 sectores**.
- Resumen final: mejor vuelta, promedio, consistencia (desviación) y **vuelta ideal** (suma de los mejores sectores).
- Historial en `~/f1tenth_logs/sesiones_sepang.csv`.

## 6. Visualización en RViz

El controlador publica en el tópico `/trayectoria` (`MarkerArray`):
- la trayectoria recorrida, coloreada según la velocidad;
- la vuelta anterior en gris;
- el punto objetivo del GAP;
- un texto con la vuelta actual, la última y la mejor.

Para verla, en RViz: **Add → By topic → /trayectoria → MarkerArray**.

## 7. Parámetros principales

| Parámetro | Valor | Descripción |
|---|---|---|
| `max_speed` | 12.0 m/s | Velocidad máxima |
| `corner_min_speed` | 8.0 m/s | Velocidad mínima objetivo en curvas abiertas |
| `corner_min_sharp` | 6.25 m/s | Velocidad objetivo en horquillas |
| `a_lat_max` | 10.0 m/s² | Aceleración lateral máxima |
| `ld_min` / `ld_gain` | 1.2 m / 0.25 s | Distancia de mirada |
| `fov_deg` | 90° | Campo de visión utilizado |
| `car_half_width` | 0.155 m | Semiancho del vehículo |
| `bubble_radius` | 0.30 m | Burbuja de seguridad |
| `line_max_offset` | 0.5 m | Desplazamiento máximo de la trazada interior |
| `ttc_aeb` | 0.20 s | Umbral del frenado de emergencia |
| `race_laps` | 10 | Vueltas de la carrera |

## 8. Estructura del proyecto

```text
F1Tenth-Repository/
│
├── README.md
├── README_es.md
├── iniciar_sepang.sh
├── bag_a_csv.py
├── resumen_carrera.py
│
└── src/
    ├── controllers/
    │   ├── controllers/
    │   │   ├── gap_node.py
    │   │   └── lap_counter.py
    │   ├── package.xml
    │   └── setup.py
    │
    └── f1tenth_gym_ros/
        ├── config/
        ├── launch/
        └── maps/
            ├── Sepang_map.png
            └── Sepang_map.yaml
```

## 9. Compilación

```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
colcon build --packages-select controllers f1tenth_gym_ros --symlink-install
source install/setup.bash
```

## 10. Ejecución

```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
source install/setup.bash
./iniciar_sepang.sh
```

El script termina solo: el vehículo se detiene al completar 10 vueltas, se cierran el simulador y RViz, se imprime el cuadro resumen y la grabación rosbag se convierte a CSV. La carrera terminó bien cuando aparecen **"Carrera terminada."**, después **"CSV guardados en ..."** y vuelve el prompt. **No se debe presionar Ctrl+C mientras convierte.**

**Grabación rosbag:** se graban `/scan`, `/ego_racecar/odom` y `/drive` en `~/f1tenth_bags/PARTE1_<fecha>/`. Al terminar, `bag_a_csv.py` crea en la carpeta `csv/`:

| Archivo | Columnas |
|---|---|
| `ego_racecar_odom.csv` | tiempo, x, y, rumbo, velocidad lineal x/y, velocidad angular z |
| `drive.csv` | tiempo, velocidad comandada, ángulo de dirección |
| `scan.csv` | tiempo, ángulo inicial, incremento, r0 … rN (distancias del LiDAR) |

Si la carrera se corta con Ctrl+C, la grabación queda sin convertir y se convierte a mano:

```bash
python3 bag_a_csv.py ~/f1tenth_bags/PARTE1_<fecha>
```

Si quedaron procesos de una ejecución anterior, se pueden cerrar con:

```bash
pkill -f gym_bridge; pkill -f gap_node; pkill -f lap_counter; pkill -f rviz2; pkill -f "ros2 bag"
```

Los registros de cada ejecución se guardan en `~/f1tenth_logs/`.

## 11. Resultados

### Ejecución final (10 vueltas consecutivas)

| Métrica | Valor |
|---|---|
| Mejor vuelta | **49.09 s** |
| Promedio de vueltas lanzadas | 49.22 s |
| Consistencia (desviación entre vueltas) | 0.07 s |
| Holgura mínima a la pared | 0.30 – 0.45 m |
| Activaciones del AEB | **0** |
| Colisiones | **0** |
| Distancia por vuelta | ~486 m |

### Alternativas evaluadas y descartadas
- **Mayor velocidad en horquillas (6.5 m/s):** reducía ~0.2 s por vuelta, pero en una vuelta la holgura bajó a 0.22 m y se activó el AEB.
- **Velocidad máxima mayor a 12 m/s:** aumentaba el tiempo de vuelta, porque las frenadas estables se alargan.
- **Aprendizaje iterativo del perfil de velocidad:** el vehículo aprendía vuelta a vuelta dónde frenar antes y dónde acelerar. Logró 48.64 s, pero aumentó la variabilidad entre vueltas (0.48 s) y produjo un casi-choque, por lo que se priorizó la confiabilidad.

## 12. Conclusiones

- Follow the Gap permite recorrer el circuito usando solo el LiDAR, sin mapa ni trayectoria predefinida.
- El tiempo de vuelta no estaba limitado por la velocidad en rectas, sino por la **estabilidad en frenada y en curva**. Incorporar el modelo dinámico del vehículo (frenada estable, compensación de subviraje) fue lo que permitió alcanzar un tiempo de vuelta de 49.09 s manteniendo la estabilidad.
- Las métricas registradas (holgura mínima, AEB, consistencia) permitieron justificar cada cambio con datos y descartar las mejoras que comprometían la seguridad.
- Como trabajo futuro se plantea un planificador global con localización y control predictivo (MPC) para optimizar la trazada completa.