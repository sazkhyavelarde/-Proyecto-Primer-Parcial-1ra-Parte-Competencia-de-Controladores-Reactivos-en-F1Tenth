# Proyecto F1TENTH – Competencia de Controladores Reactivos

## 1. Descripción del proyecto

El presente proyecto consiste en la implementación de un controlador reactivo para un vehículo autónomo F1TENTH, utilizando ROS 2 Humble y el simulador F1TENTH Gym.

El objetivo principal es lograr que el vehículo recorra de manera autónoma el circuito Sepang, mediante el procesamiento de las mediciones de un sensor LiDAR. Para ello, se implementó el algoritmo **Follow the Gap**, que permite identificar espacios libres y seleccionar una dirección de movimiento en función de los obstáculos detectados.

Además del controlador, se implementó un nodo para contabilizar las vueltas y registrar los tiempos obtenidos durante las simulaciones. Esto permite evaluar el desempeño del vehículo y comparar los resultados de las pruebas realizadas.

## 2. Herramientas utilizadas

Para el desarrollo del proyecto se utilizaron las siguientes herramientas:

- **Ubuntu:** sistema operativo utilizado para ejecutar el entorno de simulación.
- **ROS 2 Humble:** plataforma que permite la comunicación entre los nodos del sistema.
- **Python:** lenguaje utilizado para programar los controladores.
- **F1TENTH Gym:** simulador empleado para evaluar el comportamiento del vehículo.
- **RViz:** herramienta utilizada para visualizar el vehículo, el mapa y la información del entorno.
- **Visual Studio Code:** editor utilizado para desarrollar y modificar los archivos del proyecto.

El circuito seleccionado para las pruebas fue **Sepang**.

## 3. Funcionamiento del controlador Follow the Gap

El controlador principal se encuentra en el archivo `gap_node.py` y utiliza el algoritmo Follow the Gap para determinar la dirección de movimiento del vehículo.

El funcionamiento general del controlador se desarrolla de la siguiente manera:

### 3.1. Recepción de datos del LiDAR

El vehículo recibe las mediciones del sensor LiDAR mediante el tópico `/scan`.

Estas mediciones representan las distancias entre el vehículo y los obstáculos presentes en su entorno.

### 3.2. Procesamiento de las mediciones

Una vez recibidos los datos, el controlador analiza las distancias disponibles para identificar las zonas por las que el vehículo puede circular.

También considera márgenes de seguridad alrededor de los obstáculos, con la finalidad de reducir el riesgo de colisiones.

### 3.3. Identificación de espacios libres

El algoritmo busca los espacios libres entre los obstáculos detectados.

A partir de estos espacios, selecciona una dirección objetivo que permita continuar el recorrido.

### 3.4. Cálculo de la dirección

Después de seleccionar el punto objetivo, el controlador calcula el ángulo de dirección necesario para orientar el vehículo hacia el espacio libre.

El ángulo se ajusta considerando las condiciones del circuito y la respuesta del vehículo.

### 3.5. Control de velocidad

La velocidad no permanece constante durante todo el recorrido.

El controlador la modifica dependiendo de las condiciones del entorno. En las rectas puede alcanzar velocidades mayores, mientras que en las curvas se reduce para mejorar la estabilidad.

También se consideran condiciones de frenado cuando el espacio disponible no es suficiente para continuar de forma segura.

### 3.6. Publicación de comandos

Finalmente, el controlador publica los comandos de dirección y velocidad mediante el tópico `/drive`.

De esta manera, el vehículo puede modificar su movimiento continuamente a partir de las mediciones del LiDAR.

## 4. Contador de vueltas

Para evaluar el desempeño del controlador, se implementó un nodo adicional denominado `lap_counter.py`.

Este nodo utiliza la información de odometría publicada en el tópico `/ego_racecar/odom` para identificar el movimiento del vehículo y detectar el cruce de una línea de meta configurada.

Entre sus principales funciones se encuentran:

- Contabilizar las vueltas completadas.
- Registrar el tiempo de cada vuelta.
- Identificar la vuelta más rápida.
- Calcular la velocidad promedio y máxima.
- Dividir el recorrido en tres sectores para registrar tiempos parciales.
- Obtener estadísticas de las vueltas realizadas.
- Guardar los resultados en archivos CSV para su posterior análisis.

La ubicación de la línea de meta debe configurarse de acuerdo con el circuito utilizado.

## 5. Estructura del proyecto

Los principales archivos utilizados son:

```text
F1Tenth-Repository/
│
├── README.md
├── README_es.md
├── iniciar_sepang.sh
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

El archivo `gap_node.py` contiene el controlador reactivo, mientras que `lap_counter.py` se encarga de registrar los resultados de las vueltas.

El archivo `iniciar_sepang.sh` corresponde al script utilizado para iniciar la simulación.

## 6. Compilación del proyecto

Para compilar los paquetes se debe abrir una terminal de Ubuntu y ejecutar:

```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
colcon build --packages-select controllers f1tenth_gym_ros --symlink-install
source install/setup.bash
```

## 7. Ejecución de la simulación

Una vez compilado el proyecto, se puede iniciar la simulación mediante el siguiente comando:

```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
source install/setup.bash
./iniciar_sepang.sh
```

El script permite iniciar la simulación según la configuración establecida en el proyecto.

El controlador y el contador de vueltas deben encontrarse en ejecución para realizar el recorrido y registrar los resultados. Si el script no inicia alguno de estos nodos, será necesario ejecutarlo por separado.

## 8. Evaluación del desempeño

El desempeño del controlador se analiza principalmente mediante los tiempos de vuelta y el comportamiento del vehículo durante el recorrido.

Durante las pruebas se consideran aspectos como la capacidad para completar el circuito, la estabilidad en las curvas, la velocidad alcanzada y la respuesta frente a los obstáculos.

Los registros obtenidos mediante el contador de vueltas permiten comparar diferentes ejecuciones e identificar posibles mejoras en la configuración del controlador.

## 9. Conclusiones

La implementación del algoritmo Follow the Gap permite controlar el movimiento del vehículo F1TENTH a partir de las mediciones del sensor LiDAR, sin necesidad de establecer previamente una trayectoria completa.

El ajuste de la velocidad y la dirección según las condiciones del entorno permite evaluar el comportamiento del vehículo en diferentes secciones del circuito Sepang.

Por otra parte, el contador de vueltas facilita el registro y análisis de los resultados, proporcionando información útil para comparar el desempeño del controlador durante las simulaciones.