
source /opt/ros/humble/setup.bash
source ~/F1Tenth-Repository/install/setup.bash
unset F1TENTH_SIM_CONFIG      # simulador con la configuración original (sim.yaml)
cd ~/F1Tenth-Repository

echo "===================================="
echo " F1TENTH - SEPANG"
echo " Simulador + Contador + Follow the Gap"
echo "===================================="

INICIO=$(date +%Y-%m-%dT%H:%M:%S)
RESUMEN=/tmp/resumen_parte1.txt
rm -f "$RESUMEN"
mkdir -p ~/f1tenth_bags
BAG=~/f1tenth_bags/PARTE1_$(date +%Y%m%d_%H%M%S)
BAG_PID=""

terminar() {
    trap - EXIT               # que el cierre se haga una sola vez
    trap '' INT TERM          # este script sigue vivo para mostrar el cuadro
    if [ -n "$BAG_PID" ]; then
        kill -INT "$BAG_PID" 2>/dev/null   # cerrar la grabación de forma ordenada
        sleep 3
    fi
    kill -TERM 0 2>/dev/null  # cierra todo lo que arrancó (simulador, RViz, nodos)
    sleep 3
    [ -s "$RESUMEN" ] && cat "$RESUMEN"
    if [ -d "$BAG" ]; then
        if grep -q "Carrera terminada" "$RESUMEN" 2>/dev/null; then
            echo "Convirtiendo la grabación a CSV..."
            python3 ~/F1Tenth-Repository/bag_a_csv.py "$BAG"
        else
            # carrera cortada con Ctrl+C: no se convierte (así se cierra rápido)
            echo "Carrera interrumpida: la grabación quedó en $BAG sin convertir."
            echo "Si la quieres en CSV: python3 ~/F1Tenth-Repository/bag_a_csv.py $BAG"
        fi
    fi
    exit 0
}
trap terminar EXIT INT TERM

# Iniciar simulador
ros2 launch f1tenth_gym_ros gym_bridge_launch.py &
sleep 8

# Grabación rosbag de LiDAR, odometría y comando. Se arranca con Ctrl+C habilitado
# para poder cerrarla bien al final (en un script, los procesos en segundo plano lo ignoran).
python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
  ros2 bag record -o "$BAG" /scan /ego_racecar/odom /drive > /tmp/rosbag_parte1.log 2>&1 &
BAG_PID=$!

# Iniciar contador de vueltas
ros2 run controllers lap_counter &
sleep 2

# Iniciar Follow the Gap (se detiene solo al completar race_laps = 10 vueltas)
ros2 run controllers gap_node --ros-args -p run_tag:=SEPANG -p race_laps:=10 &

echo ""
echo "F1Tenth iniciado. Grabando rosbag en $BAG"
echo "Se cierra solo al terminar las 10 vueltas (Ctrl+C para cortar antes)."

# Espera a que el carro termine sus 10 vueltas y guarda el cuadro resumen
python3 ~/F1Tenth-Repository/resumen_carrera.py --tags SEPANG --laps 10 --since $INICIO > "$RESUMEN"
sleep 2