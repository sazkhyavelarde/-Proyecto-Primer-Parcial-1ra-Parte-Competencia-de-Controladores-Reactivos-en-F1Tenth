source /opt/ros/humble/setup.bash
source ~/F1Tenth-Repository/install/setup.bash
export F1TENTH_SIM_CONFIG=~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim_obstaculos.yaml
INICIO=$(date +%Y-%m-%dT%H:%M:%S)
RESUMEN=/tmp/resumen_obstaculos.txt
rm -f "$RESUMEN"

terminar() {
    trap '' INT TERM          # este script sigue vivo para mostrar el cuadro
    kill -TERM 0 2>/dev/null  # cierra todo lo que arrancó (simulador, RViz, nodos)
    sleep 3
    [ -s "$RESUMEN" ] && cat "$RESUMEN"
    exit 0
}
trap terminar EXIT INT TERM

ros2 launch f1tenth_gym_ros gym_bridge_launch.py &
sleep 6
ros2 run controllers lap_counter &
sleep 1

# Mismo controlador de la parte 1, con margen extra solo junto a obstáculos pequeños.
ros2 run controllers gap_node --ros-args -p run_tag:=OBSTACULOS -p obs_margin_extra:=0.08 -p race_laps:=10 &

# Espera a que termine la carrera y guarda el cuadro resumen
python3 ~/F1Tenth-Repository/resumen_carrera.py --tags OBSTACULOS --laps 10 --since $INICIO > "$RESUMEN"
sleep 2