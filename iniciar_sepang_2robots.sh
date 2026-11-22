source /opt/ros/humble/setup.bash
source ~/F1Tenth-Repository/install/setup.bash
export F1TENTH_SIM_CONFIG=~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim_2robots.yaml
INICIO=$(date +%Y-%m-%dT%H:%M:%S)
RESUMEN=/tmp/resumen_2robots.txt
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

# Contadores de vueltas: uno por carro
ros2 run controllers lap_counter &
ros2 run controllers lap_counter --ros-args -r __node:=lap_counter_oponente \
  -r /ego_racecar/odom:=/opp_racecar/odom -p log_dir:=$HOME/f1tenth_logs/oponente &
sleep 1

# Oponente: mismo controlador, más lento (8.7 m/s) y cargado a la derecha.
# Trayectoria en azul en RViz y texto debajo de la salida.
ros2 run controllers gap_node --ros-args -r __node:=opponent_node \
  -r /scan:=/opp_scan -r /drive:=/opp_drive -p odom_topic:=/opp_racecar/odom \
  -p max_speed:=8.7 -p lane_bias:=-0.4 -p obs_margin_extra:=0.08 \
  -p run_tag:=OPONENTE -p race_laps:=10 \
  -p viz_laser_frame:=opp_racecar/laser -p "viz_text_color:=[0.2,0.6,1.0]" \
  -p viz_text_dy:=-3.5 -p viz_speed_colors:=false \
  > /tmp/oponente.log 2>&1 &
sleep 1

# Carro principal: con adelantamiento, se detiene a las 10 vueltas
ros2 run controllers gap_node --ros-args -p overtake_enabled:=true \
  -p run_tag:=PRINCIPAL -p race_laps:=10 &

# Espera a que terminen los dos carros y guarda el cuadro resumen
python3 ~/F1Tenth-Repository/resumen_carrera.py --tags PRINCIPAL OPONENTE --laps 10 --since $INICIO > "$RESUMEN"
sleep 2