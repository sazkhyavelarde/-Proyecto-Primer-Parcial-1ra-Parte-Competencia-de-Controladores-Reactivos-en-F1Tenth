source /opt/ros/humble/setup.bash
source ~/F1Tenth-Repository/install/setup.bash
export F1TENTH_SIM_CONFIG=~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim_2robots.yaml
INICIO=$(date +%Y-%m-%dT%H:%M:%S)
RESUMEN=/tmp/resumen_parte2.txt
rm -f "$RESUMEN"
mkdir -p ~/f1tenth_bags
BAG=~/f1tenth_bags/PARTE2_$(date +%Y%m%d_%H%M%S)
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

ros2 launch f1tenth_gym_ros gym_bridge_launch.py &
sleep 6

# Grabación rosbag de LiDAR, odometría y comando de los dos carros. Se arranca con
# Ctrl+C habilitado para poder cerrarla bien al final.
python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
  ros2 bag record -o "$BAG" /scan /ego_racecar/odom /drive /opp_scan /opp_racecar/odom /opp_drive \
  > /tmp/rosbag_parte2.log 2>&1 &
BAG_PID=$!

# Contadores de vueltas: uno por carro
ros2 run controllers lap_counter &
ros2 run controllers lap_counter --ros-args -r __node:=lap_counter_oponente \
  -r /ego_racecar/odom:=/opp_racecar/odom -p log_dir:=$HOME/f1tenth_logs/oponente &
sleep 1

# Oponente: mismo controlador, a 8.7 m/s como máximo (unos 57 s por vuelta) y cargado
# a la derecha. Trayectoria en azul en RViz y texto debajo del carro.
ros2 run controllers gap_node --ros-args -r __node:=opponent_node \
  -r /scan:=/opp_scan -r /drive:=/opp_drive -p odom_topic:=/opp_racecar/odom \
  -p max_speed:=8.7 -p lane_bias:=-0.4 -p obs_margin_extra:=0.08 \
  -p overtake_enabled:=true -p ot_speed_only:=true \
  -p run_tag:=OPONENTE -p race_laps:=10 \
  -p viz_laser_frame:=opp_racecar/laser -p "viz_text_color:=[0.2,0.6,1.0]" \
  -p viz_text_dy:=-3.5 -p viz_follow_dy:=-3.5 -p viz_speed_colors:=false \
  > /tmp/oponente.log 2>&1 &
sleep 1

# Carro principal: el mismo Follow the Gap con el manejo de obstáculos de la prueba 1.
# El oponente es para él un obstáculo más (dinámico): lo esquiva por el hueco libre.
# viz_rival_odom: odometría del otro carro (aviso de rebase en RViz y, con safe_rival,
# para no chocarlo: en curvas y obstáculos va detrás de él y lo pasa en recta).
# ot_speed_only (en los dos carros): si el otro va justo adelante en el mismo carril, no se le
# llega encima a toda velocidad; para pasarlo, Follow the Gap busca el hueco libre a su lado.
ros2 run controllers gap_node --ros-args -p viz_rival_odom:=/opp_racecar/odom \
  -p overtake_enabled:=true -p ot_speed_only:=true -p safe_rival:=true \
  -p obs_margin_extra:=0.15 -p obs_brake_walls:=true -p obs_angle_deg:=100.0 -p free_window_deg:=1.5 \
  -p run_tag:=PRINCIPAL -p race_laps:=10 &

# Espera a que terminen los dos carros y guarda el cuadro resumen
python3 ~/F1Tenth-Repository/resumen_carrera.py --tags PRINCIPAL OPONENTE --laps 10 --since $INICIO > "$RESUMEN"
sleep 2