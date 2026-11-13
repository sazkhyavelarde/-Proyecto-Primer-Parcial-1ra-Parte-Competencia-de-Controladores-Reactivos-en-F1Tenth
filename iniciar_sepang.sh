#!/bin/bash

set -e

cd ~/F1Tenth-Repository

source /opt/ros/humble/setup.bash
source install/setup.bash

echo "===================================="
echo " F1TENTH - SEPANG"
echo " Simulador + Contador + Follow the Gap"
echo "===================================="

PIDS=()

cleanup() {
    trap - INT TERM EXIT

    echo ""
    echo "Deteniendo F1Tenth..."

    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done

    wait 2>/dev/null || true
}

trap cleanup INT TERM EXIT

# Iniciar simulador
ros2 launch f1tenth_gym_ros gym_bridge_launch.py &
PIDS+=($!)

sleep 8

# Iniciar contador de vueltas
ros2 run controllers lap_counter &
PIDS+=($!)

sleep 2

# Iniciar Follow the Gap
ros2 run controllers gap_node &
PIDS+=($!)

echo ""
echo "F1Tenth iniciado."
echo "Presiona Ctrl+C para detener."

wait
