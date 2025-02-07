#!/bin/bash

sudo timedatectl set-local-rtc 1


# Configuración
declare -A INTERFACES
INTERFACES["can0"]="500000"  # Bitrate para can0
INTERFACES["can1"]="1000000"  # Bitrate para can1
LOG_DIR="/home/arus/ARUS_logs/candumps"
SLEEP_TIME=3

# Crear el directorio de logs si no existe
mkdir -p "$LOG_DIR"



# Función para configurar una interfaz CAN y ejecutar candump
start_candump() {
    local interface=$1
    local bitrate=$2

    while true; do
        # Intentar configurar la interfaz
        sudo ip link set $interface up type can #bitrate $bitrate

        if [ $? -eq 0 ]; then
            # Guardar log cadump indefinidamente
            local log_file="$LOG_DIR/${interface}_candump_$(date +'%Y-%m-%d_%H-%M-%S').txt"

            candump $interface >> "$log_file" 2>&1 &

            echo "Interfaz $interface iniciada, guardando candump en ~/ARUS_logs/candumps"
            break

        else
            sleep $SLEEP_TIME
        fi
    done
}


# Bucle para manejar ambas interfaces CAN en paralelo
for interface in "${!INTERFACES[@]}"; do
    bitrate=${INTERFACES[$interface]}
    start_candump $interface $bitrate
done

# Ejecutar script de selección de misión
echo "Lanzando missionHandle"
source /opt/ros/humble/setup.bash
source /home/arus/ws/install/setup.bash
/home/arus/ws/src/DRIVERLESS2/src/common/common_meta/missionHandle

# Esperar indefinidamente para mantener el script activo
wait
