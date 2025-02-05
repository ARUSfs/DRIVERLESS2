#!/bin/bash

# Buscar los PIDs de todos los procesos relacionados con ros2
processes=$(ps aux | grep -E '_exec|rviz|ros2' | grep -v grep)

# Si encontramos algún proceso, los mostramos y los matamos
if [ -n "$processes" ]; then
  # Obtener el PID de cada proceso y matarlos
  echo "$processes" | while read -r line; do
    pid=$(echo $line | awk '{print $2}')
    process_name=$(echo $line | awk '{print $11}')
    echo "Matando el proceso: $process_name (PID: $pid)"
    kill -2 $pid #Mandar SIGNINT para cerrar de manera ordenada
  done
else
  echo "No se encontraron procesos relacionados con ros2."
fi

