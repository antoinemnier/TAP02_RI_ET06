# TAP02_RI_ET06 - FANUC M-10iA Pick-and-Place

**Universidad EIA - Robótica Industrial**
**Equipo ET06**: Antoine Meunier & Diego Oxman

Este repositorio contiene la configuración de MoveIt 2 y los scripts de ejecución para el ciclo de pick-and-place del robot FANUC M-10iA, desarrollado para el Taller de Cinemática y Planeación de Movimiento.

## 1. Requisitos
* Ubuntu 24.04 LTS
* ROS 2 Jazzy
* MoveIt 2
* MATLAB (para la validación del modelo DH y el Jacobiano analítico)

## 2. Compilación del Workspace
Clona este repositorio en tu carpeta personal y compílalo usando `colcon`:

```bash
cd ~/TAP02_RI_ET06
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install

## Terminal 1: Lanzar MoveIt y RViz
cd ~/TAP02_RI_ET06
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch et06_m10ia_moveit_config demo.launch.py

## Terminal 2: Ejecutar el script principal
cd ~/TAP02_RI_ET06
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/et06_pick_place/scripts/run_pick_and_place.py

Cuando el programa lo solicite, escribe START y presiona Enter para ver la animación completa con evasión de obstáculos y acercamiento fino.
