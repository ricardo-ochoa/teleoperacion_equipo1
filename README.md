# PuzzleBot Nonholonomic Control Project
Framework de control no holonómico para un robot diferencial tipo PuzzleBot, con simulación en Gazebo, visualización en RViz2 y dashboard web para monitoreo en tiempo real.

Basado en:
https://github.com/nezih-niegu/pb-j_control
 , que al mismo tiempo está basado en: Ferguson, Donaire, Renton & Middleton (2018) — A port-Hamiltonian approach to the control of nonholonomic systems (arXiv:1801.06954v1).

## Team
- **Valentina Gonzalez Benedossi** | A00839507
- **Ricardo Gaspar Ochoa** | A00838841
- **Rhett Nieto Ramírez** | A01286100
- **Oscar Carranza Hernández** | A00838649
- **Jesús María Valderrama Pérez** | A01831016

## Features
- Simulación en **Gazebo**
- Visualización en **RViz2**
- Teleoperación por teclado
- Dashboard web con gráficas en vivo
- Seguimiento de trayectorias
- Comparación de controladores

## Controllers
- PID
- SMC
- ISMC
- CTC
- Port-Hamiltonian

## Prerequisites
```bash
sudo apt update
sudo apt install -y \
  ros-${ROS_DISTRO}-gazebo-ros-pkgs \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-tf-transformations \
  ros-${ROS_DISTRO}-rviz2

pip3 install tf-transformations numpy matplotlib
````

## Build
```bash
mkdir -p ~/pb_ws/src
cp -r puzzlebot_control ~/pb_ws/src/
cd ~/pb_ws
colcon build --packages-select puzzlebot_control
source install/setup.bash
```

## Quick Start

### Terminal 1
```bash
ros2 launch puzzlebot_control gazebo.launch.py
```

### Terminal 2
```bash
ros2 run puzzlebot_control teleop_keyboard
```

### Terminal 3
```bash
xdg-open http://localhost:8080
```

## Keyboard Controls
* **W / ↑**: Forward
* **S / ↓**: Backward
* **A / ←**: Left
* **D / →**: Right
* **Space**: Stop
* **R**: Reset
* **P**: Perturbations
* **Q**: Quit
* **1–5**: Change controller

## Objective

Analizar e implementar diferentes estrategias de control para un robot móvil no holonómico, evaluando su estabilidad, robustez y desempeño en seguimiento de trayectorias de manera digitá y física.
