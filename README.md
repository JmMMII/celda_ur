# UR3e Pick and Place Simulation

Este repositorio contiene la configuración de simulación y el código para realizar tareas de **pick and place** con un brazo robótico **UR3e** utilizando **ROS 2 Humble** y **MoveIt 2**. Se incluye una celda personalizada y un entorno de simulación utilizando `ros2_control`.

## Requisitos

- ROS 2 Humble
- MoveIt 2
- Universal Robots `ur_robot_driver`
- Paquetes personalizados:
  - `celda_control`
  - `celda_moveit_config`
  - `ur3e_pick_and_place`

## Cómo ejecutar la simulación

1. **Iniciar el robot con hardware simulado (fake hardware):**

   ```bash
   ros2 launch celda_control iniciar_robot.launch.py use_fake_hardware:=true launch_rviz:=false
