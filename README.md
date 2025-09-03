Para el uso de estos paquetes se deben colocar el la carpeta src de un worspace de ROS2 y construirlo con colcon build

1. **Run simulation (fake hardware):**
   ```bash
   ros2 launch celda_control iniciar_robot.launch.py use_fake_hardware:=true launch_rviz:=false
   ros2 launch celda_moveit_config move_group.launch.py
   ros2 launch celda_moveit_config moveit_rviz.launch.py

2. **Run pick and place code**
   ```bash
   ros2 launch ur3e_pick_and_place celda_pick_and_place.launch.py
