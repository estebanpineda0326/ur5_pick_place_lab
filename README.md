# Laboratorio UR5 + Gripper — Pick & Place en ROS 2 Jazzy

Paquete `ur5_pick_place`: rutina Pick & Place simulada (3 puntos, gripper,
entradas/salidas digitales por tópicos) para el UR5 del repositorio
https://github.com/DanielPalominoS/ur5_gripper_packages

## Uso
    cd ~/ur5_ws/src && git clone <este repo>
    cd ~/ur5_ws && colcon build --packages-select ur5_pick_place
    source install/setup.bash
    ros2 launch ur5_robotiq_bringup sim_gz_sgb.launch.py
    ros2 launch ur5_pick_place pick_place.launch.py cycles:=1
    ros2 topic pub --once /io/di0 std_msgs/msg/Bool "{data: true}"

## Videos
- Prueba del gripper: <link>
- Rutina Pick & Place: <link>
