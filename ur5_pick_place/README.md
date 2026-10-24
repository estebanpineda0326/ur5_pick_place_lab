# ur5_pick_place

Paquete ROS 2 (Jazzy) con dos nodos para el UR5 + gripper simple del repositorio
[ur5_gripper_packages](https://github.com/DanielPalominoS/ur5_gripper_packages):

| Nodo | Qué hace |
| --- | --- |
| `pick_place_node` | Rutina Pick & Place sobre 3 puntos de interés (HOME, PICK, PLACE) con apertura/cierre de gripper, mensajes informativos, esperas de entradas digitales y activación de salidas digitales simuladas por tópicos. |
| `moveit_goal_node` | Planifica y ejecuta trayectorias con MoveIt 2 (acción `/move_action`): objetivo articular, objetivo cartesiano y planificación del grupo `gripper`. |

## Interfaces

| Interfaz | Tipo | Uso |
| --- | --- | --- |
| `/scaled_joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Movimiento del brazo |
| `/gripper_controller/commands` | `std_msgs/msg/Float64MultiArray` | Gripper (`[0.04,0.04]` abierto, `[0.004,0.004]` cerrado) |
| `/io/di0` `/io/di1` `/io/di2` | `std_msgs/msg/Bool` (entrada) | DI0 inicio de ciclo · DI1 pieza presente · DI2 zona de entrega libre |
| `/io/do0` `/io/do1` `/io/do2` | `std_msgs/msg/Bool` (salida) | DO0 robot listo · DO1 pieza sujeta · DO2 pulso de ciclo terminado |
| `/pick_place/status` | `std_msgs/msg/String` | Mensajes informativos de la rutina |

## Rutina

| Punto | Operaciones |
| --- | --- |
| P1 HOME | Mensaje · abrir gripper · DO0 ON · esperar DI0 · DO0 OFF |
| P2 PICK | Mensaje · esperar DI1 · bajar · cerrar gripper · DO1 ON · subir |
| P3 PLACE | Mensaje · esperar DI2 · bajar · abrir gripper · DO1 OFF · subir |
| P1 HOME | Regreso · pulso DO2 · mensaje de fin de ciclo |

## Uso

```bash
# Terminal 1: simulación
ros2 launch ur5_robotiq_bringup sim_gz_sgb.launch.py

# Terminal 2: rutina (cycles:=0 para ciclo infinito, auto_inputs:=true para no esperar DI)
ros2 launch ur5_pick_place pick_place.launch.py cycles:=1

# Terminal 3: simular entradas digitales cuando el nodo las pida
ros2 topic pub --once /io/di0 std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /io/di1 std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /io/di2 std_msgs/msg/Bool "{data: true}"

# Terminal 4: observar salidas y estado
ros2 topic echo /io/do1
ros2 topic echo /pick_place/status
```

### MoveIt sobre Gazebo

```bash
ros2 launch ur5_robotiq_bringup sim_gz_sgb.launch.py launch_rviz:=false
ros2 launch ur5_pick_place moveit_gazebo.launch.py
ros2 run ur5_pick_place moveit_goal_node --ros-args -p use_sim_time:=true
```

Requiere el paquete `ur5_robotiq_moveit` generado con el MoveIt Setup Assistant
(grupos `ur_arm` y `gripper`, controlador `scaled_joint_trajectory_controller`).
