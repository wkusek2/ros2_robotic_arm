# ROS2 Robotic Arm 6-DOF

Projekt edukacyjny budowy systemu sterowania manipulatorem 6-stopniowym w ROS2 Humble.

## Stack technologiczny

- **ROS2 Humble**
- **C++** (węzły, kontrolery, kinematyka)
- **Python** (launch files)
- **Gazebo Fortress** (symulacja)
- **MoveIt2** (planowanie ruchu)
- **ros2_control** (framework kontrolerów)

## Struktura projektu

```
ros2_robotic_arm/
└── src/
    └── arm6dof/
        ├── src/arm/
        │   ├── armnode.cpp         # Główny węzeł ROS2 (ArmNode)
        │   ├── ArmController.hpp   # Kontroler ramienia
        │   ├── ArmController.cpp
        │   ├── CanBridge.hpp       # Warstwa komunikacji Waveshare USB-CAN-A
        │   └── CanBridge.cpp
        ├── launch/main.launch.py   # Launch file
        ├── urdf/arm6dof.urdf       # Opis robota
        └── urdf/meshes/            # Modele 3D STL
```

### Architektura

```
ArmNode (ROS2) → ArmController → CanBridge → USB-CAN-A → silniki AK series
```

## Status projektu

| Faza | Opis | Status |
|------|------|--------|
| 1 | Workspace ROS2, węzeł C++, launch file Python | ✅ |
| 2 | URDF 6-DOF, TF2, wizualizacja RViz2 | ✅ |
| 3 | CanBridge (Waveshare binarny protokół), ArmController, odczyt statusu (pos/vel/cur/temp), setPosMotor, setCurrentMotor, osobny wątek CAN, publikacja `joint_states` + `arm/diagnostics`, subskrypcje `arm/set_position` + `arm/set_current`, bufor stanów `getStates()` | ✅ |
| 4 | Symulacja Gazebo Fortress | ⏳ |
| 4 | ros2_control, Hardware Interface, kontrolery | ⏳ |
| 5 | Kinematyka FK/IK, parametry DH, KDL | ⏳ |
| 6 | MoveIt2, planowanie ruchu, unikanie kolizji | ⏳ |
| 7 | Action server/client, LifecycleNode, QoS | ⏳ |
| 8 | Testy, CI/CD, diagnostyki, Docker | ⏳ |

## Uruchomienie

```bash
# Zainstaluj narzędzia CAN
sudo apt install can-utils

# Zbuduj workspace
cd ~/ros2_robotic_arm
colcon build

# Załaduj środowisko
source install/setup.bash

# Uruchom węzeł
ros2 launch arm6dof main.launch.py
```

## Wymagania

```bash
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-ros-gz
sudo apt install can-utils
```

## Sprzęt

- Waveshare USB-CAN-A (CH340, protokół binarny 2 Mbps)
- Silniki CubeMars AK45-36 (VESC firmware) — POLE_PAIRS=14, GEAR_RATIO=36, Kt=2.009 Nm/A
- Sterowanie: SET_POS (cmd=4), SET_CURRENT (cmd=1) przez CAN
- Port: `/dev/ttyUSB*`

## Robot

Manipulator 6-DOF zaprojektowany w CAD. Modele 3D w formacie STL.
