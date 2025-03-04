# Spor_Switch_Map_System

## Overview
This project focuses on implementing Boston Dynamics' Spot robot for automated inspection tasks in industrial environments. Under strict cybersecurity regulations, we have developed an advanced automated site inspection and remote monitoring system. This system aims to enhance automation levels and monitoring efficiency in industrial environments, bringing significant operational benefits and competitive advantages to enterprises.

![Spot Robot](./images/portfolio_spot.jpg)

## **Framework**

### **Map Manager**
- Responsible for determining when to switch between maps and sending goal poses to the robot.

### **Spot Graph Nav**
- Enables autonomous navigation by allowing Spot to navigate to target locations based on user-defined waypoints.

### **Switch PCD Map**
- Handles map publishing to the localization system and determines the robot's initial position by detecting nearby AprilTags.


## **Demo**

### **City Science Lab Test Demo**
[![City Science Lab Test Demo](./images/demo_csl.png)](https://youtu.be/ZL_bb400D3o?si=JQu_fL6QrJcjI_Wx)

- Showcases system performance in controlled laboratory conditions.
- Demonstrates precise navigation and mapping capabilities.
- Validates map switching functionality in indoor environments.

### **Outdoor Test Demo**
[![Outdoor Test Demo](./images/demo_outdoor.png)](https://youtu.be/LI97OdDDBUY?si=Lni16U4oqysuG10Y)

- Illustrates system robustness in challenging outdoor conditions.
- Demonstrates real-world application scenarios.
- Validates system performance across varied terrain and lighting conditions.

## **Launch Instructions**

### **Start the System**
Open the following terminals and execute the respective commands:

#### **Terminal 1**
```sh
cd spot_demo/
sh start_demo.sh
```

#### **Terminal 2**
```sh
cd routing_engine_ws/
ros2 launch routing_agent RunServer.launch.py
```

#### **Terminal 3**
```sh
ssh Spot 
cd spot_map_switching_ws/
ros2 run spot_graph_nav spot_graph_nav_node 192.168.50.3
```

#### **Terminal 4**
```sh
cd spot_map_switching_ws/
ros2 run switch_pcd_map switch_pcd_map_node
```

#### **Terminal 5**
```sh
cd spot_map_switching_ws/
ros2 run map_manager map_manager_node
```

### **Visualizing the Graph**

#### **Terminal 1**
```sh
cd spot_map_switching_ws/
ros2 run switch_pcd_map switch_pcd_map_node
```

#### **Terminal 2**
```sh
cd spot_map_switching_ws/
ros2 service call /switch_map switch_map_interfaces/srv/SingleMap "mapid: 0"
```

#### **Terminal 3**
```sh
cd routing_engine_ws/src/routing_engine/graph_collector_and_visualization
python3 graph_visualizer.py
```

**Note:** Modify line 30 in `graph_visualizer.py` to match the corresponding waypoint file path.

