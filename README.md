# Spor_Switch_Map_System

An industrial inspection system was developed using Boston Dynamics' Spot. Implementing SLAM and Navigation via Spot SDK on ROS2, it features remote monitoring through a visualization interface. A Spot simulation in Isaac Sim validates algorithms and creates a Digital Twin.

<img src="./images/portfolio_spot.jpg" width="500px">

## **Framework**

### Map Manager
- Responsible for determining when to switch between maps and sending goal poses to the robot.
### Spot Graph Nav
- Enables autonomous navigation by allowing Spot to navigate to target locations based on user-defined waypoints.
### Switch PCD Map
- Handles map publishing to the localization system and determines the robot's initial position by detecting nearby AprilTags.

<img src="./images/switch_map_framework.png" width="640px">

## **Demo**
<a href="https://youtu.be/ZL_bb400D3o?si=JQu_fL6QrJcjI_Wx" title="Link Title"><img src="./images/demo.png" alt="Alternate Text" /></a>