# Spor_Switch_Map_System

This project focuses on implementing Boston Dynamics' Spot robot for automated inspection tasks in industrial environments. Under strict cybersecurity regulations, we have developed an advanced automated site inspection and remote monitoring system. This system aims to enhance automation levels and monitoring efficiency in industrial environments, bringing significant operational benefits and competitive advantages to enterprises.


<img src="./images/portfolio_spot.jpg" width="500px">

## **Framework**

### Map Manager
- Responsible for determining when to switch between maps and sending goal poses to the robot.
### Spot Graph Nav
- Enables autonomous navigation by allowing Spot to navigate to target locations based on user-defined waypoints.
### Switch PCD Map
- Handles map publishing to the localization system and determines the robot's initial position by detecting nearby AprilTags.

<img src="./images/switch_map_framework_v2.png" width="640px">

## **Demo**
### City Science Lab Test Demo
<a href="https://youtu.be/ZL_bb400D3o?si=JQu_fL6QrJcjI_Wx" title="Link Title"><img src="./images/demo_csl.png" alt="Alternate Text" /></a>

- Showcases system performance in controlled laboratory conditions
- Demonstrates precise navigation and mapping capabilities
- Validates map switching functionality in indoor environments

### Outdoor Test Demo
<a href="https://youtu.be/LI97OdDDBUY?si=Lni16U4oqysuG10Y" title="Link Title"><img src="./images/demo_outdoor.png" alt="Alternate Text" /></a>

- Illustrates system robustness in challenging outdoor conditions
- Demonstrates real-world application scenarios
- Validates system performance across varied terrain and lighting conditions
