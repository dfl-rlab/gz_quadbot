# Gazebo Go2 Quadbot for 3D Autonomous Navigation

## Overview
This repository provides a simplified simulation of the Unitree Go2 quadruped robot in a Gazebo world.  
It could integrate with the [dddmr_navigation](https://github.com/dfl-rlab/dddmr_navigation) stack and has already been tested for 3D mapping and navigation capabilities.

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/gazebo_3d_navigation/quad_3d_nav_gz_.gif" width="800" height="400"/>
</p>

## How to use
To make this simulation work with `dddmr_navigation`, you’ll need to follow the beginner guide first.  
Please head over to the [dddmr_beginner_guide](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_beginner_guide),  
where you’ll find a step-by-step tutorial on setting up and integrating the navigation stack.  

## Acknowledgements

This project is based on and extends the excellent work from [Unitree-go2-ros2](https://github.com/anujjain-dev/unitree-go2-ros2).  
We are grateful to the original author for the ROS2 integration of the Unitree Go2 robot in Gazebo.  
The original authors deserve full credit for the core work — we merely build on top of it.

As acknowledged in that project, credits also go to the following upstream works:
* [Unitree Robotics](https://github.com/unitreerobotics/unitree_ros) – For the Go2 robot description (URDF model).  
* [CHAMP](https://github.com/chvmp/champ) – For the quadruped controller framework.  
* [CHAMP Robots](https://github.com/chvmp/robots) – For robot configurations and setup examples.  


