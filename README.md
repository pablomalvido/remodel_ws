# An Approach for the Automatic Assembly of Multi-branch Wire Harnesses in the Automotive Industry using a Dual-Arm Robot

This is the repository of the journal article entitled: "An Approach for the Automatic Assembly of Multi-branch Wire Harnesses in the Automotive Industry using a Dual-Arm Robot", which is currently under review.

## System structure

The proposed ROS system is composed of six main modules, which interact between them to achieve the desired system functionality.

![github_system](https://github.com/user-attachments/assets/8c1fc257-88da-4e02-9c96-7e46d2325af0)

### CAD Platform

This module extracts relevant information from a set of input files and provides it to the rest of the modules using ROS services. This information is the used to configure the entire system. For instance, this module is able to extract points of interest from CAD files to configure the robot trajectories. This module is provided by the **CAD_Plartform_pkg** ROS package and can be started using its launcher.launch file.

### Task planner and robot operations

The system has a set of predefined configurable basic robot operations, which can be combined to perform to entire wire harness assembly. The task planner module is responsible of selecting the optimal sequence of robot operations, and to configure them. To decide this, the task planner requests information from the CAD Platform module. This module is provided by the **task_planner_pkg** ROS package and can be started using its process.launch file.

### Perception module

This module provides vision capabilities to perceive the shape of the wire harnesses. It allows to estimate the shape of all their wires, identify the optimal grasp point to separate two cable groups, and evaluate the result of the cable separation operation. All these functionalities are provided in the form of ROS services that can be requested by the robot operations. This module is provided by the **vision_wh_pkg** ROS package and can be started using its vision.launch file. Additional information about the vision algorithm and segmentation model can be found in these two journal articles: [An approach based on machine vision for the identification and shape estimation of deformable linear objects](https://www.sciencedirect.com/science/article/pii/S0957415823001411) and [Generation of realistic synthetic cable images to train deep learning segmentation models](https://link.springer.com/article/10.1007/s00138-024-01562-y).

### Advanced manipulation

This module provides advanced manipulation capabilities for the robot, including automatic tool changing, end effector cartesian speed control, dual-arm control with multiple coordination policies, and a path planning algorithm to avoid cable entanglements with the assembly platform components. All these functionalities are provided in the form of ROS services and actions that can be requested by the robot operations. This module is provided by the **advanced_manipulation_pkg** and the **path_planning_pkg** ROS packages, both located in the advanced_manipulation_module folder. This module is started simultaneously with the task planner using the process.launch file of the task_planner_pkg package. More information about the advanced_manipulation_pkg algorithms can be found in this journal article: [Extending the motion planning framework - MoveIt with advanced manipulation functions for industrial applications](https://www.sciencedirect.com/science/article/pii/S0736584523000352).

### User Interface

The entire system can be controlled, monitored, and configured using a Web-based Graphical User Interface (GUI). The repositories of this GUI can be found at: [https://github.com/pablomalvido/UI_REMODEL](https://github.com/pablomalvido/UI_REMODEL) (front-end), and [https://github.com/pablomalvido/ROS_UI_backend](https://github.com/pablomalvido/ROS_UI_backend) (ROS back-end). More information about this component can be found in this journal article: ADD LINK

### Safety manager

This module communicates both with a safety PLC and with the task-planner to manage the safety of the system. This module is provided by the **remodel_safety_manager** ROS package and can be started using its RSM.launch file.

### Robot setup

The system was implemented using a dual-arm Motoman SDA10F robot, two Norbdo force sensors, and two WSG50 parallel grippers. The ROS workspace of the robotic platform setup can be found in the following repository: [https://github.com/pablomalvido/robots_ws](https://github.com/pablomalvido/robots_ws).

## System usage

To run the system in a virtual environment you must follow the next steps:

- Launch the GUI following the instructions of the provided GUI repositories.
- Launch the robot platform with roslaunch motoman_sda10f_moveit_config demo_no_gripper_camera.launch. This launch file is in the robots_ws workspace.
- Launch the CAD Platform with roslaunch CAD_platform_pkg launcher.launch.
- Launch the perception module with roslaunch vision_wh_pkg vision.launch.
- Launch the task planner and the advanced manipulation modules woth roslaunch task_planner_pkg process.launch.
- Go to the GUI 'Home' page and start the robot execution with the Start button.

Additionally, you can test the performance of the vision system algorithms running the example ROS service calls in the /vision_wh_pkg/test_service_calls.txt. Before this, the robot environment, the CAD Platform, and the perception module must be running.

## Experimental results

Click in the image below to see the videos of the experiments:

[![Watch the video](https://img.youtube.com/vi/v6aONASYWL4/maxresdefault.jpg)](https://www.youtube.com/watch?v=v6aONASYWL4)
