# ROS nodes for REMODEL UI

  

This repository contains some of the nodes required to communicate with the REMODEL UI (https://github.com/pablomalvido/UI_REMODEL). Additionally, nodes from other packages, such as the CAD Platform, the robot pkg, the teaching system, or the RSM (REMODEL Safety Manager) are required. This package contains five nodes, launched by four launch files:

  

-  **launcher.launch:** Launches the **launch_script_multiple.py** node. This node contains services for launching and stopping a list of launch files, by specifying their packages and names. This node can be used directly and doesn't require modifications to be integrated with the UI.

  

-  **config.launch:** Launches the **write_files_UI.py** node. This node is used to edit and read the configuration parameters (robot speeds, sensors' calibration, offset values, etc.). There are two services to interact with the configuration file, */UI/get_config* and */UI/set_config* to read and edit the config parameters respectively by indicating the relative path of the config file and the new data. The config file must be csv following the structure of the /files/config.csv file. This node can be used directly and doesn't require modifications to be integrated with the UI (unless you change the location of the config file, then, you must update the path in the node).

  

-  **moveit_manual.launch:** Launches the **moveit_tests.py** node. This node contains services for the manual control of the robot. These services are: */UI/get_moveit_groups*, which retrieves the name of all the move_groups defined and their predefined configurations; */UI/move_group* to move the robot with the UI manual control (type=0 for motion to named target, type=1 for relative cartesian motion, and type=2 for absolute cartesian motion; and */UI/get_arms_pose* to get the current position of both robot arms. To use this node in the UI, the names of the motion groups of the robot must be updated.

  

-  **process.launch:** Launches the two following nodes:

	- **process_UI_server.py:** This node contains the action server of the robot process. Initially it request all the necessary information to perform the process to the CAD Platform (i.e., the sequence of operations, dimensions and structure of the handled wiring harness, coordinates of points of interest, etc.) and then it initializes the action server. The action server executes the different operations of the process, and it uses three variables (index, step1, and step2) to keep track of the status of the process, so it can be paused and resumed. All the motion planning functions are included in this file (cartesian speed control, synchronized dual-arm cartesian motion, motion stopping, etc.). This node can't be used directly with the UI as it is tailored for the Yaskawa robot and the ELVEZ use case, but it can be used as a reference to add this functionalities the system planners of other use cases.

	- **client_ELVEZ_test.py:** This node contains the action client of the robot process and different services to interact with the UI. These services allow the robot to start, stop, pause, resume, or run step by step (execute just the selected operation) the process. Additionally, it contains two topic publishers to indicate the UI the index of the current operation (within the whole process), and some logs to display in the UI. This node can be used directly with the UI, but it must be adapted to be compatible with the modifications in the **process_UI_server node**.

	- **publish_feedback.py:** This node publishes contain publishers that publish UI feedback. In particular, it publishes the speed of each end effector. This node must be modified for the number of robots used and their specific groups names.

-  **fake_sensors.launch:** Launches a node that publishes fake data for the force and tactile sensors.

-  **record_traj.launch:** Launches a node that simulates the services for recording and executing trajectories.

-  **update_files.launch:** Launches the **load_files.py** node. This node contains services for loading files from an USB flash drive. These services are: */UI/get_usbs*, which retrieves the name of all the USBs connected to the computer; */UI/get_files_usb* which retrieves the name of all the files and directories contained inside the selected USB; and */UI/copy_file* to copy-paste the selected file of the USB into a local directory.

# Additional considerations

- The CAD Platform nodes needs to provide a service server to request the sequence of all the operations of the process, so they can be displayed in the dropdown menu of the UI.
  
- The robot launcher file must spawn cameras in the required locations of the simulation environment. To stream the videos of this cameras in the UI, the name of the topics of the cameras must be updated in the Home.vue page of the UI to be compatible. You can find more information about this in the following link: https://roboticsknowledgebase.com/wiki/tools/stream-rviz/