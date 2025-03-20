
# Projet Robot Pick and Place avec détection d'image et navigation autonome

## 1.Description

  Ce projet implémente un système robotique autonome capable de réaliser des opérations  de Pick and Place en utilisant la détection d'images pour identifier les objets, la navigation (Nav2)
  pour se déplacer vers les objets et les zones de dépôt, ainsi que la manipulation robotique pour la prise et le placement (MoveIt2).


## 2.Installation 
   ROS2 Jazzy + robots DevContainer
   This is a Dev container for Visual Studio Code, offering ROS 2 Jazzy and PAL Tiago robot.

## 3.Start DevContainer
  You need Visual Studio Code preinstalled
   ```bash
  ** git clone https://gitlab.com/f2m2robserv/jazzy-ros-ynov/**

  1.Visual Studio Code et Docker doivent être installés avant de lancer le conteneur:

   ```bash

   **code jazzy-ros-ynov/**
    ```

  2.When VSCode opens, trust the sources, and accept the installation of the Dev Container extension.

  3.To build the workspace use:

   ```bash
   **cd ~/ros2_ws
   colcon build --symlink-install
   source ~/.bashrc**
    ```


  4.important : Après chaque modification du code, il est nécessaire  de reconstruire le projet en utilisant la commande suivante (le  workspace):

   ```bash
   **colcon build**

 Puis, sourcez l'environnement avec :

   ```bash
 **source install/setup.bash**

 5.Cela garantit que les changements sont correctement pris en compte dans le système.



## Tiago 2D navigation using Nav2

 Documentation : https://docs.pal-robotics.com/sdk-dev/navigation

 1.Start Gazebo simulation of Tiago robot:
   ```bash
  **
   ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=pick_and_place**

 2.In a new terminal, start cartographer to run SLAM:
   ```bash

   **ros2 launch tiago_2dnav tiago_nav_bringup.launch.py slam:=True is_public_sim:=True**

 3.In a new terminal, start teleoperation from the keyboard:
   ```bash

   **ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/key_vel**
    ```


 4.Then move the robot to accumulate map data.
 Close teleoperation with Ctrl+C and save map to file our_map using:
   ```bash

   **ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/pal_maps/maps/our_map/map**




 5.Close the SLAM terminal with Ctrl+C.
 If you have not used --symlink-install, workspace building and sourcing are needed so that the new map is installed.

 6.Start 2D navigation by loading your our_map map, using:
   ```bash

   **ros2 launch tiago_2dnav tiago_nav_bringup.launch.py is_public_sim:=True world_name:=our_map**





## teleoperation and navigation both publish Twist robot commands   to /cmd_vel: make sure they don't publish at the same time, your robot would      receive contradictory commands.
 In RViz use the arrows to control navigation:


 1.Estimate 2D pose to initialize random particles to an approximate position around the actual robot position
 Send a 2D nav goal to any point of the map: your robot must navigate there

 2.Command navigation from Python code using the navigate_to_pose action service (see workshops instructions)
 4-waypoints robot patrol: Inspire from the goto code to add a new patrol node, endlessly patroling between 4 map poses.

 3.Tiago arms and gripper manipulation using MoveIt2
 Documentation: https://docs.pal-robotics.com/sdk-dev/manipulation

 
 4.Command MoveIt using color handles from the GUI in RViz:
   ```bash

  **ros2 launch tiago_moveit_config moveit_rviz.launch.py**
   ```
 
 
 5.Command MoveIt from Python file ros2_ws/src/tiago_pick_and_place/tiago_pick_and_place/pick.py:
  ```bash
  **ros2 launch tiago_pick_and_place plan.launch.py use_sim_time:=True**
   ```












