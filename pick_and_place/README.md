
# Projet Robot Pick and Place avec détection d'image et navigation autonome

## 1.Description

  Ce projet implémente un système robotique autonome capable de réaliser la détection d'objets ( dans notre cas :une canette rouge) en utilisant OpenCV pour le traitement d'image. Il identifie les objets et utilise la navigation (Nav2) pour se déplacer vers une position où le robot peut centrer la canette rouge, facilitant ainsi la mise en œuvre d'une éventuelle opération de Pick and Place.

## 2.Installation 

 ### Prérequis

Avant de commencer, assurez-vous d'avoir les éléments suivants installés :

- **ROS 2 Jazzy** (ou toute version compatible de ROS 2)
- **Docker** pour exécuter des conteneurs Dev
- **Visual Studio Code** 

  robots DevContainer:
   This is a Dev container for Visual Studio Code, offering ROS 2 Jazzy and PAL Tiago robot.

## 3.Start DevContainer
  You need Visual Studio Code preinstalled
   ```bash
  git clone https://gitlab.com/f2m2robserv/jazzy-ros-ynov/
  ```

  1.Visual Studio Code et Docker doivent être installés avant de lancer le conteneur:

   ```bash

   code jazzy-ros-ynov/

   ```
    

  2.When VSCode opens, trust the sources, and accept the installation of the Dev Container extension.

  3.To build the workspace use:
  ```bash
   
   cd ~/ros2_ws
   colcon build --symlink-install
   source ~/.bashrc

   ```
    

   
  4.Important : Après chaque modification du code, il est nécessaire  de reconstruire le projet en utilisant la commande suivante (le  workspace):
   
   **colcon build**

 Puis, sourcez l'environnement avec :
 ```bash

 source install/setup.bash

  ```

 5.Cela garantit que les changements sont correctement pris en compte dans le système.





## Lancement du projet 

 Documentation : https://docs.pal-robotics.com/sdk-dev/navigation

 1.Start Gazebo simulation of Tiago robot:
   ```bash
  
   ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=pick_and_place

   ```

 2.Lancer le noeud pour le traitement d' image :

   ```bash

      ros2 run pick_and_place detection_red

  ```



 3.Start 2D navigation by loading your our_map map, using:
    **pour permettre au robot de ce localiser sur la map**
    In RViz use the arrows to control navigation:


  1.Estimate 2D pose to initialize random particles to an approximate position around the actual robot position

  ```bash 
   ros2 launch tiago_2dnav tiago_nav_bringup.launch.py is_public_sim:=True world_name:=our_map
  ```



 2.Lancer le noeud pour la navigation la navigation : 
   ```bash

    ros2 run pick_and_place goto

   ```
    
 3.Lancer le noeud pour le pick and place :
   **code à paufiner**
  ```bash

    ros2 run pick_and_place pick

  ```

  
4.Lancer un launch
  Le CLI ROS permet de démarrer des launch files avec :
  **code à paufiner**
  ```bash

      ros2 launch pick_and_place  pick_canette.launch.py

  ```




## Command navigation from Python code using the navigate_to_pose action service (see workshops instructions)
 4-waypoints robot patrol: Inspire from the goto code to add a new patrol node, endlessly patroling between 4 map poses.

 3.Tiago arms and gripper manipulation using MoveIt2
 Documentation: https://docs.pal-robotics.com/sdk-dev/manipulation

 
 4.Command MoveIt using color handles from the GUI in RViz:
   ```bash
  ros2 launch tiago_moveit_config moveit_rviz.launch.py
   
   ```
 

 5.Command MoveIt from Python file ros2_ws/src/pick_and_place/pick_and_place/pick.py:
   ```bash
  ros2 launch pick_and_place plan.launch.py use_sim_time:=True
   ```












