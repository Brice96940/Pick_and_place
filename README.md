# Projet Robot Pick and Place avec détection d'image et navigation autonome

## I. Bref description

  Ce projet implémente un système robotique autonome capable de réaliser la détection d'objets ( dans notre cas :une canette rouge) en utilisant OpenCV pour le traitement d'image et de centrer le robot par rapport a l'objet detecté. 

## II. Installation 

### Prérequis

Avant de commencer, assurez-vous d'avoir les éléments suivants installés :

- **ROS 2 Jazzy** (ou toute version compatible de ROS 2)
- **Visual Studio Code** 

  
## III. Instructions de lancement

### 1.Démarrer DevContainer
  
    ```bash
      git clone https://gitlab.com/f2m2robserv/jazzy-ros-ynov/
    ```
    
  1.Visual Studio Code et Docker doivent être installés avant de lancer le conteneur:
    
    ```bash
  
     code jazzy-ros-ynov/
  
    ```
    

  2.Lorsque VSCode s'ouvre, faites confiance aux sources et acceptez l'installation de l'extension Dev Container.

  3.Pour construire l'espace de travail, utilisez:
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

 
### 2.Lancer les differents Noeuds
  
1.dans un terminal, Démarrer la simulation Gazebo du robot Tiago :

      ```bash
      
      ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=pick_and_place
    
      ```
2.Lancement du code de détection de couleur

  Il existe 2 façons d'utiliser le code de détection. L'un avec "teleokey" et l'autre avec le "goto".
  Nous allons commencer par lancer le code de détection de couleur avec le "teleokey". Pour ce faire, on doit :


**2.1.Lancement du code de détection de couleur avec "teleopkey"**

2.1.1 dans un nouveau terminal, Lancer le noeud pour le traitement d' image :

    ```bash
  
        ros2 run pick_and_place detection_red
    ```

2.1.2 dans un nouveau terminal, démarrer la téléopération à partir du clavier:

    ```bash
  
        ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/key_vel
    ```
Maintenant vous pouvez deplacer le robot vers les canettes et constater que le robot se centre par rapport a la canette rouge detecté.


**2.2.Lancement du code de détection de couleur avec "goto"**

2.2.1.Dans un nouveau terminal, Commencez la navigation en 2D en chargeant votre carte our_map, en utilisant :

    **pour permettre au robot de se localiser sur la map**
    
  Dans RViz, utilisez les flèches pour contrôler la navigation : (NB : il est important de savoir que a ce niveau vouz devriez deja avoir la Map de l'environement)

  Estimer la pose 2D pour initialiser des particules aléatoires à une position approximative autour de la position réelle du robot
    
    ```bash 
        ros2 launch tiago_2dnav tiago_nav_bringup.launch.py is_public_sim:=True world_name:=our_map
    ```



2.2.2.Dans un nouveau terminal, Lancer le noeud pour le traitement d' image : 

    ```bash
  
      ros2 run pick_and_place detection_red
    ```
  
2.2.3.Dans un nouveau terminal, Lancer le noeud pour la navigation: 

    ```bash
  
      ros2 run pick_and_place goto
    ```

  Ici vous pouvez voir le robot s'approcher des canette tout en se centrant par rapport a la canette.

  
3.Lancer le noeud pour le pick and place :

  **code à paufiner**
    ```bash
  
      ros2 run pick_and_place pick
  
    ```

  
## Références et bibliographie
Ce projet s’appuie sur les sources suivantes :

Documentation : https://docs.pal-robotics.com/sdk-dev/navigation

Documentation : https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html

Documentation : https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html

Documentation : https://gitlab.com/f2m2robserv/jazzy-ros-ynov/-/tree/main#ros2-jazzy--robots-devcontainer

Documentation : https://gitlab.com/ymollard/humble-ros-cremi/-/tree/main/snippets

site: https://www.youtube.com/watch?v=g2xPdt3lVrw

site: https://www.youtube.com/watch?v=K0EZKG1-fkw

site: https://www.youtube.com/watch?v=aFNDh5k3SjU

cours: https://robservynov.netlify.app/












