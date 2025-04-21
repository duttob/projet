# Projet Robotics

 Nous avons un nœud qui contrôle le robot ICREATE 3 et il peut être mit dans 2 modes: mode manuel ou autonome.
 Une interface graphique (GUI) est également fournie pour permettre une interaction utilisateur intuitive.

## Fonctionnalités

- **Mode manuel** : Contrôle direct du robot via des boutons directionnels.
- **Mode autonome** : Le robot suit une logique de machine à états (FSM) pour effectuer des tâches comme l'exploration, éviter des obstacles, le docking et l'undocking.
- **Interface graphique** : Une interface utilisateur basée sur `tkinter` permet de basculer entre les modes, de contrôler manuellement le robot, et de surveiller son statut.
- **Gestion des capteurs** : Abonnements aux topics ROS2 pour détecter les obstacles.
- **Actions ROS2** : Utilisations des actions de docking et undocking.

## Dépendances

Ce projet utilise les bibliothèques suivantes :

- ROS2 (rclpy)
- `geometry_msgs` pour les messages de type `Twist`
- `irobot_create_msgs` pour les messages et actions spécifiques au robot ICREATE 3
- `tkinter` pour l'interface graphique

## Exécution

# Pour exécuter le projet:

```bash
colcon build
````
```bash	
source install/setup.bash
```

```bash
ros2 run projet robot_control_node
```