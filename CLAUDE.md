# Contexte du projet (lu automatiquement par Claude Code)

BE Véhicule Autonome ENSEM, groupe 1. Robot AgileX LIMO n° 201 (Jetson Nano, Ubuntu 20.04, ROS 2 Foxy).
Notre code : paquet ROS 2 `be_limo/` (planification, contrôle, détection, supervision). Voir `be_limo/README.md`.
Le reste du dépôt (`limo/`, `environment/`, `room207-limo/`, ...) est une copie des dépôts du labo : ne pas le modifier.

## Les trois machines (regarder l'invite avant chaque commande)

| Invite | Machine | Usage |
|---|---|---|
| `e70348u@bi-fdd-f-l001` | PC de l'école, Ubuntu récent **sans ROS** | git, docker, ssh |
| `root@bi-fdd-f-l001` | conteneur Docker `foxy` sur le PC (ROS 2 Foxy) | simulation Gazebo, RViz, supervision |
| `jetson@nano` | le robot, `ssh jetson@100.75.155.201` | vrai robot |

- Le workspace `~/ros2_ws` du PC est monté dans le conteneur en `/root/ros2_ws`.
- Commande dans le conteneur depuis le PC : `docker exec foxy bash -lc "source /opt/ros/foxy/setup.bash && source /root/ros2_ws/install/setup.bash && <commande>"`.
  Si le conteneur est arrêté : `xhost +local:docker && docker start foxy`.
- Commande sur le robot depuis le PC : `ssh jetson@100.75.155.201 "bash -lc 'source ~/ros2_ws/install/setup.bash && <commande>'"`.
- Sur le robot, `be_limo` est une copie (pas un clone git) dans `~/ros2_ws/src/be_limo`. Pour le mettre à jour :
  `scp -r ~/ros2_ws/src/be-limo/be_limo jetson@100.75.155.201:/home/jetson/ros2_ws/src/`
  puis sur le robot `cd ~/ros2_ws && colcon build --symlink-install --packages-select be_limo`.

## Pièges déjà rencontrés

- **Ne jamais lancer deux drivers** (`limo_start` ou `real_l2.launch.py` avec `driver:=true`) en même temps : `Check sum failed` en boucle.
  Avant de relancer : `pkill -f limo_base; pkill -f ydlidar; pkill -f astra`.
- Après un redémarrage du robot, le port du châssis revient en `root tty` : `sudo chgrp dialout /dev/ttyTHS1 && sudo chmod 660 /dev/ttyTHS1`.
- Un terminal ouvert avant un `colcon build` ne voit pas les nouveaux paquets : `source ~/ros2_ws/install/setup.bash`.
- Sur le Jetson, la CLI `ros2` met plus de 3 s à démarrer, et `ros2 topic pub --once` perd souvent le message.
  Utiliser `timeout 10 ros2 topic pub -r 2 /supervisor/command std_msgs/String "{data: '{\"cmd\": \"start\"}'}"` (ou `estop`, `reset`).
- Gazebo : la fenêtre (gzclient) est trop lente sur ce PC (pas de carte graphique dans Docker). Lancer sans (`gui:=false`, défaut).
- Ne pas faire `apt upgrade` sur le Jetson. Ne jamais écrire de mot de passe ou de token dans le dépôt.

## Lancements utiles

- Simulation (conteneur) : `ros2 launch be_limo sim_l2.launch.py`
- Robot réel (robot) : `ros2 launch be_limo real_l2.launch.py initial_node:=2 mission:=demo use_camera:=true`
- Cartographie de la salle : `ros2 launch be_limo mapping.launch.py start_node:=2 map:=~/maps/salle207.yaml`
- Localisation lidar : `... real_l2.launch.py localization:=lidar map:=~/maps/salle207.yaml initial_node:=<n>`
- Tests sans ROS (n'importe où avec Python 3, numpy, OpenCV, pytest) : `cd be_limo && python3 -m pytest -q test/`

## Sécurité

Le robot roule pour de vrai : avant toute commande qui le fait bouger (`start`, téléopération, `cmd_vel`),
demander confirmation à l'utilisateur et vérifier que la piste est dégagée. Vitesse ≤ 0,15 m/s pour les premiers essais.

## État au 06/11/2026

- Simulation et tests hors ligne : OK.
- Robot réel : driver, lidar et caméra OK, détection de voie OK. Premier essai du parcours : le robot a bougé mais
  n'a pas suivi la route (cause pas encore identifiée : à diagnostiquer avec `/robot_pose` et `/controller/state`).
- Localisation lidar (AMCL) : écrite, pas encore essayée (carte de la salle à faire).
