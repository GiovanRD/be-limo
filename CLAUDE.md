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
- Le PC et le robot sont sur deux sous-réseaux : la découverte DDS ne passe pas, le conteneur ne voit aucun topic du robot.
  Pour voir/piloter le robot depuis le PC : téléopération web + tunnel SSH (voir plus bas).
- Foxy : `ros2 topic echo --once` n'existe pas. Lire un message avec `timeout 8 ros2 topic echo /x | head -30`.
- Envoyer une commande au robot depuis un script : un petit publieur rclpy est plus fiable que `ros2 topic pub` (démarrage > 3 s).
- `pkill -f <motif>` lancé par SSH tue aussi la session SSH si le motif apparaît dans la commande SSH elle-même :
  mettre les `pkill` dans un script copié sur le robot (`scp`), puis l'exécuter.
- `local_controller` : `speed` de `config/be_limo.yaml` (0,25) l'emporte sur `speed:=` du launch. Vitesse effective :
  `ros2 param get /local_controller speed`. La fixer sans bouger : commande `{"cmd": "pause", "speed": 0.15}`.
- Sur ce robot, le gyroscope est sur `/limo/imu` (100 Hz, pas `/imu`), `/odom` à 50 Hz. Caméra : `/rgb/image_raw` (640×480 rgb8).
- Jetson Nano chargé : avec le pilote web + vision, ~50 % de CPU libre. Chaque abonnement Python à 100 Hz coûte ~30 % d'un cœur ;
  ne pas lire de paramètre ROS ni l'horloge ROS dans un callback rapide. `ros2 topic hz` ne répond pas toujours : compter avec un script rclpy.
- Gazebo dans le conteneur : la caméra simulée ne publie rien si `DISPLAY` est faux (sans message d'erreur).
  L'écran du PC est `:1` : `xhost +local:docker` puis lancer avec `DISPLAY=:1`.
- Relancer sur le robot : nos `launch` laissent parfois des noeuds orphelins (doublons, charge ×2). `~/diag/start_teleop.sh` tue tout avant.
- `scp -r be_limo` échoue sur les `__pycache__` créés par root dans le conteneur : copier avec
  `tar --exclude=__pycache__ -czf - be_limo | ssh jetson@100.75.155.201 "tar -xzf - -C ~/ros2_ws/src"`.

## Lancements utiles

- Simulation (conteneur) : `ros2 launch be_limo sim_l2.launch.py`
- Robot réel (robot) : `ros2 launch be_limo real_l2.launch.py initial_node:=2 mission:=demo use_camera:=true`
- Cartographie de la salle : `ros2 launch be_limo mapping.launch.py start_node:=2 map:=~/maps/salle207.yaml`
- Localisation lidar : `... real_l2.launch.py localization:=lidar map:=~/maps/salle207.yaml initial_node:=<n>`
- **Pilote web** (modes manuel / route caméra / suivi de personne, arrêt obstacles, carte, caméra) :
  robot `ros2 launch be_limo teleop.launch.py [initial_node:=N] [localization:=lidar map:=...]`
  (ou `bash ~/diag/start_teleop.sh` qui arrête d'abord tout), PC hors conteneur
  `ssh -N -L 8080:localhost:8080 jetson@100.75.155.201`, puis http://localhost:8080 (README section 10).
- Tests sans ROS (n'importe où avec Python 3, numpy, OpenCV, pytest) : `cd be_limo && python3 -m pytest -q test/`

## Sécurité

Le robot roule pour de vrai : avant toute commande qui le fait bouger (`start`, téléopération, `cmd_vel`),
demander confirmation à l'utilisateur et vérifier que la piste est dégagée. Vitesse ≤ 0,15 m/s pour les premiers essais.

## État au 06/11/2026

- Simulation et tests hors ligne : OK.
- Robot réel : driver, lidar et caméra OK, détection de voie OK. Premier essai du parcours : le robot a bougé mais
  n'a pas suivi la route. Diagnostic (roues en l'air) : les roues braquent, mais **l'odométrie ne voit pas les virages**
  (`/odom` ω = 0 alors que `/cmd_vel` ω = 0,12). Le châssis annonce le mode 2 (Mecanum) alors que le driver commande
  en Ackermann (`motion_mode: 1` forcé dans `limo_start`) ; le driver calcule l'odométrie dans le mode du châssis.
  Piste de correction : cap à partir du gyroscope `/limo/imu` dans `localization_node`. Braquage saccadé : le driver
  remet les roues droites à chaque `v = 0` (arrêts `STOP_OBSTACLE` répétés).
- Odométrie corrigée `odom_imu` (cap du gyroscope) : écrite, lancée avec le driver, testée hors ligne ; pas encore vérifiée en roulant.
- Pilote web (manuel / route / suivi + arrêt obstacles + carte) : déployé sur le robot (tout démarre, rien n'a roulé).
  Mode route testé dans Gazebo (voie suivie à ~5 cm, arrêt à 0,35 m de l'obstacle). Mode suivi testé hors ligne seulement.
  À faire au sol : vérifier la détection de voie (robot désarmé sur la piste, calibrer camera_height/pitch), puis essais à 0,10 m/s.
- Localisation lidar (AMCL) : écrite, pas encore essayée (carte de la salle à faire).
