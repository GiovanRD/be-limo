# BE Véhicule Autonome — AgileX LIMO

Groupe 1 (BE 2026) : Crystal RIBU, Morgan HOFFMANN, Giovan ROCCA-DURINGER
**LIMO n° 201 — IP 100.75.155.201 — poste BI-FDD-F-W253**

Ce guide se suit **dans l'ordre**. Chaque étape a le même format :

> 📍 **Où** : sur quelle machine et dans quel terminal
> ⌨️ **Tape ça** : la commande à copier
> ✅ **Tu dois voir** : le résultat attendu
> ❌ **Sinon** : quoi faire si ce n'est pas le cas

Deux machines entrent en jeu :
- **PC** : l'ordinateur de la salle (ou ton portable).
- **ROBOT** : le Jetson Nano dans la LIMO, qu'on pilote depuis le PC via SSH.

---

## Sommaire

0. [Le projet en bref](#0-le-projet-en-bref)
1. [Vérifier le PC](#1-vérifier-le-pc)
2. [Se connecter au robot](#2-se-connecter-au-robot)
3. [Démarrer le robot](#3-démarrer-le-robot)
4. [Vérifier les capteurs](#4-vérifier-les-capteurs)
5. [Faire bouger le robot](#5-faire-bouger-le-robot)
6. [Simulation salle 207](#6-simulation-salle-207)
7. [Enregistrer un essai](#7-enregistrer-un-essai)
8. [Comprendre la loi de commande](#8-comprendre-la-loi-de-commande)
9. [Écrire son propre contrôleur](#9-écrire-son-propre-contrôleur)
10. [Problèmes connus](#10-problèmes-connus)
11. [Documents du projet](#11-documents-du-projet)

---

## 0. Le projet en bref

**But** : rendre la LIMO autonome sur la piste de la salle 207.

**Architecture** (Rendu 1, `rendu_1/Diagramme_Projet_Limo.docx`) : chaque bloc est un nœud ROS 2, les flèches sont des topics.

```text
CAPTEURS      limo_base, lidar, caméra    ──►  /odom  /scan  /camera/image_raw
PERCEPTION    localisation, vision        ──►  /robot_pose  /lane_detection  /obstacles
PLANIFICATION global_planner              ──►  /plan  (à partir de /goal_pose)
CONTRÔLE      local_controller            ──►  /cmd_vel  ──►  moteurs du robot
SUPERVISION   interface graphique (état, événements, objectifs)
```

**Planning** (`rendu_1/Planificateur de Projet LIMO.xlsx`) :

| Activité | Qui | Semaines |
|---|---|---|
| A0 Lancement, répartition | Crystal, Giovan | 1–4 |
| A1 Prise en main LIMO + ROS 2 | Crystal, Morgan | 1–4 |
| A2 Planification et contrôle | Crystal, Giovan | 6–10 |
| A3 Détection et perception | Giovan, Morgan | 6–10 |
| A4 Interface de supervision | Tous | 15–17 |
| A5 Intégration, soutenance | Tous | 18 |

---

## 1. Vérifier le PC

Le robot et la simulation utilisent **Ubuntu 20.04 + ROS 2 Foxy**. Ne mélange pas avec Humble.

### 1.1 ROS 2 est-il chargé ?

📍 PC, un terminal
⌨️
```bash
echo $ROS_DISTRO
```
✅ `foxy`
❌ Rien ne s'affiche : charge ROS, puis rends-le automatique :
```bash
source /opt/ros/foxy/setup.bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```
❌ `bash: /opt/ros/foxy/setup.bash: No such file` : ROS 2 Foxy n'est pas installé. Suis `limo/configuration/native_ubuntu20.04_foxy/README.md` (Ubuntu) ou `limo/configuration/wsl2_ubuntu20.04_foxy/README.md` (Windows).

### 1.2 Le workspace est-il compilé ?

📍 PC
⌨️
```bash
source ~/ros2_ws/install/setup.bash
ros2 pkg list | grep -E "limo|room207|teleoperation|recording"
```
✅ Une liste qui contient au moins `limo_base`, `limo_car`, `limo_description`, `room207_limo`, `teleoperation`, `recording`
❌ `No such file` ou liste vide : le workspace n'est pas compilé. Mets les dépôts dans `~/ros2_ws/src` (voir `limo/configuration/native_ubuntu20.04_foxy/README.md`), puis :
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```
✅ La compilation finit par `Summary: N packages finished`
❌ `Failed` sur un paquet : lis le message juste au-dessus. Le plus souvent une dépendance manque → relance `rosdep install ...`.

> 💡 Dans **chaque nouveau terminal**, refais `source ~/ros2_ws/install/setup.bash` (ou ajoute la ligne à `~/.bashrc`).

---

## 2. Se connecter au robot

### 2.1 Le réseau

- La LIMO doit être sur le Wi-Fi **ENSEMSAMI**.
- Le PC doit être sur le **réseau filaire ENSEM** ou sur **eduroam**.
- Mots de passe : voir `limo/configuration/limo_ubuntu20.04_foxy/README.md` (ne les mets pas dans le code).

📍 PC
⌨️
```bash
ping -c 3 100.75.155.201
```
✅ `3 packets transmitted, 3 received`
❌ `100% packet loss` ou `Destination Host Unreachable` :
1. Le robot est-il allumé ? Attends 1 à 2 minutes après l'allumage.
2. Le robot est-il sur ENSEMSAMI ? Branche un écran sur la LIMO pour vérifier.
3. Le PC est-il sur eduroam ou le filaire ENSEM (pas un partage de connexion) ?

### 2.2 SSH

📍 PC
⌨️
```bash
ssh jetson@100.75.155.201
```
✅ On te demande le mot de passe, puis l'invite devient `jetson@...:~$`
❌ `WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED` (le robot a été réinstallé) :
```bash
ssh-keygen -R 100.75.155.201
```
puis relance `ssh`.
❌ `Connection refused` / `timed out` : retour à l'étape 2.1.

📍 ROBOT (dans la session SSH)
⌨️
```bash
whoami
```
✅ `jetson`. Tu es bien sur le robot.

### 2.3 Même « canal » ROS sur les deux machines

📍 PC **et** ROBOT
⌨️
```bash
echo $ROS_DOMAIN_ID
```
✅ La même valeur des deux côtés (vide = 0, c'est OK si vide des deux côtés)
❌ Valeurs différentes : les deux machines ne se voient pas. Mets la même :
```bash
export ROS_DOMAIN_ID=0
```

---

## 3. Démarrer le robot

📍 ROBOT, **terminal SSH n°1** (il reste ouvert tout le temps)
⌨️
```bash
cd ~/ros2_ws
ros2 launch limo_start limo_real.launch.py
```
✅ Des lignes `[INFO]` défilent, sans `[ERROR]`, et le terminal reste occupé
❌ Erreur sur `/dev/ttyTHS1` (port du châssis) : vérifie les droits, voir 4.3.
❌ Erreur caméra / `OpenNI2` : lance sans la caméra :
```bash
ros2 launch limo_start limo_real.launch.py use_camera:=false
```
❌ `Package 'limo_start' not found` : le workspace du robot n'est pas compilé. Voir `limo/configuration/limo_ubuntu20.04_foxy/README.md` (`build-robot.sh`).

> Pour arrêter le robot : **Ctrl+C** dans ce terminal.

---

## 4. Vérifier les capteurs

Ouvre un **deuxième terminal SSH** sur le robot (`ssh jetson@100.75.155.201` dans un nouvel onglet). Le premier doit continuer à tourner.

### 4.1 Les topics existent-ils ?

📍 ROBOT, terminal SSH n°2
⌨️
```bash
ros2 topic list
```
✅ La liste contient au moins `/cmd_vel`, `/odom`, `/scan`, `/limo/imu` (et `/rgb/image_raw` si caméra)
❌ Liste presque vide : le terminal n°1 a planté, regarde son message d'erreur.

### 4.2 Les capteurs publient-ils ?

📍 ROBOT, terminal SSH n°2. Fais-les un par un, **Ctrl+C** entre chaque.
⌨️
```bash
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic hz /limo/imu
ros2 topic hz /rgb/image_raw
```
✅ Une ligne `average rate: ...` qui se répète toutes les secondes
❌ Rien ne s'affiche :
- `/scan` → le lidar, voir 4.3
- `/rgb/image_raw` → la caméra ; relance le robot avec `use_camera:=false` si tu n'en as pas besoin
- `/odom` → le châssis, voir 4.3

### 4.3 Les ports matériels

📍 ROBOT
⌨️
```bash
ls -l /dev/ttyTHS1 /dev/ydlidar
```
✅ Pour `/dev/ttyTHS1` : `crw-rw---- 1 root dialout ...` ; `/dev/ydlidar` existe
❌ Droits différents sur `/dev/ttyTHS1` :
```bash
sudo chgrp dialout /dev/ttyTHS1
sudo chmod 660 /dev/ttyTHS1
```
❌ `/dev/ydlidar: No such file` : débranche/rebranche le lidar et regarde `ls /dev/ttyUSB*`. Voir la section *Troubleshooting* de `limo/configuration/limo_ubuntu20.04_foxy/README.md`.

### 4.4 Voir ce que dit un capteur

⌨️
```bash
ros2 topic echo /odom
```
✅ Des blocs `pose: position: x: ... y: ...` qui défilent

---

## 5. Faire bouger le robot

> ⚠️ **Avant** : robot sur une zone dégagée **ou roues dans le vide**. Une personne garde la main près de l'interrupteur. L'arrêt logiciel n'est pas un arrêt d'urgence.

### 5.1 Une seule commande, pour tester

📍 ROBOT, terminal SSH n°2
⌨️
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```
✅ Le robot avance brièvement puis s'arrête
❌ Rien ne bouge : le terminal n°1 tourne-t-il ? Vérifie avec `ros2 topic info /cmd_vel` : il doit y avoir `Subscription count: 1`.

### 5.2 Au clavier (dans le terminal)

📍 ROBOT, terminal SSH n°2
⌨️
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
✅ Un panneau de touches s'affiche (`i` avance, `,` recule, `j`/`l` tourne, `k` stop). Le robot réagit.
❌ `Package not found` : `sudo apt install ros-foxy-teleop-twist-keyboard`

### 5.3 Avec l'interface graphique ZQSD (depuis le PC)

Il faut **4 terminaux** :

| # | Où | Tape ça |
|---|---|---|
| 1 | ROBOT | `ros2 launch limo_start limo_real.launch.py` |
| 2 | ROBOT | `ros2 run robot_communication tcp_server` |
| 3 | PC | `ssh -N -L 8765:127.0.0.1:8765 jetson@100.75.155.201` |
| 4 | PC | `ros2 run teleoperation teleop_limo_real` |

✅ Une fenêtre s'ouvre ; Z avant, S arrière, Q gauche, D droite, Espace stop, Échap quitter
❌ Terminal 2 : `Package 'robot_communication' not found` : ce paquet n'est **pas** dans ce dépôt, il faut le récupérer sur le GitLab Robotic lab.
❌ Terminal 3 ne rend pas la main : **c'est normal**, il doit rester ouvert.
❌ Terminal 4 : `Connection refused` : le terminal 2 ou 3 n'est pas lancé.

---

## 6. Simulation salle 207

Pour tester sans le robot. Tout se passe sur le **PC**.

📍 PC, terminal n°1
⌨️
```bash
ros2 launch room207_limo room207_limo.launch.py
```
✅ Gazebo s'ouvre avec la piste et la LIMO ; RViz s'ouvre aussi (le premier lancement peut prendre 1 minute)
❌ Fenêtre noire ou Gazebo plante sous WSL : vérifie `wsl --version` dans PowerShell (WSLg doit apparaître).
❌ `Package 'room207_limo' not found` : retour à l'étape 1.2.

📍 PC, terminal n°2
⌨️
```bash
ros2 run teleoperation teleop_limo_simu --ros-args -r /cmd_vel:=/cmd_vel_gazebo
```
✅ Une fenêtre ZQSD s'ouvre ; clique dedans, le robot bouge dans Gazebo
❌ Le robot ne bouge pas : la fenêtre ZQSD doit avoir le focus (clique dessus). Vérifie `ros2 topic info /cmd_vel_gazebo` : `Publisher count: 1`, `Subscription count: 1`. Voir aussi [problème n°2](#10-problèmes-connus).

Autres mondes, si besoin :
```bash
ros2 launch ensem_room207 ensem_room207.launch.py       # salle 207 complète
ros2 launch agilex_sand_table agilex_sand_table.launch.py
ros2 launch limo_car ackermann_gazebo.launch.py         # LIMO seule, sol vide
```

---

## 7. Enregistrer un essai

📍 PC (simu) ou ROBOT (réel), terminal n°1
⌨️
```bash
ros2 run recording recorder --ros-args -p topics:="['/odom', '/scan', '/cmd_vel']"
```
✅ Le nœud attend, sans erreur

📍 Même machine, terminal n°2
⌨️
```bash
ros2 run recording recording_widget
```
✅ Boutons **Start recording** / **Stop recording**. Après Stop, un dossier `~/ros2_ws/data/rosbag_<date>` apparaît.

Pour vérifier et tracer :
```bash
ros2 bag info ~/ros2_ws/data/rosbag_<date>
ros2 run room207_limo plot_odometry ~/ros2_ws/data/rosbag_<date>
```
✅ `bag info` montre la durée et le nombre de messages ; `plot_odometry` crée des PDF (trajectoire, vitesses, cap) dans `~/ros2_ws/data/`
❌ `Count: 0` sur un topic : il ne publiait pas pendant l'essai (le nom est peut-être différent, ex. IMU = `/limo/imu` sur le robot, `/imu_plugin/out` en simu).

---

## 8. Comprendre la loi de commande

### 8.1 Comment on commande la LIMO

On publie un message sur **`/cmd_vel`** avec deux nombres :
- `linear.x` = **v**, la vitesse d'avance en m/s
- `angular.z` = **ω**, la vitesse de rotation en rad/s (positif = tourne à gauche)

Le robot convertit lui-même (v, ω) en angle de braquage des roues avant.

### 8.2 Le modèle de la voiture

La LIMO est une voiture (Ackermann) : elle ne peut pas tourner sur place. Modèle « bicyclette » :

```text
Ẋ = v·cos ψ          (X, Y) = position,  ψ = cap
Ẏ = v·sin ψ          δ      = angle de braquage
ψ̇ = v·tan δ / L      L      = empattement = 0,24 m
```

Passer de l'un à l'autre :
```text
ω = v·tan δ / L          δ = atan(L·ω / v)
```

| Valeur utile | |
|---|---|
| Empattement L | 0,24 m |
| Voie avant | 0,168 m |
| Rayon de roue | 0,045 m |
| Braquage max | ±30° (en pratique ~25°) |
| Rayon de virage mini | ≈ 0,55 m |
| Roues motrices | arrière seulement |

### 8.3 Deux niveaux de commande

1. **Bas niveau (déjà fait)** : garder la vitesse et l'angle demandés. C'est le châssis sur le vrai robot, le plugin Gazebo en simu, et `limo/matlab/+limo/controller.m` en MATLAB (PID vitesse Kp=15, Ki=2, Kd=0,1 ; PID braquage Kp=5, Ki=1, Kd=0,1 ; 100 Hz).
2. **Haut niveau (à faire par le groupe)** : choisir v et ω pour **suivre une trajectoire**. C'est le `local_controller` de votre architecture.

### 8.4 La méthode proposée : Pure Pursuit

Idée : on vise un point de la trajectoire situé à une distance **Ld** devant le robot, et on braque pour l'atteindre en arc de cercle.

```text
α = angle entre l'axe du robot et le point visé
δ = atan( 2·L·sin α / Ld )
ω = v·tan δ / L
```

Réglage : **Ld petit** = suit précisément mais oscille ; **Ld grand** = stable mais coupe les virages. Commencer avec Ld = 0,4 m et v = 0,3 m/s.

Alternative : **Stanley**, `δ = erreur_de_cap + atan(k · erreur_latérale / v)`.

---

## 9. Écrire son propre contrôleur

### 9.1 Créer le paquet

📍 PC
⌨️
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python suivi_trajectoire --dependencies rclpy geometry_msgs nav_msgs
```
✅ Un dossier `suivi_trajectoire/` est créé

### 9.2 Le code

Crée `~/ros2_ws/src/suivi_trajectoire/suivi_trajectoire/pure_pursuit.py` :

```python
import math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

L, V, LD = 0.24, 0.3, 0.4                       # empattement, vitesse, distance de visée
PATH = [(0.5 * i, 0.0) for i in range(20)]      # trajectoire test : ligne droite (à remplacer par /plan)

class PurePursuit(Node):
    def __init__(self):
        super().__init__('local_controller')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
        target = next((pt for pt in PATH if math.hypot(pt[0] - p.x, pt[1] - p.y) >= LD), PATH[-1])
        alpha = math.atan2(target[1] - p.y, target[0] - p.x) - yaw
        delta = max(-0.4, min(0.4, math.atan2(2 * L * math.sin(alpha), LD)))
        cmd = Twist()
        cmd.linear.x = V if math.hypot(PATH[-1][0] - p.x, PATH[-1][1] - p.y) > 0.1 else 0.0
        cmd.angular.z = cmd.linear.x * math.tan(delta) / L
        self.pub.publish(cmd)

def main():
    rclpy.init(); rclpy.spin(PurePursuit())
```

Dans `setup.py` du paquet, ajoute dans `console_scripts` :
```python
'pure_pursuit = suivi_trajectoire.pure_pursuit:main',
```

### 9.3 Compiler et lancer

📍 PC
⌨️
```bash
cd ~/ros2_ws
colcon build --packages-select suivi_trajectoire --symlink-install
source install/setup.bash
ros2 run suivi_trajectoire pure_pursuit
```
✅ Pas d'erreur ; `ros2 topic echo /cmd_vel` (autre terminal) montre des commandes ; le robot (simu) avance puis s'arrête au bout de la ligne
❌ `No executable found` : la ligne `console_scripts` manque dans `setup.py`, puis recompile.

### 9.4 Ordre de validation

1. **MATLAB** (sans ROS) : `addpath('limo/matlab'); run_limo_gazebo`. Le bloc `LIMO (Gazebo)` de `limo_lib.slx` accepte une entrée `cmd_vel`.
2. **Gazebo** : même nœud, avec l'adaptateur lancé dans un autre terminal (`ros2 run limo_car gazebo_twist_adapter.py`, voir problème n°2).
3. **Robot réel** : vitesse ≤ 0,3 m/s, zone dégagée, quelqu'un à l'interrupteur.
4. Enregistrer chaque essai (étape 7) pour les courbes du rapport.

---

## 10. Problèmes connus

Repérés dans les sources fournies par le labo :

| # | Problème | Solution |
|---|---|---|
| 1 | `ros2 launch room207_limo demo.launch.py` plante : il appelle `teleop_limo`, qui n'existe pas | Dans `room207-limo/launch/demo.launch.py`, remplacer `teleop_limo` par `teleop_limo_simu` |
| 2 | En simu salle 207, Gazebo écoute **`/cmd_vel_gazebo`** (pas `/cmd_vel`) et y attend un **angle de braquage** (pas ω) | Téléop : ajouter `--ros-args -r /cmd_vel:=/cmd_vel_gazebo` (étape 6). Votre contrôleur (qui publie ω sur `/cmd_vel`) : lancer en plus l'adaptateur `ros2 run limo_car gazebo_twist_adapter.py`, qui convertit `/cmd_vel` → `/cmd_vel_gazebo`. Ne pas lancer la téléop et l'adaptateur en même temps. |
| 3 | `robot_communication` absent | Le récupérer sur le GitLab Robotic lab |
| 4 | Le monde chargé est `qcar_large_track`, pas `ensem_room207` comme annoncé | Changer le monde dans `room207_limo.launch.py` si besoin |
| 5 | IMU : nom différent | `/limo/imu` sur le robot, `/imu_plugin/out` en simu |
| 6 | En simu, `/odom` est parfait | Le vrai robot dérivera : prévoir de la marge |
| 7 | `sudo apt upgrade` sur le robot | **Ne pas le faire** : casse les pilotes NVIDIA |
| 8 | PC de salle effacés | `git push` avant de partir |

---

## 11. Documents du projet

| Document | Où |
|---|---|
| **Fiche groupe à compléter** (rôles, problèmes) | `group-informations/README.md` |
| Rendu 1 : diagramme + planning | `rendu_1/` |
| Règles du GitLab Robotic lab | `documentations/README.md` |
| Installation du robot | `limo/configuration/limo_ubuntu20.04_foxy/README.md` |
| Installation du PC (Ubuntu / Windows) | `limo/configuration/native_ubuntu20.04_foxy/README.md`, `limo/configuration/wsl2_ubuntu20.04_foxy/README.md` |
| Réinstaller le Jetson | `documentations/limo1_ubuntu20.04_foxy/README.md` |
| Simulation | `room207-limo/README.md`, `environment/README.md` |
| Téléopération, enregistrement | `teleoperation/README.md`, `recording/README.md` |
| Modèle MATLAB, paramètres physiques | `limo/matlab/README.md`, `limo/matlab/+limo/parameters.m` |
| Conversion `/cmd_vel` du robot | `limo/ros2_foxy/limo_ros2/limo_base/src/limo_driver.cpp` |
| Docs en ligne | [ROS 2 Foxy](https://docs.ros.org/en/foxy/) · [limo_ros2 AgileX](https://github.com/agilexrobotics/limo_ros2) · [Gazebo Classic](https://classic.gazebosim.org/tutorials) |

**À produire par le groupe** : fiche groupe complète, rapport, documentation technique, présentation, enregistrements (rosbags) et courbes.

---

## Contenu du dépôt

| Dossier | Contenu |
|---|---|
| `limo/` | Logiciel LIMO (ROS 2 Foxy/Humble, MATLAB, configuration) |
| `room207-limo/` | Simulation salle 207 |
| `environment/` | Mondes Gazebo |
| `teleoperation/` | Téléopération clavier |
| `recording/` | Enregistrement rosbag |
| `documentations/` | Organisation et guides d'installation |
| `group-informations/` | Fiche du groupe |
| `rendu_1/` | Rendu 1 |
