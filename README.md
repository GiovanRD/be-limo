# BE Véhicule Autonome — AgileX LIMO

Groupe 1 (BE 2026) : Crystal RIBU, Morgan HOFFMANN, Giovan ROCCA-DURINGER
LIMO n° **201** — IP **100.75.155.201** — poste **BI-FDD-F-W253**

Guide pratique pour se connecter à la LIMO, utiliser ROS 2 et implémenter la loi de commande. Les détails complets sont dans les README de chaque dossier (liens en fin de page).

---

## 1. Le projet en bref

Objectif : rendre la LIMO autonome sur la piste QCar de la salle 207 (suivi de trajectoire, perception, supervision).

Architecture retenue (Rendu 1, `rendu_1/Diagramme_Projet_Limo.docx`) :

```text
PROCÉDÉ            limo_base_node, lidar_node, camera_node  →  /odom  /scan  /camera/image_raw
PERCEPTION         localisation_node, vision_node          →  /robot_pose  /lane_detection  /obstacles  /detected_events
PLANIFICATION      global_planner                          ←  /goal_pose        →  /plan
CONTRÔLE           local_controller                        →  /cmd_vel  →  limo_base_node
SUPERVISION        interface (état, événements, objectifs)
```

Planning (`rendu_1/Planificateur de Projet LIMO.xlsx`) :

| Activité | Qui | Semaines |
|---|---|---|
| A0 Lancement, répartition | Crystal, Giovan | 1–4 |
| A1 Prise en main LIMO + archi ROS 2 | Crystal, Morgan | 1–4 |
| A2 Planification et contrôle | Crystal, Giovan | 6–10 |
| A3 Détection et perception | Giovan, Morgan | 6–10 |
| A4 Interface de supervision | Tous | 15–17 |
| A5 Intégration finale, soutenance | Tous | 18 |

---

## 2. Environnement de référence

| Cible | Système |
|---|---|
| LIMO réelle (Jetson Nano) | Ubuntu 20.04, **ROS 2 Foxy** |
| Poste de simulation | Ubuntu 20.04, ROS 2 Foxy, Gazebo Classic 11, RViz2 (natif, ou Windows + WSL2 + Docker) |
| MATLAB | R2026b + Simulink (ROS Toolbox seulement pour la liaison Gazebo) |

**Toujours Foxy.** Humble existe mais `room207-limo` n'est pas testé avec. Ne jamais mélanger les paquets Foxy et Humble dans un même workspace.

---

## 3. Installer le poste (simulation)

**Ubuntu 20.04 natif** (guide : `limo/configuration/native_ubuntu20.04_foxy/README.md`)

```bash
# ROS 2 Foxy Desktop installé au préalable
mkdir -p ~/robotic_lab && cd ~/robotic_lab
git clone https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/limo.git
cd limo
git sparse-checkout set configuration/native_ubuntu20.04_foxy ros2_foxy/limo_ros2 \
  ros2_foxy/ros2_astra_camera ros2_foxy/ydlidar_ros2_driver ros2_foxy/YDLidar-SDK
bash configuration/native_ubuntu20.04_foxy/setup.sh      # dépendances + build dans ~/ros2_ws
```

**Windows** : WSL2 + Docker Desktop, voir `limo/configuration/wsl2_ubuntu20.04_foxy/README.md` (`dev.sh build`, `dev.sh up`, `dev.sh build-workspace`, `dev.sh shell`).

Ajouter ensuite les dépôts communs dans `~/ros2_ws/src` : `environment`, `teleoperation`, `recording`, et **le dépôt du groupe** (fork de `room207-limo`), puis :

```bash
source /opt/ros/foxy/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash          # à refaire dans chaque nouveau terminal
```

---

## 4. Connexion à la LIMO

1. **Réseau** : LIMO sur le Wi-Fi **ENSEMSAMI**, PC sur le réseau filaire ENSEM ou sur **eduroam** (les deux réseaux sont pontés). Identifiants Wi-Fi et compte robot : voir `limo/configuration/limo_ubuntu20.04_foxy/README.md` (ne pas les recopier dans le code).
2. **SSH** :
   ```bash
   ssh jetson@100.75.155.201
   whoami && hostname               # doit afficher jetson
   ```
   Après une réinstallation du robot : `ssh-keygen -R 100.75.155.201`.
3. **Même `ROS_DOMAIN_ID`** sur le PC et le robot (0 par défaut) :
   ```bash
   echo $ROS_DOMAIN_ID
   ```

Première installation du logiciel sur le robot (déjà faite normalement) : `limo/configuration/limo_ubuntu20.04_foxy/README.md` (`install-deps.sh`, `setup-devices.sh`, `prepare-workspace.py`, `build-robot.sh`).

---

## 5. Démarrer le robot réel

Sur la LIMO (terminal SSH 1) :

```bash
cd ~/ros2_ws
ros2 launch limo_start limo_real.launch.py                  # ou l'alias : start
ros2 launch limo_start limo_real.launch.py use_camera:=false   # sans caméra
```

Vérifications (terminal SSH 2) :

```bash
ros2 topic list
ros2 topic hz /odom          # odométrie
ros2 topic hz /scan          # lidar
ros2 topic hz /limo/imu      # IMU
ros2 topic hz /rgb/image_raw # caméra
ls -l /dev/ttyTHS1 /dev/ydlidar   # port châssis + lidar
```

Premier test de mouvement, **roues dans le vide ou zone dégagée** :

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # publie /cmd_vel
ros2 topic echo /odom
```

### Téléopération graphique depuis le PC (ZQSD)

```bash
# Robot, terminal 1 : driver          → ros2 launch limo_start limo_real.launch.py
# Robot, terminal 2 : serveur TCP     → ros2 run robot_communication tcp_server
# PC, terminal 1    : tunnel SSH
ssh -N -L 8765:127.0.0.1:8765 jetson@100.75.155.201
# PC, terminal 2    : interface
ros2 run teleoperation teleop_limo_real
```

Z avant · S arrière · Q gauche · D droite · Espace stop · Échap quitter.
⚠️ L'arrêt logiciel n'est pas un arrêt d'urgence : garder la main sur l'interrupteur du robot.

---

## 6. Simulation (salle 207)

```bash
ros2 launch room207_limo room207_limo.launch.py    # Gazebo + LIMO + RViz
ros2 run teleoperation teleop_limo_simu            # second terminal, clavier ZQSD
```

Autres mondes : `ros2 launch ensem_room207 ensem_room207.launch.py`, `qcar_large_track`, `agilex_sand_table`.
LIMO seule sur sol vide : `ros2 launch limo_car ackermann_gazebo.launch.py`.

---

## 7. Mémo ROS 2

| Topic | Type | Rôle |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Commande : `linear.x` = v (m/s), `angular.z` = ω (rad/s) |
| `/cmd_vel_gazebo` | `geometry_msgs/Twist` | Commande du plugin Gazebo (voir §8.2) |
| `/odom` | `nav_msgs/Odometry` | Position, cap, vitesses |
| `/scan` | `sensor_msgs/LaserScan` | Lidar YDLidar |
| `/limo/imu` (réel), `/imu_plugin/out` (simu) | `sensor_msgs/Imu` | Centrale inertielle |
| `/rgb/image_raw`, `/depth_camera/depth/image_raw` | `sensor_msgs/Image` | Caméra Astra |

```bash
ros2 node list
ros2 topic list
ros2 topic echo /odom
ros2 topic hz /scan
ros2 interface show geometry_msgs/msg/Twist
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
ros2 run rqt_graph rqt_graph          # graphe des nœuds
ros2 pkg create --build-type ament_python mon_paquet --dependencies rclpy geometry_msgs nav_msgs
```

---

## 8. Loi de commande

### 8.1. Modèle (bicyclette, Ackermann)

```text
Ẋ = v cos ψ        Ẏ = v sin ψ        ψ̇ = (v / L) tan δ
```

| Paramètre | Valeur |
|---|---|
| Empattement L | 0,24 m |
| Voie (pivots) | 0,168 m |
| Rayon de roue | 0,045 m |
| Masse (Gazebo) | 7,05 kg |
| Braquage max | ±30° (roue intérieure en butée vers 25°) |
| Propulsion | roues arrière seulement |

Conversions : `ω = v tan δ / L`  ⇔  `δ = atan(L ω / v)`.

### 8.2. Ce que chaque cible attend

| Cible | Topic | `angular.z` signifie |
|---|---|---|
| Robot réel (mode Ackermann) | `/cmd_vel` | vitesse de lacet ω ; le driver calcule R = v/ω puis l'angle des roues |
| Gazebo via adaptateur (`limo_car ackermann.launch.py`) | `/cmd_vel` | ω ; l'adaptateur borne δ à 23,6° et coupe après 0,5 s sans commande |
| Gazebo direct (`room207_limo`, pas d'adaptateur) | `/cmd_vel_gazebo` | **angle de braquage** δ = atan(L ω / \|v\|) |

→ Écrire le contrôleur en `(v, ω)` sur `/cmd_vel`. En simulation salle 207, soit lancer l'adaptateur `gazebo_twist_adapter.py` (paquet `limo_car`), soit publier δ sur `/cmd_vel_gazebo`.

### 8.3. Niveau bas (déjà fait)

- **Robot réel** : asservissement vitesse/braquage interne au châssis.
- **Gazebo** : PID du plugin à 100 Hz (Kp braquage = 10 000, saturé à 20 N·m), ce qui produit une légère oscillation du braquage : comparer les essais sur des moyennes.
- **MATLAB** (`limo/matlab/+limo/controller.m`) : PID vitesse (Kp 15, Ki 2, Kd 0,1) + PID braquage par roue (Kp 5, Ki 1, Kd 0,1) avec correction d'Ackermann `δL = atan(L/(R−W/2))`, `δR = atan(L/(R+W/2))`, à 100 Hz.

### 8.4. Niveau haut : suivi de trajectoire (à implémenter)

**Pure pursuit**, simple et adapté à l'Ackermann. Point cible à distance `Ld` sur la trajectoire, angle `α` entre l'axe du robot et ce point :

```text
δ = atan( 2 L sin α / Ld )        ω = v tan δ / L        Ld ≈ 0,3 à 0,6 m (ou Ld = k·v)
```

Alternative : **Stanley** `δ = e_ψ + atan(k e_y / v)` (erreur de cap + erreur latérale).

Squelette de nœud :

```python
import math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

L, V, LD = 0.24, 0.3, 0.4
PATH = [(0.5 * i, 0.0) for i in range(20)]          # à remplacer par /plan

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

### 8.5. Valider avant le robot réel

1. MATLAB : `addpath('limo/matlab'); run_limo_gazebo` ; bloc `LIMO (Gazebo)` de `limo_lib.slx` en entrée `cmd_vel`.
2. Gazebo : même nœud, même topics.
3. Robot réel : vitesse réduite (≤ 0,3 m/s), zone dégagée.
4. Comparer MATLAB et Gazebo : `ros2 bag record /cmd_vel /odom /joint_states /limo/imu`, puis `compare_with_gazebo('dossier_du_bag')`.

---

## 9. Enregistrer et analyser

```bash
ros2 run recording recorder --ros-args -p topics:="['/odom', '/scan', '/limo/imu']"
ros2 run recording recording_widget         # ou start_recording / stop_recording
ros2 bag info ~/ros2_ws/data/rosbag_<date>
ros2 run room207_limo plot_odometry ~/ros2_ws/data/rosbag_<date>   # PDF trajectoire, vitesses, lacet
```

---

## 10. Pièges repérés dans les sources

- `room207-limo/launch/demo.launch.py` lance l'exécutable `teleop_limo`, qui n'existe pas : le paquet fournit `teleop_limo_simu` et `teleop_limo_real`. Corriger dans le fork du groupe.
- `room207_limo.launch.py` charge le monde `qcar_large_track.world`, alors que la doc annonce `ensem_room207` par défaut.
- La simu salle 207 ne lance pas l'adaptateur `/cmd_vel` → `/cmd_vel_gazebo` (voir §8.2).
- Le paquet `robot_communication` (serveur TCP pour la téléop réelle) n'est pas dans ce dépôt : à récupérer sur le GitLab Robotic lab.
- IMU : `/limo/imu` sur le robot, `/imu_plugin/out` en simulation.
- En simu, `/odom` est la vérité terrain Gazebo, pas une odométrie roues : le robot réel sera moins précis.
- Ne pas faire `apt upgrade` complet sur le Jetson (drivers NVIDIA).
- Les PC de salle ne sont pas sauvegardés : `git push` avant de partir.

---

## 11. Documents nécessaires

| Document | Où |
|---|---|
| Fiche groupe (rôles, IP, problèmes connus) **à compléter** | `group-informations/README.md` |
| Rendu 1 : diagramme d'architecture + planning | `rendu_1/` |
| Organisation du GitLab, règles de travail | `documentations/README.md` |
| Installation robot réel | `limo/configuration/limo_ubuntu20.04_foxy/README.md` |
| Installation poste Ubuntu / Windows | `limo/configuration/native_ubuntu20.04_foxy/README.md`, `.../wsl2_ubuntu20.04_foxy/README.md` |
| Réinstallation du Jetson | `documentations/limo1_ubuntu20.04_foxy/README.md` |
| Simulation salle 207 | `room207-limo/README.md`, `environment/README.md` |
| Téléopération, enregistrement | `teleoperation/README.md`, `recording/README.md` |
| Modèle MATLAB/Simulink, paramètres physiques | `limo/matlab/README.md`, `limo/matlab/+limo/parameters.m` |
| Driver robot (conversion `/cmd_vel`) | `limo/ros2_foxy/limo_ros2/limo_base/src/limo_driver.cpp` |
| Adaptateur Gazebo | `limo/ros2_foxy/limo_ros2/limo_car/scripts/gazebo_twist_adapter.py` |
| Docs externes | [ROS 2 Foxy](https://docs.ros.org/en/foxy/), [limo_ros2 AgileX](https://github.com/agilexrobotics/limo_ros2), [Gazebo Classic](https://classic.gazebosim.org/tutorials) |

À produire par le groupe : fiche groupe complétée, rapport, documentation technique, présentation de soutenance, rosbags et courbes de validation.

---

## Contenu du dépôt

| Dossier | Contenu |
|---|---|
| `limo/` | Projet principal Limo (ROS 2 Foxy/Humble, MATLAB, configuration, documentation) |
| `room207-limo/` | Simulation et démonstration salle 207 |
| `environment/` | Mondes Gazebo (salle 207, sand table, piste QCar) |
| `teleoperation/` | Téléopération clavier (simu et réel) |
| `recording/` | Enregistrement rosbag |
| `documentations/` | Organisation du groupe GitLab et guides d'installation |
| `group-informations/` | Fiche du groupe |
| `rendu_1/` | Rendu 1 : planificateur et diagramme du projet |
