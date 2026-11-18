# Rendre le livrable 2 sur le GitLab de l'université

**But** : mettre notre travail dans le dépôt GitLab du groupe (`Students/BE/BE-2026-Group-01`), puis déposer **le lien de ce dépôt** sur Arche.

**Ce qu'on dépose** : seulement le contenu de `livrables/livrable2_gitlab.zip`, c'est-à-dire :
- `README.md` : le README demandé par la consigne (configuration, architecture, démonstration). Il remplace celui du labo à la racine du dépôt ;
- `ros2_foxy/be_limo/` : notre paquet ROS 2 (code, launch, réglages, monde Gazebo, MATLAB, tests).

**Ce qu'on ne dépose pas** : le reste du GitHub (`limo/`, `environment/`, `CLAUDE.md`, `rendu_1/`, le PDF...).

Chaque étape indique **où** taper la commande. Regarde toujours le début de la ligne du terminal :

| Invite | Tu es sur |
|---|---|
| `e70348u@bi-fdd-f-l001:~$` | le PC (c'est là que tout se passe ici) |
| `root@bi-fdd-f-l001:/#` | le conteneur Docker : tape `exit` pour revenir au PC |
| `jetson@nano:~$` | le robot : tape `exit` pour revenir au PC |

Durée : 15 minutes environ.

---

## Étape 1 : créer un token GitLab (une seule fois)

Le terminal ne prend pas ton mot de passe universitaire : il faut un **token**.

1. Va sur **gitlab.univ-lorraine.fr** et connecte-toi.
2. Clique sur ton **avatar** (en haut à gauche ou à droite) → **Preferences** (ou **Edit profile**).
3. Menu de gauche : **Access tokens** (ou **Personal access tokens**) → **Add new token**.
4. Remplis :
   - **Token name** : `PC ecole`
   - **Expiration date** : dans un mois
   - **Scopes** : coche **`write_repository`** (et `read_repository`)
5. **Create personal access token**, puis **copie le token** (il commence par `glpat-`).

⚠️ Le token ne s'affiche qu'une fois. Ne le mets dans aucun fichier du dépôt.

---

## Étape 2 : récupérer l'adresse du dépôt du groupe

1. Sur GitLab : **Robotic lab** → **Students** → **BE** → **BE-2026-Group-01**.
2. Clique sur le bouton bleu **Code**.
3. Copie l'adresse sous **Clone with HTTPS** (elle se termine par `.git`).

Note aussi le **nom exact** du dépôt (par exemple `be-2026-group-01`) : il sert à l'étape 5.

---

## Étape 3 : récupérer la dernière version de notre travail

📍 **PC**, invite `e70348u@bi-fdd-f-l001:~$`
```bash
cd ~/ros2_ws/src/be-limo
git pull
ls livrables
```
✅ Tu vois `livrable2_gitlab.zip` (et ce fichier).

❌ GitHub demande un identifiant : Username `GiovanRD`, Password = ton **token GitHub** (`ghp_...`).

---

## Étape 4 : copier le dépôt GitLab du groupe sur le PC

On le met **en dehors** du workspace ROS (`~/ros2_ws/src`). Le dépôt du groupe contient aussi `room207_limo`, et `colcon` refuserait deux paquets du même nom.

📍 **PC**, invite `e70348u@bi-fdd-f-l001:~$`
```bash
mkdir -p ~/gitlab
cd ~/gitlab
git clone COLLE_ICI_L_ADRESSE_DE_L_ETAPE_2 groupe01
```
- **Username** : ton identifiant universitaire (ex. `e88049u`)
- **Password** : le **token GitLab** de l'étape 1 (collé avec Ctrl+Maj+V ; rien ne s'affiche, c'est normal)

✅ `Clonage dans 'groupe01'...` puis `Résolution des deltas: 100%`.

📍 **PC**, invite `e70348u@bi-fdd-f-l001:~/gitlab$`
```bash
cd groupe01
ls
```
✅ Tu vois au moins `README.md`, `matlab` et `ros2_foxy`.

❌ `Authentication failed` : le token est mal collé, ou il n'a pas `read_repository`/`write_repository`. Refais l'étape 1.
❌ `groupe01 existe déjà` : le dépôt a déjà été copié. Fais `cd groupe01 && git pull` et passe à l'étape 5.

---

## Étape 5 : y mettre notre travail

📍 **PC**, invite `e70348u@bi-fdd-f-l001:~/gitlab/groupe01$`
```bash
rm -rf ros2_foxy/be_limo
unzip -o ~/ros2_ws/src/be-limo/livrables/livrable2_gitlab.zip
ls ros2_foxy
```
✅ `be_limo` apparaît à côté de `room207_limo`.

### 5.1. Mettre ton adresse de l'université (au lieu de Gmail)

```bash
nano ros2_foxy/be_limo/package.xml
```
Ligne `<maintainer email="...">` : remplace l'adresse Gmail par ton adresse universitaire. Enregistre (**Ctrl+O**, Entrée) et quitte (**Ctrl+X**). Fais de même dans :
```bash
nano ros2_foxy/be_limo/setup.py
```
ligne `maintainer_email='...'`.

### 5.2. Vérifier le nom du dépôt dans le README

Le README écrit le dépôt sous le nom `be-2026-group-01`. Si le nom noté à l'étape 2 est différent :
```bash
grep -n "be-2026-group-01" README.md
```
puis corrige les lignes affichées avec `nano README.md`.

### 5.3. Vérifier ce qui va être envoyé

```bash
git status
```
✅ Tu dois voir **seulement** :
- `modifié : README.md`
- `ros2_foxy/be_limo/` dans les fichiers non suivis

❌ D'autres fichiers apparaissent : ne continue pas, et regarde ce que c'est.

---

## Étape 6 : envoyer sur GitLab

📍 **PC**, invite `e70348u@bi-fdd-f-l001:~/gitlab/groupe01$`

Ton nom et ton adresse de l'université apparaîtront comme auteur :
```bash
git config user.name "Giovan ROCCA-DURINGER"
git config user.email "TON_ADRESSE@etu.univ-lorraine.fr"
```
Puis :
```bash
git add README.md ros2_foxy/be_limo
git commit -m "Livrable 2 : planification, contrôle et détection en simulation"
git push
```
✅ Les dernières lignes ressemblent à `main -> main` (ou `master -> master`).

❌ `rejected ... (fetch first)` : quelqu'un du groupe a envoyé quelque chose entre-temps.
```bash
git pull --rebase
git push
```
❌ `You are not allowed to push` ou `403` : ton compte n'a pas le rôle *Developer* sur le dépôt, ou la branche est protégée. Demande à l'enseignant, ou passe par une **Merge request** (GitLab le propose dans le message d'erreur).

---

## Étape 7 : vérifier sur GitLab

Ouvre le dépôt **BE-2026-Group-01** dans le navigateur :

- [ ] la page d'accueil affiche **notre README** (titre « BE Véhicule Autonome 2026 — Groupe 1 ») ;
- [ ] le dossier **`ros2_foxy/be_limo`** existe, avec `be_limo/`, `launch/`, `config/`, `worlds/`, `matlab/`, `test/`, `README.md` ;
- [ ] les images de `ros2_foxy/be_limo/README.md` s'affichent (carte des nœuds) ;
- [ ] il n'y a **pas** de `CLAUDE.md`, ni les dossiers `limo/`, `environment/`.

---

## Étape 8 : déposer le lien sur Arche

1. Copie l'adresse de la page d'accueil du dépôt, dans la barre du navigateur. Elle ressemble à
   `https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/students/be/be-2026-group-01`
   (pas l'adresse en `.git` de l'étape 2).
2. Sur Arche, ligne **16/11/2026 — L2.1–L3.1** → **Déposer** → colle le lien → valide.

---

## Avant de déposer : tester la démonstration

L'enseignant va suivre le README : vérifiez que la démonstration se lance chez vous. Notre copie GitHub contient le même paquet, déjà dans le workspace :

📍 **PC**, invite `e70348u@bi-fdd-f-l001:~$`
```bash
xhost +local:docker
docker start -ai foxy
```
📍 **Conteneur**, invite `root@bi-fdd-f-l001:/#`
```bash
cd /root/ros2_ws
colcon build --symlink-install --packages-select be_limo
source install/setup.bash
ros2 launch be_limo sim_l2.launch.py
```
✅ RViz s'ouvre et le robot part tout seul. Il s'arrête au stop, puis devant l'obstacle. Pour retirer l'obstacle, dans un 2ᵉ terminal (`docker exec -it foxy bash`) :
```bash
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: obstacle_box}"
```
Notez ce qui se passe et prenez des captures d'écran : elles serviront pour l'oral.

---

## Récapitulatif

| # | Où | Quoi |
|---|---|---|
| 1 | GitLab (navigateur) | créer un token `write_repository` |
| 2 | GitLab (navigateur) | copier l'adresse HTTPS du dépôt du groupe |
| 3 | PC, `~/ros2_ws/src/be-limo` | `git pull` |
| 4 | PC, `~/gitlab` | `git clone <adresse> groupe01` |
| 5 | PC, `~/gitlab/groupe01` | `unzip -o .../livrable2_gitlab.zip`, corriger l'adresse e-mail et le nom du dépôt |
| 6 | PC, `~/gitlab/groupe01` | `git add`, `git commit`, `git push` |
| 7 | GitLab (navigateur) | vérifier README et `ros2_foxy/be_limo` |
| 8 | Arche | déposer le lien du dépôt |
