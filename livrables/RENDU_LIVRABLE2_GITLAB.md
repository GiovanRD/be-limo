# Rendre le livrable 2 sur le GitLab de l'université

**But** : mettre notre travail dans le dépôt GitLab du groupe (`Students/BE/BE-2026-Group-01`), puis déposer **le lien de ce dépôt** sur Arche.

**Ce qu'on dépose** : seulement le contenu de `livrables/livrable2_gitlab.zip`, c'est-à-dire :
- `README.md` : le README demandé par la consigne (configuration, architecture, démonstration). Il **remplace** celui du labo à la racine du dépôt ;
- le dossier `ros2_foxy/be_limo/` : notre paquet ROS 2 (code, launch, réglages, monde Gazebo, MATLAB, tests).

**Ce qu'on ne dépose pas** : le reste du GitHub (`limo/`, `environment/`, `CLAUDE.md`, `rendu_1/`, le PDF...).

**Deux méthodes, au choix**

| | Méthode A : terminal + token | Méthode B : à la main dans le navigateur |
|---|---|---|
| Outils | terminal du PC, git | navigateur seulement |
| Avantages | rapide, fiable, tous les fichiers d'un coup | rien à installer, pas de token |
| Inconvénients | il faut créer un token | plus de clics ; dépend de la version de GitLab |
| Durée | ~15 min | ~20 min |

Les deux méthodes donnent le même résultat. Finis ensuite par la partie **C. Vérifier et déposer sur Arche**.

---

## Avant de commencer (les deux méthodes)

### Récupérer l'adresse et le nom du dépôt du groupe

1. Va sur **gitlab.univ-lorraine.fr** et connecte-toi.
2. **Robotic lab** → **Students** → **BE** → **BE-2026-Group-01**.
3. Note le **nom exact** du dépôt (par exemple `be-2026-group-01`).
4. Pour la méthode A seulement : clique sur le bouton bleu **Code**, et copie l'adresse sous **Clone with HTTPS** (elle se termine par `.git`).

### Vérifier tes droits

Menu de gauche du dépôt → **Manage** → **Members** : ton compte doit avoir le rôle **Developer** (ou plus). Sinon, tu ne pourras pas envoyer de fichiers : demande à l'enseignant.

---

# A. Méthode avec le terminal et un token

Chaque étape indique **où** taper la commande. Regarde le début de la ligne du terminal :

| Invite | Tu es sur |
|---|---|
| `e70348u@bi-fdd-f-l001:~$` | le PC (c'est là que tout se passe ici) |
| `root@bi-fdd-f-l001:/#` | le conteneur Docker : tape `exit` pour revenir au PC |
| `jetson@nano:~$` | le robot : tape `exit` pour revenir au PC |

## A1. Créer un token GitLab (une seule fois)

Le terminal n'accepte pas le mot de passe universitaire : il faut un **token**.

1. Sur GitLab, clique sur ton **avatar** → **Preferences** (ou **Edit profile**).
2. Menu de gauche : **Access tokens** (ou **Personal access tokens**) → **Add new token**.
3. Remplis :
   - **Token name** : `PC ecole`
   - **Expiration date** : dans un mois
   - **Scopes** : coche **`read_repository`** et **`write_repository`**
4. **Create personal access token**, puis **copie le token** (il commence par `glpat-`).

⚠️ Le token ne s'affiche qu'une fois. Ne le mets dans aucun fichier du dépôt.

## A2. Récupérer la dernière version de notre travail

📍 **PC**, invite `e70348u@bi-fdd-f-l001:~$`
```bash
cd ~/ros2_ws/src/be-limo
git pull
ls livrables
```
✅ Tu vois `livrable2_gitlab.zip`.
❌ GitHub demande un identifiant : Username `GiovanRD`, Password = ton **token GitHub** (`ghp_...`).

## A3. Copier le dépôt GitLab du groupe sur le PC

On le met **en dehors** du workspace ROS (`~/ros2_ws/src`). Le dépôt du groupe contient aussi `room207_limo`, et `colcon` refuserait deux paquets du même nom.

📍 **PC**, invite `e70348u@bi-fdd-f-l001:~$`
```bash
mkdir -p ~/gitlab
cd ~/gitlab
git clone COLLE_ICI_L_ADRESSE_HTTPS groupe01
```
- **Username** : ton identifiant universitaire (ex. `e88049u`)
- **Password** : le **token GitLab** (collé avec Ctrl+Maj+V ; rien ne s'affiche, c'est normal)

✅ `Clonage dans 'groupe01'...` puis `Résolution des deltas: 100%`.
❌ `Authentication failed` : le token est mal collé ou n'a pas les bons droits. Refais A1.
❌ `groupe01 existe déjà` : fais `cd groupe01 && git pull`, puis passe à A4.

## A4. Y mettre notre travail

📍 **PC**, invite `e70348u@bi-fdd-f-l001:~$`
```bash
cd ~/gitlab/groupe01
rm -rf ros2_foxy/be_limo
unzip -o ~/ros2_ws/src/be-limo/livrables/livrable2_gitlab.zip
ls ros2_foxy
```
✅ `be_limo` apparaît à côté de `room207_limo`.

**Ton adresse de l'université au lieu de Gmail** :
```bash
nano ros2_foxy/be_limo/package.xml
```
Sur la ligne `<maintainer email="...">`, remplace l'adresse Gmail. Enregistre (**Ctrl+O**, Entrée) et quitte (**Ctrl+X**). Fais de même dans `nano ros2_foxy/be_limo/setup.py` (ligne `maintainer_email='...'`).

**Nom du dépôt dans le README** : s'il n'est pas `be-2026-group-01` :
```bash
grep -n "be-2026-group-01" README.md
```
puis corrige les lignes affichées avec `nano README.md`.

**Vérifier ce qui va partir** :
```bash
git status
```
✅ Seulement `modifié : README.md` et `ros2_foxy/be_limo/` (fichiers non suivis).

## A5. Envoyer sur GitLab

📍 **PC**, invite `e70348u@bi-fdd-f-l001:~/gitlab/groupe01$`
```bash
git config user.name "Giovan ROCCA-DURINGER"
git config user.email "TON_ADRESSE@etu.univ-lorraine.fr"
git add README.md ros2_foxy/be_limo
git commit -m "Livrable 2 : planification, contrôle et détection en simulation"
git push
```
✅ Les dernières lignes ressemblent à `main -> main` (ou `master -> master`).
❌ `rejected ... (fetch first)` : quelqu'un a envoyé quelque chose entre-temps. Fais `git pull --rebase` puis `git push`.
❌ `You are not allowed to push` / `403` : pas le rôle Developer, ou branche protégée. Voir « Vérifier tes droits », ou passe par la méthode B avec une *merge request*.

➡️ Passe à la partie **C**.

---

# B. Méthode à la main, dans le navigateur

Pas de terminal ni de token : tout se fait sur gitlab.univ-lorraine.fr.

## B1. Récupérer et décompresser le zip

1. Télécharge `livrable2_gitlab.zip` :
   - depuis notre GitHub : **github.com/GiovanRD/be-limo** → dossier **livrables** → `livrable2_gitlab.zip` → bouton **Download** (icône de téléchargement en haut à droite du fichier) ;
   - ou depuis la conversation où tu l'as reçu.
2. Décompresse-le : clic droit sur le fichier → **Extraire ici**.
3. Tu obtiens :
   ```text
   README.md
   ros2_foxy/
   └── be_limo/
   ```

**Ton adresse de l'université au lieu de Gmail** : ouvre `ros2_foxy/be_limo/package.xml` et `ros2_foxy/be_limo/setup.py` avec un éditeur de texte, remplace l'adresse Gmail, enregistre.
**Nom du dépôt** : s'il n'est pas `be-2026-group-01`, ouvre `README.md`, cherche (Ctrl+F) `be-2026-group-01` et corrige.

## B2. Ouvrir l'éditeur web de GitLab (Web IDE)

1. Ouvre le dépôt **BE-2026-Group-01** sur GitLab.
2. Clique sur **Edit** → **Web IDE** (ou appuie sur la touche **.** du clavier sur la page du dépôt).
3. Un éditeur ressemblant à VS Code s'ouvre, avec la liste des fichiers du dépôt à gauche.

## B3. Ajouter le dossier `be_limo`

1. Dans la liste de gauche, déplie le dossier **ros2_foxy**.
2. Depuis ton explorateur de fichiers, **glisse-dépose le dossier `be_limo`** (celui de `ros2_foxy/be_limo` décompressé) **sur le dossier `ros2_foxy`** du Web IDE.
3. Vérifie que `ros2_foxy/be_limo/` apparaît, avec ses sous-dossiers `be_limo`, `launch`, `config`, `worlds`, `rviz`, `matlab`, `test`, `doc`, `resource` et les fichiers `README.md`, `package.xml`, `setup.py`, `setup.cfg`.

❌ Le glisser-déposer de dossier ne marche pas (navigateur ou version de GitLab) : clic droit sur `ros2_foxy` → **New folder** `be_limo`, puis clic droit sur ce dossier → **Upload...** pour envoyer les fichiers, dossier par dossier. C'est plus long, mais ça marche partout. Pense à recréer chaque sous-dossier (`be_limo`, `launch`, `config`...) avant d'y envoyer ses fichiers.

## B4. Remplacer le README à la racine

1. Dans la liste de gauche, ouvre **README.md** (à la racine, pas celui de `be_limo`).
2. Sélectionne tout (**Ctrl+A**) et supprime.
3. Ouvre notre `README.md` décompressé avec un éditeur de texte, copie tout (**Ctrl+A**, **Ctrl+C**), puis colle-le dans le Web IDE (**Ctrl+V**).

## B5. Enregistrer (commit)

1. Clique sur l'icône **Source Control** dans la barre de gauche (les trois ronds reliés, avec un chiffre : le nombre de fichiers modifiés).
2. Vérifie la liste : `README.md` modifié, et les fichiers de `ros2_foxy/be_limo/` ajoutés. **Rien d'autre.**
3. Message : `Livrable 2 : planification, contrôle et détection en simulation`
4. Clique sur **Commit and push to 'main'** (ou **Commit to 'main'**).
   - Si GitLab propose **Create new branch** parce que `main` est protégée : accepte, puis crée la **merge request** proposée et fusionne-la (**Merge**). Si tu n'as pas le droit de fusionner, demande à l'enseignant.

✅ Un message confirme le commit. En revenant sur la page du dépôt, la date du dernier commit est la tienne.

➡️ Passe à la partie **C**.

---

# C. Vérifier et déposer sur Arche (les deux méthodes)

## C1. Vérifier sur GitLab

Ouvre le dépôt **BE-2026-Group-01** dans le navigateur :

- [ ] la page d'accueil affiche **notre README** (titre « BE Véhicule Autonome 2026 — Groupe 1 ») ;
- [ ] le dossier **`ros2_foxy/be_limo`** existe, avec `be_limo/`, `launch/`, `config/`, `worlds/`, `matlab/`, `test/`, `README.md` ;
- [ ] `ros2_foxy/be_limo/be_limo/` contient bien les fichiers `.py` (ex. `pure_pursuit.py`, `global_planner_node.py`) ;
- [ ] la carte des nœuds s'affiche dans `ros2_foxy/be_limo/README.md` ;
- [ ] il n'y a **pas** de `CLAUDE.md`, ni de dossiers `limo/` ou `environment/` ;
- [ ] `ros2_foxy/room207_limo` et `matlab/` (fournis par le labo) sont toujours là.

## C2. Déposer le lien sur Arche

1. Copie l'adresse de la **page d'accueil** du dépôt (barre du navigateur). Elle ressemble à
   `https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/students/be/be-2026-group-01`
   (pas l'adresse en `.git`).
2. Sur Arche, ligne **16/11/2026 — L2.1–L3.1** → **Déposer** → colle le lien → valide.

---

## Avant de déposer : tester la démonstration

L'enseignant suivra le README : vérifiez que la démonstration se lance chez vous. Notre copie GitHub contient le même paquet, déjà dans le workspace :

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

| # | Méthode A (terminal) | Méthode B (navigateur) |
|---|---|---|
| 1 | créer un token `write_repository` | télécharger et décompresser le zip |
| 2 | `git pull` du GitHub | corriger e-mail et nom du dépôt dans les fichiers |
| 3 | `git clone` du dépôt du groupe dans `~/gitlab` | ouvrir le Web IDE du dépôt |
| 4 | `unzip -o` du zip, corriger e-mail et nom du dépôt | glisser `be_limo` dans `ros2_foxy` |
| 5 | `git add`, `git commit`, `git push` | remplacer `README.md`, puis *Commit and push* |
| 6 | **C** : vérifier sur GitLab, déposer le lien sur Arche | **C** : vérifier sur GitLab, déposer le lien sur Arche |
