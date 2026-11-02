# WSL

## C'est quoi WSL

WSL (Windows Subsystem for Linux) permet de lancer un vrai Linux directement dans Windows, sans machine virtuelle ni dual boot. On l'utilise ici pour avoir un environnement ROS 2 prêt à l'emploi.

## Description de la configuration

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic

## Deux terminaux

On utilise deux terminaux différents. Chaque commande de ce README indique dans lequel la taper.

- **PowerShell (Windows)** : chercher « PowerShell » dans le menu Démarrer. Sert à gérer WSL et la manette (`wsl ...`, `usbipd ...`).
- **Terminal WSL (Linux)** : une fois l'environnement installé, chercher « ubuntu-ros2 » dans le menu Démarrer, ou taper `wsl -d ubuntu-ros2` dans PowerShell. Sert à tout le travail ROS 2 (`ros2`, `rviz2`, `ls`...).

## Installation (salle 206 rouge)

L'image est déjà présente sur chaque PC dans `C:\WSL\ubuntu-ros2`.

1. **PowerShell** : importer l'image (une seule fois par compte) :

   ```powershell
   wsl --import-in-place ubuntu-ros2 "C:\WSL\ubuntu-ros2\ext4.vhdx"
   ```

2. **PowerShell** : lancer l'environnement (ou « ubuntu-ros2 » dans le menu Démarrer) :

   ```powershell
   wsl -d ubuntu-ros2
   ```

## Connecter la manette (TP3 Francis Colas)

> **Attention :** l'interrupteur de la manette doit être sur `D`. Le mode X ne marche pas dans WSL.

La manette est déjà partagée sur tous les PC, **uniquement sur le port USB en haut à gauche de la façade avant**. Brancher le récepteur sur ce port-là.

1. **Terminal WSL** : ouvrir un terminal WSL (il doit rester ouvert).

2. **PowerShell** : vérifier que le récepteur est sur le `BUSID` `1-13` (le port en haut à gauche, le même sur tous les PC), puis attacher la manette :

   ```powershell
   usbipd list
   usbipd attach --wsl --busid 1-13
   ```

   > **Attention :** dans `usbipd list`, le `VID:PID` du récepteur doit finir par `219` (`046d:c219`). S'il finit par `21f`, l'interrupteur est sur X : le mettre sur `D` et relancer `usbipd list`.

3. **Terminal WSL** : lancer le nœud de la manette (le laisser tourner) :

   ```bash
   ros2 run joy joy_node
   ```

4. **Deuxième terminal WSL** : vérifier que la manette envoie des messages :

   ```bash
   ros2 topic echo /joy
   ```

   Les valeurs doivent changer quand on bouge les joysticks ou qu'on appuie sur les boutons.

> **Attention :** la manette se met parfois en veille. Appuyer sur n'importe quelle touche pour la réveiller.

## Avant de partir

> **Attention :** les PC de la salle ne sont pas une sauvegarde. Pousser son travail sur git (`git push`) ou le copier sur un disque dur / une clé USB avant de partir.

**PowerShell** : détacher la manette puis fermer WSL, sinon l'élève suivant ne pourra pas ouvrir l'environnement :

```powershell
usbipd detach --busid 1-13
wsl --shutdown
```

Si l'élève d'avant a oublié et que l'environnement ne s'ouvre pas, redémarrer l'ordinateur.