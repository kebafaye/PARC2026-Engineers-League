# Déploiement sur le CAYTU Sito-É réel

Ce que l'équipe installe et lance sur le robot, et ce qui reste au vestiaire.
Le règlement demande ces instructions au gel du code.

## Ce qu'on déploie

Un seul paquet : **`parc_solution`**. Le copier dans le dossier de l'équipe sur le
robot et construire par-dessus l'espace de travail de base — sans jamais toucher à
celui-ci, ni aux dossiers des autres équipes, ni aux services système.

```bash
# sur le robot, dans le dossier de l'equipe
mkdir -p ~/<equipe>/ros2_ws/src
cp -r parc_solution ~/<equipe>/ros2_ws/src/
cd ~/<equipe>/ros2_ws
colcon build --packages-select parc_solution --symlink-install
source install/setup.bash
```

**Ne pas déployer `test_assets/`** : décors de simulation et outils de mesure, rien
n'en dépend à l'exécution.

## Ce qui ne tourne PAS sur le robot réel

Ces nœuds existent pour la Phase 1 et dépendent de données que le robot réel n'a pas.
Les laisser dans le paquet ne gêne pas — il suffit de ne pas les lancer.

| Nœud | Pourquoi |
|---|---|
| `world_anchor.py` | lit la pose vérité-terrain Gazebo `/model/sitoe_robot/pose` |
| `task_solution.py` | même dépendance, et vise un but fixe en coordonnées monde |
| `evidence_map.py` | capteurs de choc simulés ; remplacé par la carte de slam_toolbox |
| `config/locations.yaml` | lieux du restaurant **simulé**, sans rapport avec l'arène |

`slam_navigation.launch.py` ne lance aucun d'eux — c'est exactement sa raison d'être.

## Dépendances système

Pas fournies sur le robot. **Demander aux organisateurs avant d'installer** (le
règlement l'impose) :

```bash
sudo apt install -y \
  ros-jazzy-slam-toolbox \
  ros-jazzy-apriltag-ros ros-jazzy-apriltag-msgs \
  ros-jazzy-image-proc
```

Si l'installation système est refusée, ces quatre paquets peuvent être construits
depuis les sources dans l'espace de travail de l'équipe.

## Lancer la solution

Trois terminaux SSH. `use_sim_time:=False` partout : le robot est sur l'horloge réelle.

```bash
# 1 - localisation, cartographie et navigation
ros2 launch parc_solution slam_navigation.launch.py use_sim_time:=False

#     ATTENDRE "Managed nodes are active" avant la suite. slam_toolbox est un noeud
#     a cycle de vie : s'il n'est pas active, Nav2 cale en repetant
#     "Invalid frame ID map". Le lifecycle_manager du lanceur s'en charge, mais la
#     sequence prend quelques secondes.

# 2 - detection des tags
ros2 launch parc_solution apriltag_detection.launch.py

# 3 - pilotage vers un tag
ros2 run parc_solution tag_goal_bridge.py
```

Déclencher une visite :

```bash
ros2 topic pub --once /parc/go_to_tag_id std_msgs/msg/Int32 "data: 4"
```

Le nœud annonce le verdict selon les deux critères du règlement :

```
VISITE VALIDE - tag36h11:4: 0.38 m (limite 0.50), desalignement 4.5 deg (limite 12)
```

## Démarrage vocal (+5 points, facultatif)

Hors ligne, sur le robot, via le réseau 4 micros ReSpeaker. À installer **pendant un
créneau d'essai**, pour que rien ne soit téléchargé le jour de la course :

```bash
pip install vosk sounddevice
# petit modèle français, ~50 Mo - le poser dans le dossier de l'equipe
```

```bash
# 4e terminal, lance AVANT race_director
ros2 run parc_solution voice_start.py --word allez --model ~/<equipe>/vosk-model-small-fr
```

`race_director` attend `/parc/start_race` ; ce nœud y publie une seule fois quand il
entend le mot. Rien d'autre ne commande le robot, donc la règle « ne pas bouger avant
le mot » est garantie par cette attente et non par la finesse du détecteur.

**En cas de doute, ne pas le lancer.** Démarrer la course à la main coûte les 5 points
du bonus et rien de plus ; un déclenchement par le bruit du public fait recommencer la
course *et* perdre le bonus. Le nœud journalise chaque candidat rejeté avec sa raison —
c'est ce qu'il faut lire pendant un essai pour régler `MIN_CONFIDENCE` sur la salle
réelle.

Répétition sans micro, pour vérifier le câblage :

```bash
ros2 run parc_solution voice_start.py --word allez --fire-now
```

## À vérifier dès le premier créneau d'essai

Par ordre d'importance. Les trois premiers peuvent invalider la solution entière.

1. **Les noms de topics.** Le guide annonce ceux de la Phase 1 (`/scan`, `/imu`,
   `/top_camera_color/image_raw`), mais le relevé `ros2_topics.md` fourni montre
   `/cmd_vel`, `/botwheel_explorer/odom`, `/camera1/camera1/color/image_raw`. Les deux
   ne peuvent pas être vrais. `ros2 topic list` sur le robot tranche. Si c'est le
   second jeu, il faut remapper : l'entrée `scan_topic` de `slam_params.yaml`, les
   remappages de `apriltag_detection.launch.py`, et le `cmd_vel` de Nav2.

2. **La taille des tags.** `apriltag_tags.yaml` déclare `size: 0.165`, le chiffre
   publié. La profondeur rapportée y est proportionnelle, et une erreur ne se voit
   pas dans la détection. Se placer à une distance mesurée au mètre ruban et lancer
   `test_assets/calibrate_tag_size.py` : si la valeur imprimée diffère, c'est elle
   qui gagne — elle passe par l'objectif et l'impression réels.

3. **La portée de détection.** Un tag de 16,5 cm n'atteint la taille en pixels qui
   décode proprement qu'à environ **1,3 m** avec la résolution de la Phase 1. Si les
   RealSense du robot publient plus haute résolution, la portée augmente d'autant —
   à mesurer, parce que la stratégie d'exploration en dépend directement.

4. **Le topic d'odométrie.** C'est lui qui protège le robot réel des contacts. Les
   capteurs de choc du réflexe sont simulés et n'existent pas ici ; à leur place, un
   détecteur de blocage compare « Nav2 commande » à « les roues tournent ». Par
   défaut il lit `/odom` — si le robot publie `/botwheel_explorer/odom`, le passer
   en paramètre :

   ```bash
   ros2 run parc_solution tag_goal_bridge.py --ros-args \
     -p odom_topic:=/botwheel_explorer/odom
   ```

   Sans le bon topic, il ne verra jamais rien et le robot roulera sans protection,
   à 2 à 5 points par contact.

5. **Les scans des caméras de profondeur.** Le robot publie `/camera1/scan` et
   `/camera2/scan`, qui voient ce que le LiDAR ne voit pas. Les ajouter à la couche
   d'obstacles de `nav2_params.yaml` ferait éviter les plateaux de table *avant* de
   les toucher, au lieu de réagir après. Pas encore fait, faute de pouvoir le tester
   ici — c'est la meilleure amélioration restante si un créneau le permet.

6. **La hauteur du LiDAR.** Il est à ~4,8 cm du sol : il voit les pieds de table,
   jamais un plateau à 0,75 m. Seules les caméras de profondeur voient ce qui dépasse.
   À garder en tête en regardant la carte construite.
