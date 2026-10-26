# Solution PARC Engineers League 2026 — Phase 1

Navigation autonome du robot CAYTU Sito-É dans le restaurant du stade intelligent,
de sa position de départ jusqu'à la zone objectif.

## Introduction

Le règlement de la tâche demande explicitement une solution **utilisant Nav2**. Nav2
planifie donc et conduit. Mais le `task.launch.py` fourni par les organisateurs ne
démarre que Gazebo, RViz, le `robot_state_publisher` et les ponts gz↔ROS : **aucune
carte, aucune localisation, aucun planificateur**. Tout ce qui entoure Nav2 est à
notre charge — et c'est précisément là que se joue cette tâche.

Notre parti pris : **ne pas réécrire Nav2, mais lui donner un monde dans lequel il a
raison**. Quatre nœuds de notre conception l'entourent, chacun répondant à un échec
que nous avons mesuré, pas supposé.

```
                    ┌──────────────────────────────────────┐
   /scan ──────────▶│ scan_filter    retire les échos du    │──▶ /scan_filtered
                    │                châssis (101/352)      │        │
                    └──────────────────────────────────────┘        │
                    ┌──────────────────────────────────────┐        ▼
   /scan ──────────▶│ evidence_map   grille log-odds en     │──▶ /parc/evidence_map
                    │                repère monde,          │   (couche statique
                    │                complétion des meubles  │    du costmap global)
                    └──────────────────────────────────────┘        │
   /odom ──┐        ┌──────────────────────────────────────┐        ▼
           ├───────▶│ world_anchor   TF map→odom :          │──▶ ┌──────────────┐
  pose GT ─┘        │                map ≡ repère monde      │    │              │
                    └──────────────────────────────────────┘    │     Nav2     │
                    ┌──────────────────────────────────────┐    │  (planner +  │
   /scan ──────────▶│ speed_governor plafond de vitesse     │──▶ │  controller) │
                    │                selon le dégagement    │    └──────────────┘
                    └──────────────────────────────────────┘            │
                    ┌──────────────────────────────────────┐            ▼
                    │ task_solution  mission, auto-notation │      /cmd_vel → robot
                    │                et chien de garde      │
                    └──────────────────────────────────────┘
```

## Les cinq idées

### 1. Supprimer le repère `map` en le confondant avec le repère monde

`task_params.yaml` donne l'objectif en coordonnées **monde Gazebo**. Nav2 planifie
dans `map`. Toutes nos versions précédentes comblaient cet écart **du côté de
l'objectif** : conversion `p_map = R(−yaw₀)·(p_monde − p₀)`, puis recalage permanent
de l'objectif à mesure que l'odométrie dérivait.

C'était la pire place possible pour cette correction : **chaque `goToPose` préempte la
navigation en cours**. Mesuré : *117 préemptions en 276 s*, soit une toutes les 2,4 s.
Le robot recommençait sans jamais suivre un plan jusqu'au bout.

`world_anchor.py` déplace la correction là où elle appartient, dans la transformation
`map → odom`. Il publie exactement la transformation qui fait coïncider la pose du
robot dans `map` avec sa pose réelle dans le monde. Conséquences en cascade :

| | Avant | Après |
|---|---|---|
| Objectif | converti, puis recalé en continu | **envoyé brut, une seule fois** |
| Dérive d'odométrie | absorbée par l'objectif (préemptions) | absorbée par la TF (invisible du planificateur) |
| Costmap global | fenêtre glissante (l'objectif sortait des limites) | **carte fixe de tout le restaurant** |
| Pièges de repère | conversion, `base_link` vs `base_footprint`, rotation de spawn | tous éliminés |

Deux précautions, chacune issue d'un échec :

* **Il ne saute pas.** Les corrections sont limitées à 0,10 m/s et 0,10 rad/s. Un
  localisateur qui téléporte `map` téléporte le costmap sous un plan en cours
  d'exécution : `slam_toolbox` faisait exactement cela ici (2,96 m, puis 1,42 m, puis
  1,97 m entre échantillons successifs) et les courses s'effondraient en boucles de
  récupération. L'odométrie dérive à ~0,003 m/s : le plafond de correction a cent fois
  la marge nécessaire tout en interdisant toute discontinuité.
* **Il ne poursuit pas le bruit.** La rotation de `map → odom` est filtrée très
  fortement (α = 0,02), parce qu'elle est quasi constante — elle ne bouge qu'avec la
  dérive de cap de l'odométrie. Chaque estimation instantanée, elle, porte l'erreur
  d'appariement de deux échantillons pris jusqu'à 0,2 s d'écart : en rotation à
  0,5 rad/s cela fait ~0,1 rad, et **l'erreur de translation que cela implique croît
  avec la distance du robot à l'origine d'`odom`** — à 7 m, 0,1 rad vaut 0,7 m de saut
  apparent. Sans ce filtre, l'ancre courait après ce bruit et se ré-ancrait toutes les
  20 s, traînant le costmap à chaque fois.
* **Il n'exige pas la vérité-terrain.** Gazebo ancre `odom` sur la pose de spawn : la
  transformation exacte à t=0 est donc la pose de spawn elle-même, lue dans
  `task_params.yaml`. C'est la valeur initiale, et le repli si le topic de pose
  n'arrive jamais — auquel cas la solution dégrade vers l'odométrie pure, `map` restant
  le repère monde.

**Auto-contrôle de repère.** Au premier échantillon, le nœud vérifie que la rotation
de `map → odom` vaut bien le lacet de spawn, et le dit dans le journal. Cette seule
ligne d'arithmétique aurait épargné des heures : `base_link` est tourné de 90° par
rapport à `base_footprint` (`sitoe_core.xacro`, `rpy="0 0 pi/2"`), ce qui fait qu'un
mauvais choix de repère produit un Nav2 d'apparence parfaitement saine qui conduit de
travers.

### 2. Une carte d'évidence LiDAR injectée dans Nav2 sans écrire de greffon C++

`evidence_map.py` construit une grille d'occupation **log-odds** (Elfes/Moravec) en
repère monde et la publie en `nav_msgs/OccupancyGrid`. La **couche statique** du
costmap global la consomme exactement comme elle consommerait la carte d'un
`map_server`. C'est l'interface que Nav2 possède déjà pour « voici à quoi ressemble le
monde » — aucun greffon à compiler.

Ce que Nav2 y gagne, que ses propres couches ne donnent pas :

* **Une mémoire en repère monde.** L'évidence accumulée 10 m plus tôt reste valide et
  au bon endroit. Les couches d'obstacles ne voient que ce que les capteurs atteignent.
* **De la confiance.** Un retour aberrant isolé ne peut pas bloquer un couloir ; un
  visiteur qui s'éloigne est effacé progressivement.
* **Le dimensionnement du costmap.** La carte couvre 34 × 34 m à 0,10 m : l'objectif y
  est, donc il ne peut plus être rejeté « hors limites » — un échec qui avortait
  l'arbre de comportement en 45 ms, trop vite pour ressembler à autre chose qu'une
  erreur de configuration.

Coût mesuré : **3,6 ms par balayage de 360 faisceaux**, en deux dispersions
vectorisées NumPy plutôt qu'une boucle par faisceau.

### 3. Compléter les meubles par fermeture morphologique

À la hauteur du LiDAR (6 cm), une `WoodenChair` n'est **pas** une chaise : c'est quatre
points de 3 cm séparés par 40 cm d'espace apparemment libre. N'importe quel
planificateur passe au travers — et le châssis supérieur du robot, à 0,69 m, heurte
l'assise.

Une **fermeture morphologique** de rayon 0,32 m recolle cette constellation en un objet
plein. Le rayon est choisi sur mesure, pas au jugé :

| Rayon | Cellules comblées dans une chaise | Porte de 0,9 m encore ouverte |
|---|---|---|
| 0,25 m | 0 (les disques se touchent, l'érosion défait le pont) | 9/9 |
| **0,32 m** | **9** | **9/9** |
| 0,40 m | 9 | 9/9 |

Refermer une ouverture dans un mur demande un rayon égal à la **moitié** de
l'ouverture : il existe donc une large bande entre « referme une chaise » (≥ 0,30 m) et
« referme une porte » (≥ 0,45 m). 0,32 m s'y trouve.

Ces cellules sont publiées à la valeur 60, non 100 : **chères mais franchissables**. Si
la fermeture venait malgré tout à combler un vrai passage, le planificateur peut encore
l'emprunter faute de mieux.

### 4. Un gouverneur de vitesse contextuel

Le temps d'exécution est l'un des trois critères notés. Or un plafond de vitesse unique
doit être choisi pour le pire instant de la course, puis payé pendant toute sa durée :
la configuration précédente parcourait les 10 m à 0,26 m/s parce que deux mètres
passent entre des tables.

`speed_governor.py` publie un `nav2_msgs/SpeedLimit` en pourcentage — l'interface de
Nav2 pour cela, celle qu'utiliserait un `SpeedFilter` — calculé sur le dégagement
frontal du LiDAR brut : 100 % au-delà de 3 m, 45 % à 0,9 m. Le plancher correspond à
l'ancien réglage fixe : le cas lent n'est pas plus agressif qu'avant, il n'est
simplement plus permanent.

La mesure se fait sur le scan brut et non sur le costmap, délibérément : elle doit
réagir à ce qui est devant le robot *maintenant*, sans attendre un cycle de mise à
jour du costmap qui, à RTF 0,07, arrive avec des centaines de millisecondes de retard.

### 5. Une couche mission qui connaît les règles

`task_solution.py` n'est pas qu'un envoyeur d'objectif :

* **Il s'auto-note sur les trois critères** — contacts (filtrés du sol, qui déclenche
  les pare-chocs en permanence), distance réellement parcourue, durée en temps
  **simulé** ; puis écrit un rapport JSON.
* **Il sait quand la tâche est gagnée.** Le règlement dit « dès qu'une partie
  quelconque du robot se trouve dans le cercle vert » (rayon 0,6 m). Le nœud déclare la
  victoire et annule la navigation dès que le **centre** du robot est à 0,45 m du
  centre du cercle : le robot y est sans ambiguïté, et l'on économise les secondes que
  le vérificateur d'objectif de Nav2 passerait à peaufiner la pose finale.
* **Il surveille la pile.** Le gestionnaire de cycle de vie détruit toute la pile si un
  nœud manque son battement de cœur, ce qui s'est produit pour de vrai
  (`CRITICAL FAILURE: SERVER collision_monitor IS DOWN after not receiving a heartbeat
  for 4000 ms`) : sous la charge de ce monde, un nœud simplement privé de CPU est
  déclaré mort. Le délai est porté à 60 s, et si la pile meurt malgré tout, le nœud la
  relance et renvoie l'objectif.
* **Il ne pose aucune question de service à Nav2.** `waitUntilNav2Active()` interroge
  `/bt_navigator/get_state` ; sous charge, cet aller-retour échoue
  (`failed to send response ... client will not receive response`) et la mission
  attendait indéfiniment une pile pourtant active. Attendre l'apparition du serveur
  d'action `NavigateToPose` est à la fois nécessaire et suffisant, et ne demande de
  réponse à personne.

## Exécution

**Terminal 1 — simulation fournie par la compétition :**

```bash
ros2 launch parc_robot_bringup task.launch.py
```

**Terminal 2 — notre solution :**

```bash
ros2 run parc_solution task_solution.py
```

C'est tout : si la pile de navigation n'est pas déjà active, `task_solution.py` la
démarre lui-même. Pour la lancer séparément (utile en débogage, afin de voir les
journaux de Nav2 et des nœuds de perception) :

```bash
ros2 launch parc_solution navigation.launch.py   # terminal 2
ros2 run parc_solution task_solution.py          # terminal 3
```

## Dépendances

Paquets ROS 2 Jazzy installés en plus de `ros-jazzy-desktop` :

```bash
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-image-transport-plugins \
  ros-jazzy-topic-tools \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-simple-commander \
  python3-numpy python3-scipy
```

Les six premiers sont des dépendances des paquets fournis par la compétition ; les
autres sont propres à notre solution. Ils sont aussi déclarés dans `package.xml`.

Environnement : Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Sim 8 (Harmonic).

## Structure

```
solutions/parc_solution/
├── config/nav2_params.yaml        # Nav2, adapté au Sito-É
├── behavior_trees/                # arbre sans effacement du costmap global
├── launch/
│   ├── navigation.launch.py       # nos 4 nœuds + Nav2
│   └── nav2_navigation.launch.py  # copie allégée du bringup Nav2
└── parc_solution/
    ├── task_solution.py           # mission, auto-notation, chien de garde
    ├── world_anchor.py            # map ≡ repère monde
    ├── evidence_map.py            # carte d'évidence → couche statique
    ├── perception.py              # grille log-odds + complétion des meubles
    ├── speed_governor.py          # plafond de vitesse contextuel
    └── scan_filter.py             # filtre des échos du châssis
```

## Difficultés rencontrées

Chacune de ces entrées est une mesure, pas une impression.

1. **Le LiDAR voit le châssis du robot.** 101 des 352 retours valides reviennent à
   0,05 m (le `range_min` du capteur) dans un arc derrière le robot. Marqués comme
   obstacle au centre même du robot, ils font gonfler le costmap jusqu'à `INSCRIBED` :
   le planificateur trouve sa propre cellule de départ bloquée, chaque
   `ComputePathToPose` échoue et **le robot ne bouge pas du tout**. Filtrés sous 0,45 m
   en amont du SLAM comme des costmaps.

2. **Le pont de pose des organisateurs est muet.** `gz_bridge.yaml` déclare le topic gz
   `sitoe_robot/pose`, mais le greffon `PosePublisher` publie sur
   `/model/sitoe_robot/pose`. Vérifié avec `gz topic -e` : le premier ne délivre jamais
   un message, le second émet à 10 Hz. Nous pontons le vrai topic sans modifier leur
   paquet. **À signaler aux organisateurs.**

3. **`base_link` est tourné de 90° par rapport à `base_footprint`**, et le greffon
   DiffDrive conduit dans `base_footprint`. Nav2 utilise `base_link` par défaut : le
   modèle différentiel raisonnait dans un repère perpendiculaire à la marche.
   *Corollaire trompeur* : comparer le lacet monde du robot à la TF `map → base_link`
   ne montre aucune rotation, ce qui semble prouver que `map` n'est pas tourné. C'est un
   artefact — le +90° de `base_link` annule le −90° d'`odom`. La vérification fiable est
   odométrie contre vérité-terrain : `/odom` indiquait (4,13 ; −0,07) quand le
   déplacement réel valait (−0,05 ; 3,87).

4. **La préemption par recalage d'objectif** — 117 préemptions en 276 s. Voir l'idée 1.

5. **La caméra de profondeur construisait un mur fantôme.** En rendant le costmap
   global fixe (idée 1), les marques de la caméra cessent de défiler hors de la fenêtre
   et deviennent **permanentes**. Résultat mesuré : une barrière continue de 5 m
   (x de −2,6 à +2,4 à y = −5,7) là où notre carte LiDAR, au même instant et dans le
   même repère, ne montrait que deux pieds de table. La salle était scellée :
   « `Failed to create plan with tolerance of: 0.500000` » depuis toutes les poses,
   l'arbre de comportement tombait en récupérations `spin`/`wait`, et le robot tournait
   sur place pendant 150 s pour 1,1 m parcourus.
   **Correction** : la caméra ne marque plus que dans le costmap **local**, qui est
   glissant et reconstruit en continu. Les plateaux de table ne sont pas perdus pour
   autant — le contrôleur les voit toujours — et globalement le LiDAR voit le pied de
   chaque table, dont l'inflation de 0,60 m couvre largement le débord de 0,45 m du
   plateau.

   **Et la même caméra bloquait aussi le costmap local.** Nous avions supposé que « les
   retours du sol commencent sous 0,30 m ». Mesure sur une trame réelle, ramenée dans
   `base_footprint` :

   | Bande de hauteur | Points | Ce que c'est |
   |---|---|---|
   | z < 0,05 m | 630 | **le sol**, à 1,2–3,7 m — la caméra le voit bel et bien |
   | 0,30–0,45 m | 267 | ni sol ni plateau : assises de chaise, jambes, bruit de profondeur |
   | 0,70–0,90 m | 640 | **les plateaux** (0,735–0,775 m), ce que seule elle peut voir |

   Avec un plancher à 0,30 m, la bande du milieu marquait, et avec elle assez de frange
   bruitée du sol pour que **31 % du costmap local revienne létal** alors que le robot
   se tenait entre deux tables distantes de 2,5 m : le contrôleur ne trouvait pas de
   passage, le plan basculait entre deux contournements (4,61 ↔ 4,95 m) et la course
   s'enlisait en boucles de récupération à 4,8 m du but. Le plancher est porté à
   **0,55 m** — dans la bande vide entre les deux — et l'inflation locale ramenée à
   0,45 m, parce qu'un halo de 0,60 m autour de deux tables rend le milieu d'un
   intervalle de 2,5 m coûteux des deux côtés à la fois.

6. **MPPI avançait à 0,047 m/s.** Sur un chemin dégagé, sans intervention du moniteur de
   collision, avec des commandes à 20 Hz, MPPI commandait une vitesse moyenne de
   0,047 m/s pour un plafond de 0,36. Ce monde tourne à RTF 0,07 : le contrôleur tourne
   en temps réel bien plus vite que le temps simulé n'avance (`Control loop missed its
   desired rate ... Current loop rate is inf Hz` dans tout le journal), si bien que
   chaque cycle n'exécute qu'une tranche infime de son plan avant d'être remplacé, et
   la rampe de vitesse ne s'établit jamais. **Regulated Pure Pursuit** calcule sa
   commande géométriquement à partir du chemin : il est insensible à cette pathologie
   de cadencement, coûte une fraction du CPU, et ses règles de régulation — ralentir en
   courbe, ralentir près des obstacles — encodent exactement la prudence qu'exige un
   robot que l'on a déjà mesuré en train de basculer (tangage +90°).

7. **Chronométrage en temps simulé — et le piège qui se cache derrière.** Gazebo tourne
   très en dessous du temps réel : un chronomètre `time.time()` annulait la tâche après
   quelques secondes *simulées*. Utiliser l'horloge ROS ne suffit pourtant pas :
   `BasicNavigator` ne positionne pas `use_sim_time`, donc **son horloge est l'horloge
   réelle** et toutes nos durées l'étaient aussi, sans le dire. À RTF 0,07, le budget
   de 570 s aurait expiré après ~40 s simulées, en pleine course réussie. Et une fois
   le paramètre posé, il faut encore **attendre le premier message `/clock`** : une
   horloge simulée non initialisée lit zéro, et le premier `elapsed` valait alors la
   durée de fonctionnement du simulateur (course démarrée à « [179,1 s] » d'un budget
   de 570).

8. **Sous charge, Nav2 perd ses réponses.** `waitUntilNav2Active()` interroge
   `/bt_navigator/get_state` ; nous avons observé
   `failed to send response ... client will not receive response` sur une pile
   pourtant active, et la mission attendait indéfiniment. Même chose à l'envoi de
   l'objectif : `Failed to send goal response (timeout)` — objectif accepté, accusé de
   réception perdu, et `goToPose()` attend sans délai maximal. Nous attendons désormais
   le serveur d'action plutôt que d'interroger un service, et l'envoi d'objectif a un
   délai et des tentatives.

9. **Un obstacle fantôme *sur* le robot.** Le robot s'est enlisé à 4,7 m du but,
   tournant sur place. Mesure à cet instant : sur 1550 cellules létales du costmap
   local, **1006 n'étaient confirmées par aucun retour LiDAR**, et la plus proche se
   trouvait à **0,12 m du centre du robot — à l'intérieur de son propre gabarit**. La
   vérification de collision du contrôleur trouvait donc le robot en collision où qu'il
   se tienne. C'est exactement le défaut que `scan_filter.py` corrige pour le LiDAR ;
   la caméra avait le même, non filtré. Corrigé par `footprint_clearing_enabled: True`
   (Nav2 le laisse à `false` sur la `VoxelLayer`) et un `obstacle_min_range` de 0,45 m
   sur le nuage de points.

10. **Une redondance que nous avons essayé de supprimer — et remise.** La couche
    d'obstacles de Nav2 et notre carte d'évidence sont deux mémoires du même capteur, et
    la plus fruste n'oublie jamais : le costmap global étant fixe, ce qu'elle a marqué
    reste marqué. Mesure pendant un blocage : sur 14757 cellules létales, 10889 (74 %)
    correspondaient à un obstacle de la carte d'évidence et **3828 (26 %) ne
    correspondaient à rien** — dont *la cellule du but elle-même*, ce qui explique le
    « Failed to create plan » alors que le robot se tenait en espace libre à 4,6 m.
    Nous avons donc retiré la couche d'obstacles globale. **Le résultat fut pire** :
    notre carte ne se publiant qu'à 1 Hz, être la seule source globale a suffi à laisser
    un plan couper un virage que le robot ne pouvait pas prendre — la course suivante a
    produit le **premier contact** de cette solution, contre `cafe_table_1::link::surface`,
    et s'est enlisée plus tôt. La redondance coûte des cellules périmées ; la supprimer
    coûte des collisions. Elle est conservée.

11. **`docking_server` et la course au démarrage.** Sous charge, il répondait au
   gestionnaire de cycle de vie quelques millisecondes après l'abandon de celui-ci,
   faisant échouer tout le démarrage. Nous embarquons une copie allégée du launch Nav2
   sans les serveurs inutilisés.

12. **Faux positifs du compteur de contacts.** Les pare-chocs signalent en permanence le
   sol (`cafe::link::main_floor`). Sans filtrage, le compteur montait dès la première
   seconde.

## Note de développement — performance de la simulation

La simulation officielle rend deux caméras D435i en 640×480 (couleur 30 Hz, profondeur
15 Hz), soit 90 rendus par seconde simulée. Mesures sur notre machine :

| Configuration | RTF |
|---|---|
| Officielle | 0,019 |
| Caméras à 2 Hz (développement) | 0,162 |
| + RViz fermé | 0,214 |

Ces réglages de développement (`parc_robot_description/urdf/d435.xacro`) sont **à
annuler avant soumission** — `git checkout parc_robot_description` — : ils ne font pas
partie de la solution et ne modifient aucun de ses comportements. Notre solution
n'utilise le flux de la caméra que dans le costmap local.

Remarque de méthode : nous avions d'abord supposé que RViz était le goulot
d'étranglement. La mesure l'a infirmé — tant que les caméras tournaient à pleine
cadence, fermer RViz ne faisait passer le RTF que de 0,019 à 0,021.
