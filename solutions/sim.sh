#!/usr/bin/env bash
# Repetition complete en simulation, avec les garde-fous que la semaine a rendus
# necessaires. Chaque etape verifie son resultat au lieu de supposer : les quatre
# bugs qui ont coute les courses de cette semaine n'ont produit AUCUNE erreur.
#
# Usage :  ./sim.sh <seed>        une repetition complete
#          ./sim.sh <seed> check  s'arrete apres les verifications, sans courir
#          ./sim.sh course        court sur une pile deja en place
#          ./sim.sh etat          ou en est la pile
#          ./sim.sh stop          arrete tout

set -uo pipefail

WS=~/PAN/ros2_ws
LOGS=/tmp/parc-sim
SEED=${1:-12}

# ISOLATION RESEAU - non negociable.
#
# Ce PC est sur le Wi-Fi du robot (Flybox_ED67). Sans isolation, les noeuds de cette
# simulation rejoignent le graphe DDS du VRAI Sito-E : son /cmd_vel pourrait piloter
# le vrai robot, notre parc_referee publierait sur /parc/* hors creneau, et le pont
# Gazebo ecrirait sur /botwheel_explorer/odom - ce que l'organisation interdit
# explicitement.
#
# ROS_AUTOMATIC_DISCOVERY_RANGE et non ROS_LOCALHOST_ONLY : ce dernier est deprecie
# en Jazzy et fait precéder CHAQUE sortie de commande d'un avertissement rcl, ce qui
# a fausse plusieurs de mes propres analyses en se melant au texte analyse.
# Le domaine 42 est une seconde barriere, independante de la premiere.
ISO=(ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=42)

# Environnement propre : une pile ROS heritee d'un shell precedent a deja fait
# echouer un lancement sur "No package metadata was found for ros2cli".
rosenv() {
  env -i HOME="$HOME" PATH=/usr/bin:/bin:/usr/local/bin TERM=xterm "${ISO[@]}" \
      bash -lc "source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash && $1"
}

# setsid : la pile doit SURVIVRE a la fin de ce script. Sans lui les lancements sont
# des enfants du script et meurent avec lui - observe, slam_toolbox introuvable deux
# minutes apres un `check` reussi, ce qui ressemblait a un plantage de SLAM.
lancer_detache() {
  local nom=$1 cmd=$2
  # DISPLAY et XAUTHORITY doivent traverser `env -i`, sinon aucune fenetre ne peut
  # s'ouvrir en mode GUI et Gazebo echoue sur un "cannot open display" peu parlant.
  setsid env -i HOME="$HOME" PATH=/usr/bin:/bin:/usr/local/bin TERM=xterm \
    DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" \
    XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}" "${ISO[@]}" \
    bash -lc "source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash && exec $cmd" \
    > "$LOGS/$nom.log" 2>&1 < /dev/null &
}

# Attend un ETAT du systeme, jamais une ligne de journal.
#
# La version precedente grepait les journaux et validait sur les bannieres de
# demarrage : `apriltag` matchait "process started with pid", et les quatre
# verifications partaient 25 s trop tot, avant que SLAM ait publie map -> odom. Un
# journal dit ce qui a ete imprime ; seul l'etat dit ce qui est vrai.
attendre() {  # attendre <description> <limite_s> <commande-test>
  local quoi=$1 limite=$2 test=$3 t=0
  printf "    %-46s" "$quoi"
  while [ $t -lt "$limite" ]; do
    if eval "$test" >/dev/null 2>&1; then echo "ok (${t}s)"; return 0; fi
    sleep 5; t=$((t+5))
  done
  echo "ECHEC (${limite}s)"
  return 1
}

# Un topic publie-t-il vraiment ? `ros2 topic hz` emploie une QoS RELIABLE et reste
# muet devant un publieur BEST_EFFORT - son silence ne prouve rien. On compte donc
# les messages avec la bonne QoS.
compte_msgs() {  # compte_msgs <topic> <type python> <secondes>
  rosenv "python3 -c \"
import rclpy, time, importlib
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
mod, cls = '$2'.rsplit('.', 1)
T = getattr(importlib.import_module(mod), cls)
rclpy.init(); n = Node('cnt'); c = [0]
q = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
               reliability=ReliabilityPolicy.BEST_EFFORT,
               durability=DurabilityPolicy.VOLATILE)
n.create_subscription(T, '$1', lambda m: c.__setitem__(0, c[0]+1), q)
t = time.time()
while time.time() - t < $3: rclpy.spin_once(n, timeout_sec=0.2)
print(c[0])\"" 2>/dev/null | tail -1
}

# Tue, puis VERIFIE. Un arret suppose est pire qu'un arret rate.
#
# La version precedente faisait confiance a pkill et enchainait. L'ancienne Gazebo
# survivait, la nouvelle demarrait par-dessus, et /clock repartait en arriere :
# slam_toolbox, bt_navigator, planner_server et collision_monitor ont tous logue
# "Detected jump back in time. Clearing TF buffer." en boucle, et SLAM n'a jamais
# pu publier map -> odom. Le symptome - "SLAM publie map -> odom : ECHEC" - ne
# designait rien de ce qui etait reellement casse.
arreter() {
  local t=0
  for p in race_director apriltag_detection slam_navigation referee.launch phase2_hard; do
    pkill -f "$p" 2>/dev/null
  done
  sleep 3
  # `gz sim` est un script ruby : le processus reel est `ruby /usr/bin/gz sim`, et
  # c'est lui qui detient l'horloge de simulation.
  pkill -f 'gz sim' 2>/dev/null
  pkill -f 'ruby.*gz' 2>/dev/null
  # A survecu a son parent une fois, et tournait a 90% de CPU sur une camera morte.
  pkill -f apriltag_node 2>/dev/null
  pkill -f rectify_node 2>/dev/null
  pkill -f parameter_bridge 2>/dev/null

  # On n'avance pas tant qu'il reste un detenteur d'horloge.
  while [ $t -lt 40 ]; do
    pgrep -f 'gz sim|ruby.*gz|phase2_hard' >/dev/null 2>&1 || break
    sleep 2; t=$((t+2))
    [ $t -ge 20 ] && { pkill -9 -f 'gz sim' 2>/dev/null; pkill -9 -f 'ruby.*gz' 2>/dev/null; }
  done
  if pgrep -f 'gz sim|ruby.*gz' >/dev/null 2>&1; then
    echo "  ATTENTION : une Gazebo survit encore - l'horloge reculera. Arret." >&2
    pgrep -af 'gz sim|ruby.*gz' | head -3 | sed 's/^/    /' >&2
    return 1
  fi
  sleep 2
  rosenv "ros2 daemon stop" >/dev/null 2>&1
  echo "  tout arrete, aucune horloge residuelle"
}

# /clock doit avancer AVANT qu'un noeud en use_sim_time demarre. Sinon il demarre a
# zero, voit l'horloge sauter, et vide son tampon TF - repete, ca empeche toute
# transformee de se resoudre.
horloge_stable() {
  printf "    %-46s" "horloge de simulation stable"
  # Un seul abonne qui lit deux echantillons, au lieu de deux `ros2 topic echo
  # --once` successifs. Chaque echo doit redecouvrir le publieur, et sous charge il
  # rend la main VIDE avant d'y arriver : observe `a=2 b=vide`, ce qui a fait echouer
  # le garde et avorter toute la course alors que Gazebo tournait parfaitement. Une
  # mesure qui echoue quand la machine est chargee est inutile - c'est precisement
  # le moment ou on en a besoin.
  local mesure a b fr
  mesure=$(rosenv "python3 -c \"
import rclpy, time
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
rclpy.init(); n = Node('clk'); v = []
q = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
               reliability=ReliabilityPolicy.BEST_EFFORT,
               durability=DurabilityPolicy.VOLATILE)
n.create_subscription(Clock, '/clock', lambda m: v.append(m.clock.sec + m.clock.nanosec*1e-9), q)
t0 = time.time(); debut = None
while time.time() - t0 < 6:
    rclpy.spin_once(n, timeout_sec=0.2)
    if v and debut is None: debut = v[-1]
print(f'{debut:.2f} {v[-1]:.2f}' if v and debut is not None else '')\"" 2>/dev/null | tail -1)
  a=$(echo "$mesure" | awk '{print $1}')
  b=$(echo "$mesure" | awk '{print $2}')
  # Exiger que l'horloge AVANCE, pas seulement qu'elle ne recule pas.
  #
  # La version precedente acceptait b >= a, donc "19s -> 19s" passait pour bon. En
  # mode GUI sur cette machine, Gazebo s'arrete net : facteur temps reel mesure a
  # 0,00x, charge 15,67 sur 12 coeurs. Nav2 restait alors 180 s sur
  # "Waiting for service controller_server/get_state" et l'echec s'affichait sur
  # bt_navigator - trois minutes perdues a designer le mauvais coupable, alors que
  # l'horloge le disait en quatre secondes.
  if [ -z "${a:-}" ] || [ -z "${b:-}" ]; then
    echo "ECHEC (pas de /clock : a=${a:-vide} b=${b:-vide})"
    return 1
  fi
  # Fenetre de 6 s cote python, d'ou la division par 6.
  fr=$(python3 -c "print(f'{($b-$a)/6:.2f}')" 2>/dev/null)
  if python3 -c "import sys; sys.exit(0 if ($b-$a)/6 >= 0.2 else 1)" 2>/dev/null; then
    echo "ok (${a}s -> ${b}s, facteur ${fr}x)"
    return 0
  fi
  echo "ECHEC (facteur temps reel ${fr}x - simulation a l'arret)"
  [ "${GUI:-0}" = "1" ] && echo "      L'affichage Gazebo+RViz sature cette machine. Relancer sans GUI=1." >&2
  return 1
}

etat() {
  echo "=== noeuds de cycle de vie ==="
  for n in slam_toolbox bt_navigator planner_server controller_server; do
    printf "  %-20s " "$n"
    rosenv "timeout 10 ros2 lifecycle get /$n" 2>/dev/null | tail -1
  done
  echo "=== flux ==="
  printf "  %-28s %s msg/5s\n" "image_rect" "$(compte_msgs /camera1/camera1/color/image_rect sensor_msgs.msg.Image 5)"
  printf "  %-28s %s msg/5s\n" "camera_info" "$(compte_msgs /camera1/camera1/color/camera_info sensor_msgs.msg.CameraInfo 5)"
  printf "  %-28s %s msg/5s\n" "scan_filtered" "$(compte_msgs /scan_filtered sensor_msgs.msg.LaserScan 5)"
  echo "=== synchro du detecteur (derniere fenetre) ==="
  grep -a 'Synchronized pairs' "$LOGS/tags.log" 2>/dev/null | tail -1 | sed 's/^/  /' || echo "  (pas de plainte - bon signe)"
}

verdict() {
  echo
  echo "=== verdict de l'ARBITRE (pas le notre) ==="
  curl -s http://localhost:8080/api/state | python3 -c "
import json,sys
d=json.load(sys.stdin)
print(f\"  etat        : {d['state']}   chrono {d['elapsed_s']:.0f}/{d['time_limit_s']:.0f} s\")
print(f\"  faux depart : {d['false_start']}\")
print(f\"  visites     : {[v.get('tag_id') for v in d['visits']]}\")
s=d['score']
print(f\"  score       : {s['total']} (tags {s['tag_points']}, nav {s['safe_points']}, voix {s['voice_bonus']})\")
print(f\"  confirmees {s['confirmed']}, en attente {s['pending']}\")" 2>/dev/null \
    || echo "  (arbitre injoignable)"
}

courir() {
  # Le robot doit etre IMMOBILE avant l'armement : l'arbitre declare un faux depart
  # au moindre mouvement en etat ARMED, puis refuse de lancer son chronometre. C'est
  # ce qui a annule une course entiere - un but Nav2 de la course precedente roulait
  # encore au moment ou j'ai arme, et notre journal a quand meme annonce 25 points
  # pendant que l'arbitre comptait 0.
  attendre "robot immobile avant armement" 60 \
    "rosenv \"python3 -c '
import rclpy, time
from rclpy.node import Node
from nav_msgs.msg import Odometry
rclpy.init(); n=Node(\\\"imm\\\"); v=[]
n.create_subscription(Odometry,\\\"/botwheel_explorer/odom\\\",lambda m: v.append(abs(m.twist.twist.linear.x)+abs(m.twist.twist.angular.z)),10)
t=time.time()
while time.time()-t < 5: rclpy.spin_once(n, timeout_sec=0.2)
raise SystemExit(0 if v and max(v) < 0.02 else 1)'\"" || return 1

  curl -s -X POST http://localhost:8080/api/judge/reset -H 'X-Judge-Token: parc2026' \
       -H 'Content-Type: application/json' -d '{}' >/dev/null
  # ORDRE : l'arbitre tire un ordre au hasard a chaque armement, sauf si on lui en
  # impose un. Pour repeter l'ordre annonce pour la course : ORDRE="4,9,1,7" ./sim.sh
  if [ -n "${ORDRE:-}" ]; then
    corps="{\"team\":\"aroka_tech\",\"order\":\"$ORDRE\"}"
    echo "    ordre impose : $ORDRE"
  else
    corps='{"team":"aroka_tech"}'
  fi
  echo "    arme : $(curl -s -X POST http://localhost:8080/api/judge/arm \
       -H 'X-Judge-Token: parc2026' -H 'Content-Type: application/json' \
       -d "$corps")"
  echo
  rosenv "ros2 run parc_solution race_director.py" 2>&1 | tee "$LOGS/course.log"
  verdict
}

case "${1:-}" in
  stop)   arreter; exit 0 ;;
  etat)   etat; exit 0 ;;
  course) courir; exit $? ;;
esac

mkdir -p $LOGS
echo "=== 0/6  table rase ==="
arreter || exit 1
free -h | awk 'NR==2{printf "  memoire disponible : %s\n", $7}'
echo "  isolation : domaine 42, decouverte limitee a LOCALHOST (le vrai robot est hors de portee)"

echo "=== 1/6  arene, seed $SEED ==="
# Par defaut headless : la fenetre Gazebo et RViz affament la simulation sur cette
# machine - la camera est passee de 0,64 a 10,8 Hz le jour ou gui:=false a ete mis.
# GUI=1 pour regarder quand meme, en sachant que la course sera moins
# representative : ce qu'on observe alors est une machine saturee, pas la solution.
GUI=${GUI:-0}
if [ "$GUI" = "1" ]; then
  echo "    AFFICHAGE ACTIF - Gazebo + RViz. Simulation nettement ralentie."
  lancer_detache arene "ros2 launch parc_robot_bringup phase2_hard.launch.py seed:=$SEED gui:=true rviz:=true"
else
  lancer_detache arene "ros2 launch parc_robot_bringup phase2_hard.launch.py seed:=$SEED gui:=false rviz:=false"
fi
attendre "lidar du robot publie" 240 \
  "[ \$(compte_msgs /scan_filtered sensor_msgs.msg.LaserScan 4) -gt 0 ]" || exit 1
attendre "camera du robot publie" 120 \
  "[ \$(compte_msgs /camera1/camera1/color/image_raw sensor_msgs.msg.Image 4) -gt 0 ]" || exit 1
horloge_stable || exit 1

echo "=== 2/6  SLAM + Nav2 ==="
# Les defauts du lancement servent desormais le ROBOT REEL : base_link, et le relais
# TwistStamped vers /botwheel_explorer/cmd_vel. La simulation doit donc se declarer.
# Son sitoe_core.xacro joint base_footprint a base_link avec rpy="0 0 pi/2" et le
# plugin DiffDrive pilote en base_footprint ; sur base_link, MPPI raisonne sur un axe
# transversal et la conduite devient erratique.
lancer_detache slam "ros2 launch parc_solution slam_navigation.launch.py \
  nav2_params_file:=$WS/install/parc_solution/share/parc_solution/config/nav2_params.yaml \
  slam_params_file:=$WS/install/parc_solution/share/parc_solution/config/slam_params.yaml \
  nav2_delay_s:=35.0"
# `grep -q active` matchait "inactive". Le garde validait donc un noeud jamais
# demarre : slam_toolbox bloque en configuration, journal arrete sur
# "failed to send response to /slam_toolbox/change_state (timeout)", et l'echec
# tombait 120 s plus tard sur map -> odom en accusant SLAM. On exige l'etat exact.
attendre "slam_toolbox actif" 180 "rosenv 'timeout 10 ros2 lifecycle get /slam_toolbox' | grep -qE '^active '" || exit 1
attendre "bt_navigator actif"  240 "rosenv 'timeout 10 ros2 lifecycle get /bt_navigator'  | grep -qE '^active '" || exit 1
attendre "SLAM publie map -> odom" 120 \
  "rosenv 'timeout 15 ros2 run tf2_ros tf2_echo map odom' | grep -q Translation" || exit 1

echo "=== 3/6  detection AprilTag ==="
# use_sim_time:=true : son defaut est false, correct pour le robot reel qui n'a pas
# de /clock, et faux ici ou Gazebo en publie un.
lancer_detache tags "ros2 launch parc_solution apriltag_detection.launch.py use_sim_time:=true tag_size:=0.12 camera:=camera1"
attendre "image rectifiee publiee" 120 \
  "[ \$(compte_msgs /camera1/camera1/color/image_rect sensor_msgs.msg.Image 4) -gt 0 ]" || exit 1

echo "=== 4/6  arbitre ==="
lancer_detache arbitre "ros2 launch parc_referee referee.launch.py odom_topic:=/botwheel_explorer/odom"
attendre "arbitre repond sur 8080" 120 "curl -sf http://localhost:8080/api/state" || exit 1

echo "=== 5/6  les verifications d'avant-course ==="
ECHEC=0

# Le dernier maillon de la chaine de vitesse. Zero abonne = le robot ne bouge pas,
# et le symptome imite exactement une machine saturee : buts ABORTED et
# "Timed out ... compute_path_to_pose" alors que le planificateur va tres bien.
printf "    %-46s" "/cmd_vel a un abonne"
n=$(rosenv "timeout 25 ros2 topic info /cmd_vel" 2>/dev/null | grep -oP 'Subscription count: \K[0-9]+')
[ "${n:-0}" -ge 1 ] && echo "ok ($n)" || { echo "ECHEC (0 abonne)"; ECHEC=1; }

# Un seul publieur, sinon deux piles se battent pour les roues.
printf "    %-46s" "/cmd_vel n'a qu'un publieur"
p=$(rosenv "timeout 25 ros2 topic info /cmd_vel" 2>/dev/null | grep -oP 'Publisher count: \K[0-9]+')
[ "${p:-0}" -le 1 ] && echo "ok ($p)" || { echo "ECHEC ($p publieurs)"; ECHEC=1; }

# image_transport deduit camera_info du chemin de l'image et ignore un remappage
# explicite. Mal place, le detecteur ne recoit jamais d'intrinseques et ne detecte
# RIEN - sans erreur, et le symptome en amont est "tag introuvable".
# On lit la DERNIERE fenetre, pas un compte d'occurrences : une plainte au demarrage
# est normale, une plainte persistante est fatale.
printf "    %-46s" "image et camera_info apparies"
sleep 12
d=$(grep -a 'Synchronized pairs' "$LOGS/tags.log" 2>/dev/null | tail -1 | grep -oE '[0-9]+$')
if [ -z "${d:-}" ]; then echo "ok (aucune plainte)"
elif [ "$d" -gt 0 ]; then echo "ok ($d paires)"
else echo "ECHEC (0 paire - aucune detection possible)"; ECHEC=1; fi

# Le planificateur repond-il ? Planifie seulement, ne bouge pas : ne risque donc
# pas de declencher un faux depart avant l'armement.
printf "    %-46s" "planner_server repond"
rosenv "timeout 45 ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose '{goal: {header: {frame_id: \"map\"}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}, use_start: false, planner_id: \"GridBased\"}'" 2>/dev/null \
  | grep -q SUCCEEDED && echo "ok" || { echo "ECHEC"; ECHEC=1; }

# Les cadences reelles : c'est ce qui dit si la machine tient la pile complete.
echo "    cadences (msg/5s) :"
printf "      %-26s %s\n" "scan_filtered" "$(compte_msgs /scan_filtered sensor_msgs.msg.LaserScan 5)"
printf "      %-26s %s\n" "image_rect"    "$(compte_msgs /camera1/camera1/color/image_rect sensor_msgs.msg.Image 5)"

[ "$ECHEC" -eq 1 ] && { echo; echo "  Verifications en echec - courir maintenant ne prouverait rien."; exit 1; }
[ "${2:-}" = "check" ] && { echo; echo "  Tout vert. La pile reste en place ; courir avec : ./sim.sh course"; exit 0; }

echo "=== 6/6  course ==="
courir
