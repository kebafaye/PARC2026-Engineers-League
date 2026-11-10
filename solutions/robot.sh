#!/usr/bin/env bash
# Pilote la pile PARC qui tourne SUR LE ROBOT, depuis ce PC, par SSH.
#
# Les noeuds tournent sur le Jetson, jamais ici. Le lien Wi-Fi mesure 2,2 Ko/s et
# 431 ms de latence : il ne peut pas porter une boucle de controle a 20 Hz, et encore
# moins un flux camera. Ce script n'envoie que des commandes et ne ramene que du
# texte de journal.
#
# Chaque lancement part avec setsid et sa sortie dans un fichier, pour que la chute
# de la session SSH - previsible sur ce lien - ne tue pas la course.
#
# Usage :  ./robot.sh build | slam | tags | check | course | logs <nom> | stop | noeuds

set -uo pipefail

# L IP du robot change d une session a l autre (vue en .126 puis .3). Surchargeable :
#   ROBOT=sitoe@192.168.1.3 ./robot.sh check
ROBOT=${ROBOT:-sitoe@192.168.1.3}
DIR=${DIR:-\~/teams/aroka_tech}

# -t force un pseudo-terminal : sans lui, ros2 launch n'affiche rien en direct.
# ControlMaster reutilise une seule connexion TCP pour toutes les commandes du
# script, ce qui evite de repayer la poignee de main SSH (plusieurs secondes ici)
# a chaque appel.
# -n : stdin depuis /dev/null. Sans lui, ssh garde le terminal ouvert et peut rester
# suspendu derriere un processus lance en arriere-plan sur le robot.
SSH=(ssh -n -o ControlMaster=auto -o ControlPath=/tmp/parc-ssh-%r@%h -o ControlPersist=120
     -o ServerAliveInterval=15 "$ROBOT")

# Tout ce qui tourne la-bas a besoin des deux workspaces : celui de CAYTU pour
# parc_msgs et parc_robot_bringup, le notre pour parc_solution.
ENV="source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && cd $DIR && source install/setup.bash"

# timeout sur TOUT appel distant. Un script de creneau qui se suspend sans rien
# afficher coute plus cher qu'une commande qui echoue franchement.
#
# La commande part encodee en base64 au lieu d'etre glissee dans `bash -lc '...'`.
# Cette enveloppe en apostrophes cassait sur toute charge contenant elle-meme une
# apostrophe : le `ps ... | awk '{printf ...}'` de `qui` est mort sur
# `syntax error near unexpected token '('` et n'a donc jamais dit a qui
# appartenaient les noeuds - precisement l'information qu'on lui demandait avant
# d'arreter quoi que ce soit sur une machine partagee. En base64 il n'y a plus de
# caractere a echapper, quelle que soit la commande.
distant() {
  local charge
  charge=$(printf %s "$1" | base64 -w0)
  timeout "${2:-45}" "${SSH[@]}" "echo $charge | base64 -d | bash -l"
}

# Lance et oublie, sans jamais suspendre l'appelant.
#
# `ssh host "cmd &"` ne rend PAS la main de facon fiable : le processus en
# arriere-plan garde le canal ouvert, et avec ControlMaster le multiplexage attend
# indefiniment. Observe en creneau : `./robot.sh slam` est reste muet sans fin.
#
# On passe donc par un script jetable deposbe sur le robot, demarre par setsid dans
# sa propre session, et le shell distant se termine explicitement par `exit 0`.
detacher() {
  local nom=$1 cmd=$2
  distant "mkdir -p $DIR && cat > $DIR/.run_$nom.sh <<\"EOS\"
$ENV
exec $cmd
EOS
chmod +x $DIR/.run_$nom.sh" 30 || return 1
  distant "setsid $DIR/.run_$nom.sh > $DIR/$nom.log 2>&1 < /dev/null & exit 0" 20
}

# Refuse d'aller plus loin si le code n'est pas sur le robot. Sans ce garde, chaque
# commande echouait sur un `cd` et le script continuait comme si de rien n'etait.
exige_deploiement() {
  if ! distant "test -f $DIR/install/setup.bash" 2>/dev/null; then
    echo "ERREUR : $DIR n'est pas compile sur le robot." >&2
    echo "         Lancer d'abord :  ./robot.sh deploy" >&2
    exit 1
  fi
}

# Detache le processus de la session SSH. Sans setsid, raccrocher tue le noeud.
#
# La premiere version de cette fonction affichait `lance` quoi qu'il arrive : le `&`
# mettait en arriere-plan toute la chaine `&&`, donc un `cd` echoue n'empechait pas
# le `echo`. Resultat observe : trois `lance` de suite alors que le dossier n'existait
# meme pas sur le robot. On verifie donc que le processus tourne VRAIMENT, et on rend
# la main en erreur sinon - c'est tout l'interet d'un lanceur.
lancer() {
  local nom=$1 cmd=$2
  echo "-> $nom sur le robot (journal : $DIR/$nom.log)"
  detacher "$nom" "$cmd" || { echo "   ECHEC : impossible de deposer le lanceur." >&2; return 1; }
  sleep 8
  if distant "pgrep -f '.run_$nom.sh' > /dev/null" 20; then
    echo "   OK, tourne. Suivre avec :  ./robot.sh logs $nom"
  else
    echo "   ECHEC : rien ne tourne. Dernieres lignes du journal :" >&2
    distant "tail -15 $DIR/$nom.log 2>/dev/null || echo '(aucun journal - la commande n a pas demarre)'" >&2
    return 1
  fi
}

case "${1:-}" in
  deploy)
    # Tout le trajet d'un coup : archive, envoi, decompression, compilation. Ecrit
    # pour qu'il n'y ait plus rien a taper sur le robot - les deux moities de la
    # procedure se sont croisees une fois (commandes du PC tapees sur le robot et
    # inversement), et le code n'est jamais arrive.
    ZIP=/tmp/aroka_tech.zip
    STAGE=$(mktemp -d)
    echo "=== 1/4  archive (parc_solution seul) ==="
    # Seulement notre paquet : src/ contient aussi parc_msgs, parc_referee et
    # parc_robot_bringup, dont des copies compilees ici masqueraient par surcouche
    # celles de ~/ros2_ws du robot. Un parc_msgs aux hash differents rend les cinq
    # topics /parc/ muets et l'arbitre ne peut plus nous parler.
    mkdir -p "$STAGE/src"
    cp -r ~/PAN/ros2_ws/src/solutions/parc_solution "$STAGE/src/"
    mkdir -p "$STAGE/src/parc_solution/outils"
    cp ~/PAN/ros2_ws/src/solutions/DEPLOY.md "$STAGE/src/parc_solution/"
    for f in calibrate_tag_size.py check_camera_rates.py grab_frame.py \
             tag_angle_sweep.py generate_tag.py README.md; do
      [ -f ~/PAN/ros2_ws/src/solutions/test_assets/$f ] &&
        cp ~/PAN/ros2_ws/src/solutions/test_assets/$f "$STAGE/src/parc_solution/outils/"
    done
    ( cd "$STAGE" &&
      find . \( -name build -o -name install -o -name log -o -name __pycache__ \
                -o -name '*.pyc' \) -prune -exec rm -rf {} + 2>/dev/null
      rm -f "$ZIP" && zip -rq "$ZIP" src )
    rm -rf "$STAGE"
    SOMME=$(sha256sum "$ZIP" | cut -c1-16)
    echo "    $ZIP  $(du -h "$ZIP" | cut -f1)  sha256 $SOMME"

    echo "=== 2/4  envoi (lent : ~2 Ko/s sur ce Wi-Fi) ==="
    scp -o ControlPath=/tmp/parc-ssh-%r@%h "$ZIP" "$ROBOT:~/" || exit 1

    echo "=== 3/4  verification de l'integrite sur le robot ==="
    DISTANTE=$(distant "sha256sum ~/aroka_tech.zip | cut -c1-16")
    if [ "$DISTANTE" != "$SOMME" ]; then
      echo "ERREUR : archive tronquee a l'envoi ($DISTANTE != $SOMME). Relancer." >&2
      exit 1
    fi
    echo "    intacte"

    echo "=== 4/4  decompression et compilation sur le Jetson ==="
    distant "mkdir -p $DIR && cd $DIR && rm -rf src && unzip -q ~/aroka_tech.zip && \
             source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && \
             colcon build" || exit 1
    echo
    echo "Pret. Suite :  ./robot.sh slam"
    ;;

  build)
    distant "source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && cd $DIR && colcon build"
    ;;

  slam)
    exige_deploiement
    lancer slam "ros2 launch parc_solution slam_navigation.launch.py use_sim_time:=False"
    ;;

  tags)
    exige_deploiement
    # Pas de use_sim_time : son defaut est deja false, ce qu'il faut sans /clock.
    lancer tags "ros2 launch parc_solution apriltag_detection.launch.py"
    ;;

  check)
    exige_deploiement
    # Les trois choses a verifier avant toute course. Chacune a deja coute une
    # course entiere cette semaine, et aucune ne produit d'erreur quand elle echoue.
    echo "=== 1. le dernier maillon a-t-il un abonne ? (0 = le robot ne bougera pas) ==="
    distant "$ENV && timeout 20 ros2 topic info /cmd_vel"
    echo
    echo "=== 2. SLAM publie-t-il map -> odom ? ==="
    distant "$ENV && timeout 15 ros2 run tf2_ros tf2_echo map odom 2>&1 | head -4"
    echo
    echo "=== 3. le detecteur apparie-t-il image et intrinseques ? ==="
    echo "    (chercher 'Synchronized pairs: 0' = aucune detection possible)"
    distant "grep -c 'Synchronized pairs:  *0' $DIR/tags.log 2>/dev/null || echo 0"
    echo
    echo "=== cadences reelles sur le Jetson ==="
    distant "$ENV && for t in /scan_filtered /camera1/camera1/color/image_raw; do printf '  %-40s ' \$t; timeout 8 ros2 topic hz \$t 2>&1 | grep -m1 average || echo '(rien)'; done"
    ;;

  course)
    exige_deploiement
    # Volontairement au premier plan : une course se surveille. Si le SSH tombe,
    # relancer avec `logs course` pour retrouver le fil.
    echo "Course en direct. Ctrl-C arrete l'affichage ET la course."
    distant "$ENV && ros2 run parc_solution race_director.py 2>&1 | tee $DIR/course.log"
    ;;

  logs)
    distant "tail -f $DIR/${2:-slam}.log"
    ;;

  noeuds)
    # Pour reperer les noeuds d'une autre equipe avant de toucher a quoi que ce soit.
    distant "$ENV && timeout 20 ros2 node list"
    ;;

  qui)
    # A lancer AVANT tout `stop`. Le robot est partage : un `pkill` par ligne de
    # commande toucherait aussi les noeuds d'une autre equipe ayant deploye le meme
    # paquet. Observe en creneau : race_director, tag_explorer et tag_goal_bridge
    # tournaient alors que nous ne les avions pas lances, et slam_toolbox etait
    # present en double.
    echo "=== sessions ouvertes sur le robot ==="
    distant "who" 30
    echo
    echo "=== qui a lance quoi, et depuis quand ==="
    echo "    (etimes = age en secondes ; plus vieux que votre creneau = pas a vous)"
    distant "ps -eo etimes,user,pid,args --sort=-etimes | grep -E 'slam_toolbox|nav2|bt_navigator|race_director|tag_explorer|tag_goal|apriltag|parc_solution|speed_governor' | grep -v grep | cut -c1-150" 40
    echo
    echo "=== d'ou viennent-ils ? (teams/ de chaque processus) ==="
    distant "for p in \$(pgrep -f 'race_director|tag_explorer|slam_toolbox|bt_navigator'); do printf '  pid %-7s ' \$p; tr \"\\0\" \" \" < /proc/\$p/environ 2>/dev/null | grep -o 'teams/[a-zA-Z0-9_-]*' | head -1 || echo '(indetermine)'; done" 40
    echo
    echo "=== nos propres lanceurs (deposes par ce script) ==="
    distant "ls -l $DIR/.run_*.sh 2>/dev/null || echo '  aucun - donc rien ne vient de nous'" 20
    echo
    echo "Lire 'depuis Ns' : un noeud plus vieux que votre creneau n'est pas a vous."
    ;;

  stop)
    # Cible nos propres lancements par leur ligne de commande, et rien d'autre :
    # le robot est partage et un pkill large couperait le travail d'une autre equipe.
    echo "Arret de nos noeuds (slam_navigation, apriltag_detection, race_director)."
    distant "pkill -f '.run_slam.sh'; pkill -f '.run_tags.sh'; \
             pkill -f 'slam_navigation.launch.py'; pkill -f 'apriltag_detection.launch.py'; \
             pkill -f 'race_director.py'; sleep 2; echo fait" 30
    ;;

  *)
    cat <<'AIDE'
Usage : ./robot.sh <commande>

  deploy    envoie le code et le compile sur le robot - A FAIRE EN PREMIER
  build     recompile sur le robot (code deja envoye)
  slam      lance SLAM + Nav2            (detache, journal slam.log)
  tags      lance la detection AprilTag  (detache, journal tags.log)
  check     les trois verifications d'avant-course, plus les cadences reelles
  course    lance la course, en direct
  logs <n>  suit un journal : slam, tags, course
  noeuds    liste les noeuds ROS du robot (reperer ceux d'une autre equipe)
  qui       proprietaire et age de chaque noeud - A LANCER AVANT TOUT stop
  stop      arrete nos trois lancements, et seulement les notres

Ordre d'un creneau :  deploy -> slam -> tags -> check -> course

Tout tourne sur le robot. Ce script n'envoie que des commandes : le lien mesure
2,2 Ko/s et 431 ms, il ne peut porter ni la boucle Nav2 ni un flux camera.
AIDE
    ;;
esac
