# Build l'ensemble des paquets et des noeuds :
build:
	colcon build --merge-install

# -------------------------------------------------------- #
# Lance le programme en mode competition :
run:
	ros2 launch aquabot_gz competition.launch.py world:=aquabot_regatta

# Lance les differents niveaux de difficulte :
easy:
	ros2 launch aquabot_gz competition.launch.py world:=aquabot_windturbines_easy competition_mode:=true

medium:
	ros2 launch aquabot_gz competition.launch.py world:=aquabot_windturbines_medium competition_mode:=true

hard:
	ros2 launch aquabot_gz competition.launch.py world:=aquabot_windturbines_hard competition_mode:=true

# -------------------------------------------------------- #
# Lance le Node des commandes de deplacement :
teleop:
	ros2 run aquabot_python teleop_keyboard.py

# Lance le Node navigation :
navigation:
	ros2 run navigation navigation_node

# Lance le Node data :
data: 
	ros2 run data data_node

# Lance le Node qr_code :
qr_code:
	ros2 run qr_code qr_code_node

# -------------------------------------------------------- #
# Lance les differents scenario aleatoires :
00:
	ros2 launch aquabot_gz competition.launch.py world:=aquabot_windturbines_competition_00 

01:
	ros2 launch aquabot_gz competition.launch.py world:=aquabot_windturbines_competition_01 

02:
	ros2 launch aquabot_gz competition.launch.py world:=aquabot_windturbines_competition_02

03:
	ros2 launch aquabot_gz competition.launch.py world:=aquabot_windturbines_competition_03

04:
	ros2 launch aquabot_gz competition.launch.py world:=aquabot_windturbines_competition_04

05:
	ros2 launch aquabot_gz competition.launch.py world:=aquabot_windturbines_competition_05

06:
	ros2 launch aquabot_gz competition.launch.py world:=aquabot_windturbines_competition_06

07:
	ros2 launch aquabot_gz competition.launch.py world:=aquabot_windturbines_competition_07

08:
	ros2 launch aquabot_gz competition.launch.py world:=aquabot_windturbines_competition_08

09:
	ros2 launch aquabot_gz competition.launch.py world:=aquabot_windturbines_competition_09
