# Ros 2

- [Index](/README.md)
-----------------------------------------------------------------------------

## Historique

ROS 2 repose sur la notion de **combinaison d'espaces de travail à l'aide de l'environnement de shell**. « Workspace » est un terme ROS pour l'emplacement sur votre système où vous développez avec ROS 2. L'espace de travail principal ROS 2 est appelé la sous-couche. Les espaces de travail locaux suivants sont appelés superpositions. Lorsque vous développez avec ROS 2, vous aurez généralement plusieurs espaces de travail actifs simultanément.

La combinaison d'espaces de travail facilite le développement contre différentes versions de ROS 2, ou contre différents ensembles de paquets. Elle permet également l'installation de plusieurs distributions ROS 2 (ou "distros", e.g. Dashing et Eloquent) sur le même ordinateur et basculant entre eux.

Cela se fait en approvisionnant des fichiers de configuration chaque fois que vous ouvrez un nouveau shell, ou en ajoutant la commande source à votre script de démarrage shell une fois. Sans trouver les fichiers de configuration, vous ne serez pas en mesure d'accéder aux commandes ROS 2, ni de trouver ou d'utiliser des paquets ROS 2. En d'autres termes, vous ne pourrez pas utiliser ROS 2.


-----------------------------------------------------------------------------
## Installation

### Variables locales 

Assurez-vous que vous avez une ***locale qui supporte UTF-8***. Si vous êtes dans un environnement minimal (comme un conteneur docker), la locale peut être quelque chose de minimal comme POSIX. Nous testons avec les paramètres suivants. Cependant, cela devrait fonctionner si vous utilisez une locale différente supportant l'UTF-8.

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

### Configuration de l'application Sources 

Vous devrez ajouter le dépôt ROS 2 apt à votre système :

> Assurez-vous d'abord que le dépôt Ubuntu Universe est activé.

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Ajoutez maintenant la ***clé GPG de ROS 2*** avec apt :

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Ajoutez ensuite le dépôt à votre liste de sources :

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Installer les paquets ROS 2

Mettre à jour vos caches de dépôts apt après avoir configuré les dépôts :

```bash
sudo apt update
```

Les paquets ROS 2 sont construits sur des systèmes Ubuntu fréquemment mis à jour. 
- Il est toujours recommandé de s'assurer que votre système est à jour avant d'installer de nouveaux paquets.

```bash
sudo apt upgrade
```

### Desktop Installation

```bash
sudo apt install ros-humble-desktop
```

### Vérification de l'installation

#### Talker-listener
Si vous avez installé ```ros-humble-desktop``` ci-dessus, vous pouvez essayer quelques exemples.

Dans un terminal, sourcez le fichier d'installation et exécutez ensuite un C++ ```talker``` :

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

Dans un autre terminal, sourcez le fichier d'installation, puis exécutez un  Python ```listener``` :

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

Vous devriez voir le locuteur dire qu'il publie des messages et l'auditeur dire que j'ai entendu ces messages. **Cela permet de vérifier que les API C++ et Python fonctionnent correctement.**

-----------------------------------------------------------------------------

## Lancement de l’environnement

Une fois que l’environnement est bien installé, il faudra à chaque fois taper les 2
commandes suivantes afin de pouvoir utiliser les commandes ROS2, et exécuter les
commandes liées au colcon workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/vrx_ws/install/setup.bash
```

Il est aussi possible de les lancer automatiquement à l’ouverture d’un nouveau terminal
en les ajoutant au fichier ```~/.bashrc```. 

- Partage les ajouts au fichier "bashrc" :

```bash
gedit ~/.bashrc
```

- L'ajout de ces lignes permet de disposer de toutes les sources d'approvisionnement lors du démarrage d'un nouveau terminal :

```bash
# Source ROS Humble
source /opt/ros/humble/setup.bash
# Source VRX Workspace
source ~/vrx_ws/install/setup.bash
```

- Ajouter ces lignes pour ajouter 3 alias à build, source et code pour le projet :

```bash
# Add Alias
alias aquabot_build='cd ~/vrx_ws && colcon build --merge-install'
alias aquabot_source='source ~/vrx_ws/install/setup.bash'
alias aquabot_code='code ~/vrx_ws/src/aquabot_competitor'
```

Pour lancer un environnement de simulation il est possible de lancer le fichier « launch »
competition.launch.py du package aquabot_gz.

```bash
ros2 launch aquabot_gz competition.launch.py world:=aquabot_regatta
```

Il est possible de déplacer le bateau à l’aide d’un noeud appelé teleop_keboard.py :

```bash
ros2 run aquabot_python teleop_keyboard.py
```

-----------------------------------------------------------------------------

## Commandes

Vous pouvez voir les nœuds et les rubriques, services et actions qui leur sont associés en utilisant les sous-commandes ```list``` des commandes respectives :

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

- Remap les commandes de l'objet 2 avec celle de l'objet 1 :
```bash
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```


### Coordonnees GPS 

```bash
> ros2 topic echo /aquabot/sensors/gps/gps/fix 
---
header:
  stamp:
    sec: 5
    nanosec: 652000000
  frame_id: aquabot/wamv/gps_wamv_link/navsat
status:
  status: 0
  service: 0
latitude: 48.046299920515615
longitude: -4.976320095040412
altitude: 0.7107898043468595
position_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
position_covariance_type: 0
```

Pour recuperer le type de la variable et l'utiliser dans le CPP :

```bash
> ros2 topic info /aquabot/sensors/gps/gps/fix 
Type: sensor_msgs/msg/NavSatFix
Publisher count: 1
Subscription count: 0
```

```cpp
// node.cpp

pos_gps = this->create_publisher<sensor_msgs::msg::NavSatFix>("/aquabot/sensors/gps/gps/fix", 10);
```

Pour connaitre la structure utilisee :

```bash
> ros2 interface show sensor_msgs/msg/NavSatFix
# Fixation du satellite de navigation pour tout système global de 
#   navigation par satellite
#
# Spécifié en utilisant l'ellipsoïde de référence WGS 84

# header.stamp spécifie l'heure ROS pour cette mesure (l'heure 
#   satellite correspondante peut être signalée à l'aide du message
#   sensor_msgs/TimeReference).

# header.frame_id est le cadre de référence rapporté par le récepteur 
#   satellite, généralement l'emplacement de l'antenne.
#   Il s'agit d'un Euclidien par rapport au véhicule, 
#   et non un ellipsoïde de référence.
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

# Satellite fix status information.
NavSatStatus status
	#
	int8 STATUS_NO_FIX =  -1        #
	int8 STATUS_FIX =      0        #
	int8 STATUS_SBAS_FIX = 1        #
	int8 STATUS_GBAS_FIX = 2        #
	int8 status
	uint16 SERVICE_GPS =     1
	uint16 SERVICE_GLONASS = 2
	uint16 SERVICE_COMPASS = 4      #
	uint16 SERVICE_GALILEO = 8
	uint16 service

# Latitude [degrés]. Les valeurs positives correspondent 
#   au nord de l'équateur et les valeurs négatives au sud.
float64 latitude

# Longitude [degrés]. La valeur positive correspond à l'est du méridien 
# d'origine, la valeur négative correspond à l'ouest.
float64 longitude

# Altitude [m]. Positif est au-dessus de l'ellipsoïde WGS 84
# (NaN silencieux si aucune altitude n'est disponible).
float64 altitude

# Position covariance [m^2] ddéfini par rapport à un plan tangentiel
# passant par la position déclarée. Les composantes sont l'Est, le Nord 
# et Up (ENU), dans l'ordre des lignes majeures.
#
# Attention : ce système de coordonnées présente des singularités aux pôles.
float64[9] position_covariance

# Si la covariance de la fixation est connue, remplissez-la complètement. 
# Si le récepteur GPS fournit la variance de chaque mesure, placez-les
# le long de la diagonale. Si seule la dilution de la précision est disponible,
# estimer une covariance approximative à partir de celle-ci.

uint8 COVARIANCE_TYPE_UNKNOWN = 0
uint8 COVARIANCE_TYPE_APPROXIMATED = 1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
uint8 COVARIANCE_TYPE_KNOWN = 3

uint8 position_covariance_type
```

-----------------------------------------------------------------------------

### Les nodes

Chaque nœud dans ROS devrait être responsable d'un **objectif unique et modulaire**, par exemple contrôler les moteurs des roues ou publier les données des capteurs d'un télémètre laser.

- Chaque nœud peut envoyer et recevoir des données d'autres nœuds via des sujets, des services, des actions ou des paramètres.

- Un système robotique complet est composé de nombreux nœuds travaillant de concert.

> [!IMPORTANT]
> Dans ROS 2, un seul exécutable (programme C++, programme Python, etc.) peut contenir un ou plusieurs nœuds.

- ```ros2 node list``` montre les noms de tous les noeuds en cours d'exécution :

```bash
> ros2 node list
/aquabot/frame_publisher
/aquabot/optical_frame_publisher
/aquabot/robot_state_publisher
/aquabot/ros_gz_bridge
/ros_gz_bridge
/teleop_keyboard
```

Ceci est particulièrement utile lorsque vous voulez interagir avec un noeud, ou lorsque vous avez un système avec de nombreux noeuds et que vous avez besoin de les suivre.


- ```ros2 node info``` renvoie une liste d'abonnés, d'éditeurs, de services et d'actions, c'est-à-dire les connexions du graphe ROS qui interagissent avec ce nœud :

```bash
ros2 node info <node_name>
```

```bash
> ros2 node info /teleop_keyboard
/teleop_keyboard
  Subscribers:

  Publishers:
    /aquabot/thrusters/left/pos: std_msgs/msg/Float64
    /aquabot/thrusters/left/thrust: std_msgs/msg/Float64
    /aquabot/thrusters/right/pos: std_msgs/msg/Float64
    /aquabot/thrusters/right/thrust: std_msgs/msg/Float64
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /teleop_keyboard/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /teleop_keyboard/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /teleop_keyboard/get_parameters: rcl_interfaces/srv/GetParameters
    /teleop_keyboard/list_parameters: rcl_interfaces/srv/ListParameters
    /teleop_keyboard/set_parameters: rcl_interfaces/srv/SetParameters
    /teleop_keyboard/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
```

> [!NOTE]
> Un nœud est un élément fondamental de ROS 2 qui sert un objectif unique et modulaire dans un système robotique.

-----------------------------------------------------------------------------

### Les Topic

ROS 2 décompose les systèmes complexes en de nombreux nœuds modulaires. ***Les topic*** sont un élément essentiel du graphe ROS, qui ***sert de bus aux nœuds pour l'échange de messages***.

Un nœud peut publier des données dans un nombre quelconque de topic et avoir simultanément des abonnements à un nombre quelconque de topic.

- ```ros2 topic list``` permet d'obtenir une liste de tous les sujets actuellement actifs dans le système :

```bash
> ros2 topic list
/aquabot/joint_states
/aquabot/pose
/aquabot/pose_static
/aquabot/robot_description
/aquabot/sensors/acoustics/receiver/range_bearing
...
```

- ```ros2 topic list -t``` rajoute le type de sujet entre parenthèses :

```bash
> ros2 topic list -t
/aquabot/joint_states [sensor_msgs/msg/JointState]
/aquabot/pose [tf2_msgs/msg/TFMessage]
/aquabot/pose_static [tf2_msgs/msg/TFMessage]
/aquabot/robot_description [std_msgs/msg/String]
/aquabot/sensors/acoustics/receiver/range_bearing [ros_gz_interfaces/msg/ParamVec]
...
```

Ces attributs, en particulier le type, permettent aux nœuds de savoir qu'ils parlent de la même information lorsqu'elle passe d'un topic à l'autre.


- Pour voir les données publiées sur un topic ***en temps reel***, utiliser :

```bash
ros2 topic echo <topic_name>
```

```bash
> ros2 topic echo /aquabot/pose
- header:
    stamp:
      sec: 11672
      nanosec: 348000000
    frame_id: aquabot/wamv/main_camera_post_link
  child_frame_id: aquabot/wamv/main_camera_post_link/wamv/main_camera_post_link_fixed_joint_lump__main_camera_post_arm_collision_collision_1
  transform:
    translation:
      x: 0.03
      y: 0.0
      z: 0.3382500000000001
    rotation:
      x: -0.4005096910359034
      y: 0.40050969103590356
      z: -0.5827452165280512
      w: 0.582745216528051
...
```

- ```ros2 topic info``` permet d'avoir le type du topic :

```bash
ros2 topic info <topic_name>
```

```bash
> ros2 topic info /aquabot/thrusters/right/thrust
Type: std_msgs/msg/Float64
Publisher count: 1
Subscription count: 1
```

Les nœuds envoient des données sur les thèmes à l'aide de messages. Les éditeurs et les abonnés doivent envoyer et recevoir l**e même type de message pour communiquer**.

Les types de sujets que nous avons vus plus tôt après avoir exécuté ```ros2 topic list -t``` permettent de savoir quel type de message est utilisé pour chaque sujet.

- ```ros2 interface show``` nous permet de connaitre la structure de donnees que le message attend :

```bash
> ros2 interface show std_msgs/msg/Float64
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

float64 data
```

-----------------------------------------------------------------------------

### Les services

Les services sont une autre méthode de communication pour les nœuds dans le graphe ROS. Les services sont basés sur ***un modèle d'appel et de réponse***, contrairement au modèle éditeur-souscripteur des sujets. 
Alors que les sujets permettent aux nœuds de s'abonner à des flux de données et d'obtenir des mises à jour continues, les services ne fournissent des données que lorsqu'ils sont spécifiquement appelés par un client.

- Pour lister l'ensemble des services actifs du systeme :

```bash
> ros2 service list
/teleop_keyboard/describe_parameters
/teleop_keyboard/get_parameter_types
/teleop_keyboard/get_parameters
/teleop_keyboard/list_parameters
/teleop_keyboard/set_parameters
/teleop_keyboard/set_parameters_atomically
...
```

Presque tous les nœuds de ROS 2 ont ces services d'infrastructure à partir desquels les paramètres sont construits


- Pour voir les types de tous les services actifs en même temps, vous pouvez ajouter l'option ```--show-types```, abrégée en ```-t```, à la commande list :

 ```bash
> ros2 service list -t
/aquabot/frame_publisher/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/aquabot/frame_publisher/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/aquabot/frame_publisher/get_parameters [rcl_interfaces/srv/GetParameters]
/aquabot/frame_publisher/list_parameters [rcl_interfaces/srv/ListParameters]
/aquabot/frame_publisher/set_parameters [rcl_interfaces/srv/SetParameters]
...
```

- Pour trouver tous les services d'un type spécifique, utiliser la commande :

```bash
ros2 service find <type_name>
```

```bash
> ros2 service find rcl_interfaces/srv/GetParameters
/aquabot/frame_publisher/get_parameters
/aquabot/optical_frame_publisher/get_parameters
/aquabot/robot_state_publisher/get_parameters
/aquabot/ros_gz_bridge/get_parameters
/ros_gz_bridge/get_parameters
/teleop_keyboard/get_parameters
```

Ici on a tous les services de type ```get_parameters```

- On peut appeler des services à partir de la ligne de commande, mais on doit d'abord connaître la structure des arguments d'entrée.

```bash
ros2 interface show <type_name>
```

```bash
> ros2 interface show rcl_interfaces/srv/SetParameters
# A list of parameters to set.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting each parameter succeeded or not and why.
SetParametersResult[] results
	bool successful
	string reason
```

Le ```---``` sépare la structure de la requête (ci-dessus) de la structure de la réponse (ci-dessous). 


-----------------------------------------------------------------------------

### Les parametres

Un paramètre est une ***valeur de configuration d'un nœud***. On peut considérer les paramètres comme des réglages de nœuds. Un nœud peut stocker des paramètres sous forme d'entiers, de flottants, de booléens, de chaînes de caractères et de listes.
Dans ROS 2, chaque nœud maintient ses propres paramètres.

- Pour voir les paramètres appartenant à vos nœuds, ouvrir un nouveau terminal et entrer la commande :

```bash
> ros2 param list
/aquabot/frame_publisher:
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  qos_overrides./tf.publisher.depth
  qos_overrides./tf.publisher.durability
  qos_overrides./tf.publisher.history
  qos_overrides./tf.publisher.reliability
  qos_overrides./tf_static.publisher.depth
  qos_overrides./tf_static.publisher.history
  qos_overrides./tf_static.publisher.reliability
  use_sim_time
  world_frame
/aquabot/optical_frame_publisher:
  qos_overrides./parameter_events.publisher.depth
....
```

> Chaque nœud possède le paramètre ```use_sim_time``` 