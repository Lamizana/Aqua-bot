# Gazebo

- [Index](/README.md)
-------------------------------------------------------------------------------
## Introduction

La simulation de robots est un outil essentiel dans la boîte à outils de tout roboticien. 
Un simulateur bien conçu permet de tester rapidement des algorithmes, de concevoir des robots, d'effectuer des tests de régression et d'entraîner des systèmes d'intelligence artificielle à l'aide de scénarios réalistes. 

***Gazebo permet de simuler avec précision et efficacité des populations de robots dans des environnements intérieurs et extérieurs complexes***. Gazebo est doté d'un moteur physique robuste, de graphiques de haute qualité et d'interfaces programmatiques et graphiques pratiques. Et surtout, Gazebo est gratuit et bénéficie d'une communauté dynamique.


-------------------------------------------------------------------------------

## Installation

Installation par default :
```bash
curl -sSL http://get.gazebosim.org | sh
```

Pour lancer Gazebo :
```bash
gazebo
```

### Installation Gazebo Garden [ recommandé ] :

Installation des outils nécessaires:
```bash
sudo apt-get update
sudo apt-get install lsb-release curl gnupg
```

Installation de Gazebo Garden:

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-garden
```

Toutes les bibliothèques devraient être prêtes à être utilisées et  prêt à être exécuté :

```bash
gz sim app
```

### Désinstaller l'installation binaire :

Si vous avez besoin de désinstaller Gazebo ou de passer à une installation basée sur la source une fois que vous avoir déjà installé la bibliothèque à partir de binaires, exécuter la commande suivante:

```bash
sudo apt remove gz-garden && sudo apt autoremove
```


-------------------------------------------------------------------------------

## Utilisation


