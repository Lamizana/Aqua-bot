# Aqua-bot

> By Alex LAMIZANA, Mathys VITIELLO, Nathan TRIJAUD
-------------------------------------------------------------------------------

## Sommaire

1. [ Le Design Thinking.](/doc/designThinking.md)
2. [ Installation et tutoriels Ros 2 .](/doc/ros2.md)
3. [ Installation et tutoriels Gazebo Garden .](/doc/gazebo.md)


-------------------------------------------------------------------------------

## Introduction

Développer et implémenter un système de mission dans un drone de surface (USV) virtuel au sein d’un environnement de développement et de simulation mis à disposition par les organisateurs.

L’édition 2024 de la compétition Aqua.Bot propose de vous affronter par équipe autour d’une mission simulée de maintenance d’un champ éolien. Les équipes participantes seront amenées à ***développer et implémenter un système de mission dans un drone de surface (USV)* virtuel au sein d’un environnement de développement et de simulation mis à disposition par les organisateurs***.

L’environnement de simulation proposé permet de s’affranchir des étapes de fabrication de l’USV, pour se concentrer sur la réalisation de l’automatisation du système durant les missions.

L’environnement proposé permet d’assurer la répétabilité de vos essais et ainsi d’évaluer les compétiteurs avec un échantillon de simulation suffisamment représentatif.

Les compétences minimums requises pour cette mission sont : **C++, Python, ROS2, Gazebo**

- Site sirehna :
https://www.sirehna-challengeaquabot.com/program/challenge-aquabot-2

- Gihub Aquabot Sirehna :
https://github.com/sirehna/Aquabot#installation

- Gihub Aquabot Sirehna Competitor:
https://github.com/sirehna/Aquabot-Competitor#installation

-------------------------------------------------------------------------------

### Objectifs

L’automatisation complète de l’USV est demandée aux participants.

Dans le cadre de la maintenance du parc éolien, ***le drone doit réaliser trois tâches successives*** pour compléter la mission WINDWATCH :

- Inspection du statut de chaque éolienne
    
- Identification de l’éolienne défaillante dans le champ

- Stabilisation & Maintenance devant l’éolienne défaillante

-------------------------------------------------------------------------------

### Deroulement du Challenge

Le challenge se déroulera en 4 phases :

- **Observation** (1 semaine) :
    - Installation et prise en main de l’environnement de simulation
    - Se référer au manuel utilisateur fourni au lancement de la compétition.

- **Idéation** (1 semaine) :
    - Organisation des équipes et projets.
    - Adoptez une approche agile en divisant l'objectif en sous-objectifs.

- **Prototypage** (8 semaines) :
    - Développement et mise en œuvre de l’USV et de son système de mission en suivant une approche collaborative et participative.
    
- **Pitch** (2 semaines) :
    - Mise en forme du dossier final et préparation à la présentation orale.

-------------------------------------------------------------------------------

### Livrable de mi-parcours

Merci de ***télécharger votre livrable de mi-parcours d'ici le 18 octobre 2024 23h***.

Voici la structure attendue (3/4 slides) : 

- 1 slide qui présente votre équipe et votre organisation au quotidien (fréquences de vos réunions, présentiel/distanciel,...) ;
    
- 1 à 2 slides sur vos avancées aujourd’hui (matériel et logiciels utilisés, état d'avancement de votre prototype) ;
    
- 1 slide présentant vos objectifs et vos attentes (formation, intervenants,..) pour la suite du programme. 

-------------------------------------------------------------------------------

### Description de l’environnement de simulation

L’environnement de développement de simulation **Gazebo ainsi que le framework de robotique ROS2 sont imposés**. Il sera mis à la disposition de chaque équipe inscrite un manuel utilisateur comprenant le guide d’installation de l’environnement ainsi que le guide de prise en main de ces outils.

Cet environnement se charge de simuler l’environnement marin (mer, bateaux, houle …), ainsi que le comportement de votre USV et de ses capteurs. Il met également à disposition des scénarios d’entrainement qui permettent aux équipes d’évaluer la performance de leurs algorithmes avec les mêmes critères que ceux qui seront utilisés pour le classement des équipes.

> [!IMPORTANT]
> L’USV mis à votre disposition est un monocoque de 6 mètres de long avec deux propulseurs azimutaux. Il aura une vitesse maximale de 12 nœuds.

-------------------------------------------------------------------------------

### Environnement

L’environnement marin dans lequel évolue les bateaux est soumis à une faible houle, mais sans aucun courant ni vent.

La taille de la zone de jeu est définie par défaut autour du point d’apparition de votre USV, par un carré de taille de ***600 mètres x 600 mètres***.

L'environnement maritime contient des obstacles statiques, tel que des éoliennes, des ilots ou encore des rochers.

-------------------------------------------------------------------------------

### Les scénarios

Vos algorithmes seront évalués sur des scénarios différents, allant d’un niveau facile jusqu’à difficile. Nous mettons à votre disposition pour vous aider durant la compétition un certain nombre de scénarios de façon à faciliter votre autoévaluation.

> Les équipes seront libres d’ajouter des scénarios complémentaires pour leur entrainement.

**Chaque scénario durera 20 minutes durant lesquels il faudra compléter les 3 étapes, ainsi que potentiellement l’étape BONUS**.

Les éléments d’un scénario pouvant être amenés à changer automatiquement sont :

- La position des éoliennes.

- Le statut des éoliennes.

- L’orientation du QR code d’identification.

- L’éolienne sur laquelle effectuer la maintenance,

- Les vagues et le vent.

Nous fixerons des valeurs maximales pour chaque élément au début de la compétition.

-------------------------------------------------------------------------------

### Développements à réaliser

> [!IMPORTANT]
> L’automatisation complète de votre USV est demandée.

3 étapes clés seront à réaliser pour compléter la ***mission WINDWATCH*** :

- **Étape 1** : *Inspection du statut de chaque éolienne* :
    Il devra naviguer au sein de l’environnement en évitant les collision pour aller identifier chacune des éoliennes en scannant leur QR code et en renvoyant leurs données à la plateforme.

- **Étape 2** : *Identification de l’éolienne défaillante dans le champ* :
    - Un ping de la plateforme leur sera envoyé avec l’identification de l’éolienne sur laquelle l’équipage doit intervenir, votre USV devra donc retrouver l’éolienne correspondant à la demande de la plateforme et la rallier.

- **Étape 3** : *Stabilisation & Maintenance devant l’éolien défaillante* :
    - Maintenir la position de l’USV devant l’éolienne pour la bonne conduite de la maintenance.

- **Étape Bonus** : *Round trip* :
    - Effectuer 2 tours complet de l’éolienne en conservant un cap précis afin de s’assurer du bon état de celle-ci.

Les modules suivants devront être développés :

- Un module de perception pour traiter les données capteur, détecter les éoliennes et lire les QR code.

- Un module de guidage et contrôle de l’USV faisant les choix de route et envoyant les ordres de contrôle à la propulsion.

-------------------------------------------------------------------------------

### Paramètres de notation

Chaque équipe devra fournir avant la date spécifiée :

- Le **code source** des modules développés compatibles avec l’environnement de simulation fournit.

- **Un rapport** précisant :
    - L’organisation de l’équipe mise en place dans le cas d’un développement collectif.
    - Une description des algorithmes mis en place.
    - Une notice d’installation (préciser le cas échéant la méthode d’installation de librairies complémentaires par exemple).
    - Les difficultés rencontrées et leur résolution si résolus.

> [!IMPORTANT]
> Chaque équipe recevra une note qui sera la somme d’une évaluation sur le rapport, d’une évaluation sur la performance et d’une évaluation sur leur pitch de présentation durant la finale.

- Il sera également demandé à l’issue de la phase d’IDEATION une présentation des équipes et projets.

#### Evaluation du rapport et de la qualité de codage :

- Qualité générale du rapport : observation et design thinking, gestion de projet et roadmap.
    
- Niveau technique des solutions proposées, et clarté de la description.
    
- Qualité générale du code livré.

- 15 pages maximum.

> [!WARNING]
> Une pénalité par jour de retard (date de réception, quelle que soit l’heure) sur la livraison du code source et/ou du rapport sera comptabilisée.

#### Evaluation de la performance selon des critères spécifique et écart type :

**Toute collision entrainera la fin du scénario**. Les points accumulés avant la collision resteront comptabilisé

Seront comptabilisés comme collision : La collision avec un obstacle fixe (rochers, phare, éolienne...).

Une inspection du code livré sera réalisée pour identifier toute tentative de triche. En cas de doute sur ce qui autorisé ou non, contacter l’équipe organisatrice.

#### Evaluation du pitch :

Une finale réunissant l’ensemble des participants sera organisée dans les locaux de Sirehna, au Technocampus Océan, 5 rue de l’Halbrane, **44340 BOUGUENAIS, France**.

Durant la finale, votre équipe devra ***présenter ses travaux pendant un pitch de courte durée (entre 5 et 7 minutes) devant un jury***, à l’aide d’un support visuel composé d’un ***maximum de 10 slides*** pour votre pitch-deck*. A l’issu des présentations, une note vous sera attribuée.

Celle-ci sera alors pondérée avec votre rapport et la performance de votre USV durant les tests.

Les équipes victorieuses seront annoncées durant l’événement final.

-------------------------------------------------------------------------------

##  Installation d'une VM Ubuntu sur VirtualBox

lien: https://www.malekal.com/comment-installer-ubuntu-sur-virtualbox/

- A mettre dans ***sgoinfre***.

Pour se familiariser avec ROS2, il est conseillé de suivre les tutoriels ROS
directement sur le site ROS.
- Principes fondamentaux : https://docs.ros.org/en/humble/Concepts/Basic.html
- Tutoriels : https://docs.ros.org/en/humble/How-To-Guides.html
