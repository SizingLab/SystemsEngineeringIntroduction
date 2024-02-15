(page_lecture_1)=
# System engineering introduction with a cubesat thermal control study case

Les cubesat sont des nanosatellite utilisant des formats standardisés et des kits de sous-sytèmes techniques réutilisables fournis par un ecosystème d'entreprises et start-ups.
Votre entreprise veut proposer des kits permettant le controle thermique des cubesat. Votre premier produit s'interessera aux applications nécessitant un réchauffement.

![image](https://phxcubesat.asu.edu/sites/default/files/styles/panopoly_image_full/public/general/image_1.png?itok=QFqUmAUZ)

Vous aurez:
- à adapter le kit aux besoins spécifiques des clients. Ces besoins seront capturés dans un formulaire spécifique .
- proposer des éléments chauffants/capteurs adaptés aux besoins (conception préliminaire).
- identifier rapidement la dynamique thermique du produit à partir d'une carte d'interface et d'un programme informatique sur PC
- proposer une loi de commande validée par simulation et expérimentalement. Vous prouverez par simulation le robustesse de la loi de commande par rapport à des variations paramétriques correspondant à différents environnement de tests/qualification/utilisation. Vous quantifierez également les rejets de perturbation.
- implementer la loi de commande sur un microcontroleur et un algorithme de monitoring (state flow) permettant la protection du sous-système à réchauffer.

> Déliverables à fournir:  
- Document de spécification (formulaire) permettant de définir les besoins en contrôle de température
- Notebook de justification technique de la solution de rechauffage
- Notebook d'identification et justification des lois de commande
- Codes C 'arduino' d'implementation de la commande et du monitoring.  

> Planning des séances: 

20h de contact et 20 h de travail personnel distribué selon le planning suivant:

**Part 1 : Specification and predesign**

| Séance | Type | Durée | Moyen / Salle | Objectif |
| -:- | -:- | -:- | -:- |-:- |
|| CM | 1.25 h | amphi (EM) | System engineering approachs in satellite program |
|| CM | 1.25 h | amphi (AM) | Thermal control subsystem requirement |
|| Home | 2.5 h | | Document Reading (CubeSat) + First Specification document |
|| CM | 1.25 h | amphi (AR) | Jupyter notebook et python (control, odeint, ...) |
|| TD | 2.5h | salle TD (EM+AM+AR+ES)| Preliminary design |
|| Home | 2.5h | | Specification document + widget de calcul |

1er rendu

**Part 2 : Control and software development**

| Séance | Type | Durée | Moyen / Salle | Objectif |
| -:- | -:- | -:- | -:- |-:- |
| | CM | 1.25 h (AR ou Th. Rocacher ou LR) | Presentation carte TCLab +  microcontroleurs +  |
| | TD | 2.5h | HIL + salle TD (AR+ES+AM+LR/GG) | Carte TClab et échelon d'identification |
| | Home | 2.5h | | Identification d'un modèle (1er, 2nd ordre) pour la commande |
| | TD | 2.5h | salle TD  (AR+ES+AM+LR/GG) | Commande et validation par simulation (carte TC Lab) |
| | Home | 2.5h | | Commande et validation par simulation sur modèle linéarisé |
| | TP | 3.75h | salle XAO (AR+ES+AM+LR + GG + MB + IH) | Prototypage de la commande (PID anti-windup) sur micro-controleur (python + arduino)|
| | TP | 2.5h | salle HIL (AR+ES+AM+LR + GG + MB + IH) | Réponse à un nouveau besoin, TP exam|

