# Reprise du projet ET06 — 8 septembre 2026

## Environnement
- Ubuntu 24.04 / ROS 2 Jazzy
- Workspace : ~/TAP02_RI_ET06
- Robot : FANUC M-10iA
- Groupe : manipulator
- Reperes : base_link -> tool0

## Travail realise
- Configuration MoveIt personnelle creee.
- HOME et READY definis.
- Modele DH valide dans MATLAB pour HOME, PICK et PLACE.
- Solutions IK de pre_pick et pre_place obtenues.
- Poses conservees dans :
  src/et06_pick_place/config/key_poses.yaml

## Scene
- Script : src/et06_pick_place/scripts/setup_scene.py
- Quatre objets : support, piece, surface de depot, obstacle.
- Obstacle :
  centre = [0.45, -0.06, 0.975]
  dimensions = [0.04, 0.04, 0.35]
- Les cinq configurations sont valides.
- Le segment articulaire direct HOME -> pre_pick est bloque
  par le poste a s = 0.55, selon le test echantillonne.
- Piece non attachee.
- Aucun cycle complet execute.

## Planification
- RRTConnect et RRTstar configures pour manipulator.
- longest_valid_segment_fraction = 0.001.
- ValidateSolution conserve.
- Script : src/et06_pick_place/scripts/benchmark_planners.py

## Benchmark HOME -> pre_pick
Dossier :
src/et06_pick_place/results/20260908_153443_847604

- RRTConnect : 10/10 succes.
- RRTstar : 1/10 succes.
- RRTConnect retenu provisoirement.
- Cause exacte des neuf echecs RRTstar non determinee.
- Les echecs ont zero point et environ cinq secondes d'appel.

Resume :
- RRTConnect : temps MoveIt median 0.033116 s.
- RRTConnect : longueur articulaire mediane 3.34373 rad.
- RRTConnect : duree mediane 13.6432 s.
- Unique succes RRTstar : longueur 1.88414 rad.

## Prochaine action
Check si l'ajout du calcul d'acceleration RMS a ete fait
dans scripts/summarize_benchmark.py ; sinon l'ajouter.
Lancer le resume sur les resultats existants.
Ne pas refaire le benchmark inutilement.

Puis :
- Approche cartesienne pre_pick -> pick.
- Profils cubique et quintique en Python.
- Verification des vitesses et accelerations.
- Suite du cycle, attachement et detachement de la piece.
- Jacobien.
- Integration et documentation des scripts dans le paquet.

## Attention
- Les scripts Python sont actuellement lances directement.
- L'installation et les dependances du paquet restent a finaliser.
- Les mesures concernent la simulation, pas la precision physique.

## Dernier point de reprise — 9 septembre 2026

### Avancement
- HOME, PICK et PLACE : modele DH valide contre TF2/MoveIt.
- Scene de quatre objets en place.
- Les cinq configurations cles sont valides.
- RRTConnect retenu provisoirement : 10/10 succes.
- RRTstar : 1/10 succes ; cause exacte des echecs non determinee.
- Benchmark resume avec longueur articulaire, duree et acceleration RMS.

### Approche pre_pick -> pick
- Descente de 10 cm, orientation constante.
- MoveIt Cartesian path complet : fraction 1.0, 26 points.
- Chemin exporte depuis MATLAB en fonction de l'avancement s.
- Profils cubique et quintique reparametres en Python.
- Duree commune rouge ajustee : 3.160710 s.
- Facteurs articulaires utilises : vitesse 0.1, acceleration 0.1.
- Limite effective d'acceleration articulaire : 0.1 rad/s2.
- Les deux profils respectent les limites aux points analyses.
- Verification cartesienne DH :
  - ecart transversal maximal : environ 1.243 micrometre ;
  - vitesse maximale cubique : 0.04746 m/s ;
  - vitesse maximale quintique : 0.05932 m/s ;
  - acceleration estimee hors bords cubique : 0.05994 m/s2 ;
  - acceleration estimee hors bords quintique : 0.05779 m/s2.
- Verification de validite : 2001/2001 points pour chaque profil.
- Ces controles sont echantillonnes, pas une garantie continue.
- Quintique retenu comme candidat pour l'approche.
- Aucune execution de cet enchainement effectuee.
- Piece toujours non attachee.

### Fichiers importants
- config/key_poses.yaml
- scripts/setup_scene.py
- scripts/compute_pick_approach.py
- scripts/generate_approach_profiles.py
- scripts/retime_approach.py
- scripts/validate_retimed_trajectory.py
- results/cartesian/pre_pick_to_pick.json
- results/cartesian/pre_pick_to_pick_path.csv
- results/retimed/cubic_red_joints.csv
- results/retimed/quintic_red_joints.csv
Chemins ci-dessus relatifs a src/et06_pick_place/.

### Controleurs
- Materiel configure : mock_components/GenericSystem.
- fanuc_arm_controller actif.
- joint_state_broadcaster actif.
- Action MoveIt : /execute_trajectory.
- Action controleur :
  /fanuc_arm_controller/follow_joint_trajectory.
- MoveItSimpleControllerManager configure pour les six joints.
- Commande en position ; etats position et vitesse.
- controller_manager update_rate : 100 Hz.

### Prochaine action
Ecrire un seul programme pour :
1. Lire l'etat articulaire actuel.
2. Planifier avec RRTConnect vers le debut exact du CSV quintique.
3. Demander confirmation avant execution.
4. Executer le transfert et verifier son succes.
5. Check l'arrivee au depart de l'approche.
6. Executer le CSV quintique avec son timing de 3.160710 s.
7. Attendre et verifier le resultat.
Ne pas appliquer un nouveau timing qui remplacerait notre profil.

### Encore a faire
- Execution et observation de 4A + 4B.
- Attach the workpiece et transfert 4C.
- Approche 4D avec les limites appropriees.
- Profils bleus articulaires et verification avec piece attachee.
- Detachement et cycle complet.
- Jacobien manuel et comparaison MoveIt/KDL.
- Verification des vitesses avec le Jacobien.
- Finaliser integration, dependances et installation du paquet.
- README, rangement des outils de diagnostic, video et soutenance.
- Expliquer chaque script et choix technique a l'etudiant.
