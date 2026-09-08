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
- Script : src/et06_pick_place/scripts/check_scene.py
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
- Script : src/et06_pick_place/scripts/test_plan.py

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
Verifier si l'ajout du calcul d'acceleration RMS a ete fait
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
