clear;
clc;
format long g;

%% 1. Configurations

q_home = zeros(6,1);

% Mesure /joint_states envoyee depuis ROS.
q_mesure = [
    -2.876514219678939e-05
     5.470863590016962e-06
    -9.094930989667774e-05
     3.4889108780771484e-05
    -8.495541396550835e-05
     2.7370617492124426e-05
];

% Matrice TF recopiee du terminal : seulement 3 decimales.
T_tf_arrondi = [
    0  0  1  0.890
    0 -1  0  0
    1  0  0  1.250
    0  0  0  1
];

%% 2. Calcul DH

T_home = fk_dh(q_home);
T_mesure = fk_dh(q_mesure);

disp('=== DH : HOME theorique ===');
disp(T_home);

disp('=== DH : angles mesures ===');
disp(T_mesure);

disp('=== TF arrondi recopie du terminal ===');
disp(T_tf_arrondi);

fprintf('Ecart articulaire maximal a HOME : %.9g rad\n', ...
    max(abs(q_mesure - q_home)));

fprintf(['Difference de position entre DH(q_mesure) et TF arrondi : ' ...
         '%.9g m\n'], ...
    norm(T_mesure(1:3,4) - T_tf_arrondi(1:3,4)));

fprintf(['Attention : cette difference inclut l''arrondi TF ; ' ...
         'ce n''est pas une validation haute precision.\n\n']);

%% 3. Verification independante : DH contre chaine URDF

Q = [
    q_home, ...
    q_mesure, ...
    [0.5; 0; 0; 0; 0; 0], ...
    [0.3; 0.4; -0.5; 0.2; -0.4; 0.6]
];

% Tests supplementaires de l'equivalence cinematique.
% Ce ne sont PAS des configurations validees sans collision.
rng(6);
Q = [Q, 0.5 * (2 * rand(6,20) - 1)];

max_ep = 0;
max_eR = 0;

for k = 1:size(Q,2)
    Td = fk_dh(Q(:,k));
    Tu = fk_urdf(Q(:,k));

    ep = norm(Td(1:3,4) - Tu(1:3,4));
    eR = norm(Td(1:3,1:3) - Tu(1:3,1:3), 'fro');

    max_ep = max(max_ep, ep);
    max_eR = max(max_eR, eR);
end

fprintf('=== DH contre chaine URDF : %d configurations ===\n', ...
    size(Q,2));
fprintf('Erreur maximale de position : %.12g m\n', max_ep);
fprintf('Erreur maximale de rotation, norme Frobenius : %.12g\n', ...
    max_eR);

% Les constantes RPY de l'URDF sont des approximations decimales
% de pi et -pi/2. Une tres petite difference est donc attendue.
assert(max_ep < 1e-9 && max_eR < 1e-8, ...
    'Echec de la verification DH / URDF.');

disp('Verification interne DH / URDF reussie.');

%% 4. Validation DH contre la mesure TF2 precise
% Robot immobile, proche de HOME.
% q_mesure correspond au message /joint_states releve.
% TF affiche avec 12 chiffres apres la virgule.

T_tf_precis = [
    0.000181377172  -0.000028756904   0.999999983138   0.890022561565
    0.000062254509  -0.999999997648  -0.000028768196  -0.000025601922
    0.999999981613   0.000062259726  -0.000181375381   1.249920152592
    0                0                0                1
    ];

T_dh_mesure = fk_dh(q_mesure);

delta_p = T_dh_mesure(1:3,4) - T_tf_precis(1:3,4);
delta_R = T_dh_mesure(1:3,1:3) - T_tf_precis(1:3,1:3);

erreur_position = norm(delta_p);
erreur_rotation_fro = norm(delta_R, 'fro');

disp('=== Validation DH contre TF2 precis ===');

disp('Difference de position [m] :');
disp(delta_p);

fprintf('Erreur de position : %.12g m\n', erreur_position);
fprintf('Ecart de rotation, norme de Frobenius : %.12g\n', ...
    erreur_rotation_fro);

% Seuils proposes pour notre verification numerique,
% pas des specifications de precision physique du robot.
assert(erreur_position < 1e-8, ...
    'Ecart de position DH / TF2 trop important.');

assert(erreur_rotation_fro < 1e-8, ...
    'Ecart de rotation DH / TF2 trop important.');

disp('Validation numerique DH / TF2 reussie.');

%% 5. Partie 3 : verification de la solution IK de PICK

q_pick = [
    -0.38050638088717653
    -0.008753653785249053
    -0.46129981068573
    1.1647952226565548e-09
    -1.1182501711547923
    0.38050638018104654
    ];

T_pick_cible = [
    1  0  0   0.75
    0 -1  0  -0.30
    0  0 -1   0.85
    0  0  0   1
    ];

T_pick_dh = fk_dh(q_pick);

disp('=== PICK : cinematique directe DH ===');
disp(T_pick_dh);

erreur_pick_position = norm( ...
    T_pick_dh(1:3,4) - T_pick_cible(1:3,4));

erreur_pick_rotation = norm( ...
    T_pick_dh(1:3,1:3) - T_pick_cible(1:3,1:3), 'fro');

fprintf('PICK : ecart de position DH / cible = %.12g m\n', ...
    erreur_pick_position);

fprintf('PICK : ecart de rotation DH / cible, Frobenius = %.12g\n', ...
    erreur_pick_rotation);

%% 6. PICK : comparaison DH contre la cinematique directe MoveIt

% Position retournee par /compute_fk
p_pick_moveit = [
    0.7499999971756399
    -0.3000000022671659
    0.8499999982271286
    ];

% Quaternion retourne : ordre explicite [x, y, z, w]
quat_pick_moveit = [
    1.0
    -9.840767084469948e-11
    -6.770872278719478e-10
    2.9716020217570323e-10
    ];

% Normalisation avant conversion
quat_pick_moveit = quat_pick_moveit / norm(quat_pick_moveit);

x = quat_pick_moveit(1);
y = quat_pick_moveit(2);
z = quat_pick_moveit(3);
w = quat_pick_moveit(4);

% Conversion quaternion -> matrice de rotation
% Sans necessiter de toolbox supplementaire
R_pick_moveit = [
    1-2*(y*y+z*z),  2*(x*y-z*w),    2*(x*z+y*w)
    2*(x*y+z*w),    1-2*(x*x+z*z),  2*(y*z-x*w)
    2*(x*z-y*w),    2*(y*z+x*w),    1-2*(x*x+y*y)
    ];

T_pick_moveit = eye(4);
T_pick_moveit(1:3,1:3) = R_pick_moveit;
T_pick_moveit(1:3,4) = p_pick_moveit;

% Recalcul avec les angles de la solution IK
T_pick_dh = fk_dh(q_pick);

ep_pick = norm( ...
    T_pick_dh(1:3,4) - T_pick_moveit(1:3,4));

eR_pick = norm( ...
    T_pick_dh(1:3,1:3) - T_pick_moveit(1:3,1:3), 'fro');

disp('=== PICK : transformation MoveIt ===');
disp(T_pick_moveit);

disp('=== PICK : transformation DH ===');
disp(T_pick_dh);

fprintf('PICK : erreur de position DH / MoveIt = %.12g m\n', ...
    ep_pick);

fprintf('PICK : ecart de rotation DH / MoveIt, Frobenius = %.12g\n', ...
    eR_pick);

% Seuils de verification numerique proposes pour ce modele
assert(ep_pick < 1e-8, ...
    'Verifier la position : ecart DH / MoveIt trop important.');

assert(eR_pick < 1e-8, ...
    'Verifier la rotation : ecart DH / MoveIt trop important.');

disp('Validation numerique de PICK reussie.');

%% 7. PLACE : verification de la solution IK

q_place = [
    0.38050578298679605
    -0.008752906279149665
    -0.46130350259048086
    4.958048048952841e-11
    -1.1182457324839004
    -0.38050578321996226
    ];

T_place_cible = [
    1  0  0   0.75
    0 -1  0   0.30
    0  0 -1   0.85
    0  0  0   1
    ];

T_place_dh = fk_dh(q_place);

disp('=== PLACE : cinematique directe DH ===');
disp(T_place_dh);

ep_place_cible = norm( ...
    T_place_dh(1:3,4) - T_place_cible(1:3,4));

eR_place_cible = norm( ...
    T_place_dh(1:3,1:3) - T_place_cible(1:3,1:3), 'fro');

fprintf('PLACE : ecart de position DH / cible = %.12g m\n', ...
    ep_place_cible);

fprintf('PLACE : ecart de rotation DH / cible, Frobenius = %.12g\n', ...
    eR_place_cible);

%% 8. PLACE : comparaison DH contre la cinematique directe MoveIt

% Position retournee par /compute_fk
p_place_moveit = [
    0.7500001808264501
    0.299999555436531
    0.8499970587053737
    ];

% Quaternion retourne : ordre [x, y, z, w]
quat_place_moveit = [
    1.0
    -1.0574328904800532e-10
    -8.178005868548199e-10
    4.370490543854716e-10
    ];

quat_place_moveit = quat_place_moveit / norm(quat_place_moveit);

x = quat_place_moveit(1);
y = quat_place_moveit(2);
z = quat_place_moveit(3);
w = quat_place_moveit(4);

R_place_moveit = [
    1-2*(y*y+z*z),  2*(x*y-z*w),    2*(x*z+y*w)
    2*(x*y+z*w),    1-2*(x*x+z*z),  2*(y*z-x*w)
    2*(x*z-y*w),    2*(y*z+x*w),    1-2*(x*x+y*y)
    ];

T_place_moveit = eye(4);
T_place_moveit(1:3,1:3) = R_place_moveit;
T_place_moveit(1:3,4) = p_place_moveit;

T_place_dh = fk_dh(q_place);

ep_place = norm( ...
    T_place_dh(1:3,4) - T_place_moveit(1:3,4));

eR_place = norm( ...
    T_place_dh(1:3,1:3) - T_place_moveit(1:3,1:3), 'fro');

disp('=== PLACE : transformation MoveIt ===');
disp(T_place_moveit);

disp('=== PLACE : transformation DH ===');
disp(T_place_dh);

fprintf('PLACE : erreur de position DH / MoveIt = %.12g m\n', ...
    ep_place);

fprintf('PLACE : ecart de rotation DH / MoveIt, Frobenius = %.12g\n', ...
    eR_place);

% Seuils de coherence numerique entre les deux modeles,
% distincts d'une tolerance de precision de l'IK.
assert(ep_place < 1e-8, ...
    'Verifier la position : ecart DH / MoveIt trop important.');

assert(eR_place < 1e-8, ...
    'Verifier la rotation : ecart DH / MoveIt trop important.');

disp('Validation numerique de PLACE reussie.');

%% Verification du chemin cartesien retourne par MoveIt

D = jsondecode(fileread("pre_pick_to_pick.json"));

% Verifier l'ordre des articulations avant d'utiliser le modele DH.
expected_names = "joint_" + string((1:6)');
actual_names = string(D.joint_names);
assert(isequal(actual_names(:), expected_names), ...
    'Ordre des articulations different : reordonner les donnees.');

N = numel(D.points);
P = zeros(N,3);
time = zeros(N,1);
orientation_error = zeros(N,1);

% Orientation souhaitee : quaternion xyzw = [1,0,0,0].
R_target = diag([1,-1,-1]);

for k = 1:N
    q = D.points(k).positions(:);
    T = fk_dh(q);

    P(k,:) = T(1:3,4)';
    time(k) = D.points(k).time_s;

    orientation_error(k) = norm( ...
        T(1:3,1:3) - R_target, 'fro');
end

% Ecart transversal par rapport a la droite x=0.75, y=-0.30.
transverse_error = sqrt( ...
    (P(:,1)-0.75).^2 + (P(:,2)+0.30).^2);

p_start = [0.75,-0.30,0.95];
p_end = [0.75,-0.30,0.85];

fprintf('\n=== Chemin cartesien MoveIt ===\n');
fprintf('Nombre de points : %d\n', N);
fprintf('Duree provisoire MoveIt : %.6f s\n', time(end)-time(1));
fprintf('Erreur position initiale : %.9g m\n', ...
    norm(P(1,:)-p_start));
fprintf('Erreur position finale : %.9g m\n', ...
    norm(P(end,:)-p_end));
fprintf('Ecart transversal maximal : %.9g m\n', ...
    max(transverse_error));
fprintf('Ecart orientation maximal, Frobenius : %.9g\n', ...
    max(orientation_error));

% Une descente monotone correspond a des differences z <= 0.
fprintf('Plus grande variation de z entre points : %.9g m\n', ...
    max(diff(P(:,3))));

figure;
tiledlayout(2,1);

nexttile;
plot(time, P(:,3), 'o-', 'LineWidth', 1.2);
grid on;
xlabel('Temps provisoire MoveIt [s]');
ylabel('z [m]');
title('Approche cartesienne : positions retournees');

nexttile;
plot(time, 1000*transverse_error, 'o-', 'LineWidth', 1.2);
grid on;
xlabel('Temps provisoire MoveIt [s]');
ylabel('Ecart transversal [mm]');
title('Ecart a la droite x=0.75, y=-0.30');

%% Export du chemin articulaire en fonction de l'avancement

% P et D ont ete calcules dans la section precedente.
% Progression normalisee selon la hauteur effectivement obtenue.
s_path = (P(1,3) - P(:,3)) / (P(1,3) - P(end,3));

% Verifications avant interpolation.
assert(all(diff(s_path) > 0), ...
    'La progression du chemin doit etre strictement croissante.');

Q_path = zeros(N,6);

for k = 1:N
    Q_path(k,:) = D.points(k).positions(:)';
end

path_table = array2table( ...
    [s_path, Q_path], ...
    'VariableNames', ...
    {'s','joint_1','joint_2','joint_3', ...
    'joint_4','joint_5','joint_6'});

writetable(path_table, 'pre_pick_to_pick_path.csv');

disp('Chemin exporte : pre_pick_to_pick_path.csv');
disp(path_table([1,end],:));

%% Verification cartesienne des profils reparametres
% Verification echantillonnee, hors execution.
% Vitesses et accelerations estimees par differences finies.

files = {'cubic_red_joints.csv', 'quintic_red_joints.csv'};
labels = {'Cubique', 'Quintique'};

figure;
tiledlayout(3,1);

for profile = 1:2
    M = readmatrix(files{profile});

    t = M(:,1);
    q = M(:,2:7);
    n = numel(t);

    assert(all(diff(t) > 0), ...
        'Les temps doivent etre strictement croissants.');

    P = zeros(n,3);
    rotation_error = zeros(n,1);
    R_target = diag([1,-1,-1]);

    for k = 1:n
        T = fk_dh(q(k,:)');
        P(k,:) = T(1:3,4)';

        rotation_error(k) = norm( ...
            T(1:3,1:3) - R_target, 'fro');
    end

    % Derivees numeriques de la position cartesienne.
    V = zeros(n,3);
    A = zeros(n,3);

    for axis = 1:3
        V(:,axis) = gradient(P(:,axis), t);
        A(:,axis) = gradient(V(:,axis), t);
    end

    speed = sqrt(sum(V.^2,2));
    acceleration = sqrt(sum(A.^2,2));

    transverse_error = sqrt( ...
        (P(:,1)-0.75).^2 + (P(:,2)+0.30).^2);

    % Ecarter deux points a chaque extremite pour les maxima
    % d'acceleration numerique : les bords sont moins precis.
    interior = 3:n-2;

    fprintf('\n=== %s : verification cartesienne ===\n', labels{profile});
    fprintf('Duree : %.6f s\n', t(end)-t(1));
    fprintf('Ecart transversal maximal : %.9g m\n', ...
        max(transverse_error));
    fprintf('Erreur finale de position : %.9g m\n', ...
        norm(P(end,:)-[0.75,-0.30,0.85]));
    fprintf('Ecart orientation maximal, Frobenius : %.9g\n', ...
        max(rotation_error));
    fprintf('Vitesse cartesienne maximale estimee : %.9g m/s\n', ...
        max(speed));
    fprintf('Acceleration maximale estimee hors bords : %.9g m/s2\n', ...
        max(acceleration(interior)));

    nexttile(1);
    plot(t, P(:,3), 'LineWidth', 1.5);
    hold on;
    grid on;
    ylabel('z [m]');
    title('Approche apres reparametrisation articulaire');

    nexttile(2);
    plot(t, speed, 'LineWidth', 1.5);
    hold on;
    grid on;
    ylabel('Norme vitesse [m/s]');

    nexttile(3);
    plot(t, acceleration, 'LineWidth', 1.5);
    hold on;
    grid on;
    ylabel('Norme acceleration [m/s^2]');
    xlabel('Temps [s]');
end

nexttile(1);
legend(labels, 'Location', 'best');

nexttile(2);
yline(0.200, 'k:', 'Limite rouge');

nexttile(3);
yline(0.300, 'k:', 'Limite rouge');

%% Fonctions locales

function T = fk_dh(q)
    q = q(:);

    a = [0.150, 0.600, 0.200, 0, 0, 0];
    alpha = [-pi/2, pi, -pi/2, pi/2, -pi/2, 0];
    d = [0.450, 0, 0, -0.640, 0, -0.100];
    offset = [0, -pi/2, 0, 0, 0, 0];

    T = eye(4);

    for i = 1:6
        theta = q(i) + offset(i);

        ct = cos(theta);
        st = sin(theta);
        ca = cos(alpha(i));
        sa = sin(alpha(i));

        A = [
            ct, -st*ca,  st*sa, a(i)*ct
            st,  ct*ca, -ct*sa, a(i)*st
             0,     sa,     ca, d(i)
             0,      0,      0, 1
        ];

        T = T * A;
    end

    T6_tool = diag([1, -1, -1, 1]);
    T = T * T6_tool;
end

function T = fk_urdf(q)
    % Reproduction directe des origines et axes de l'URDF fourni.
    q = q(:);

    T = tr(0,0,0.450) * rz(q(1)) ...
      * tr(0.150,0,0) * ry(q(2)) ...
      * tr(0,0,0.600) * ry(-q(3)) ...
      * tr(0,0,0.200) * rx(-q(4)) ...
      * tr(0.640,0,0) * ry(-q(5)) ...
      * tr(0.100,0,0) * rx(-q(6));

    % RPY exacts tels qu'ecrits dans le fichier URDF.
    T = T * ry(-1.570796327) * rx(3.1415926535);
end

function T = tr(x,y,z)
    T = eye(4);
    T(1:3,4) = [x;y;z];
end

function T = rx(t)
    c = cos(t);
    s = sin(t);
    T = [1 0 0 0; 0 c -s 0; 0 s c 0; 0 0 0 1];
end

function T = ry(t)
    c = cos(t);
    s = sin(t);
    T = [c 0 s 0; 0 1 0 0; -s 0 c 0; 0 0 0 1];
end

function T = rz(t)
    c = cos(t);
    s = sin(t);
    T = [c -s 0 0; s c 0 0; 0 0 1 0; 0 0 0 1];
end

export("validation_home.mlx", "validation_home.m", Format="m");