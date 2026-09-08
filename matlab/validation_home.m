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