%% ET06 - FANUC M-10iA
clear;
clc;
close all;
format short e;

%% 1. Joint configurations

% Joint values measured near HOME using /joint_states
q_home = [
    -2.876514219678939e-05
     5.470863590016962e-06
    -9.094930989667774e-05
     3.4889108780771484e-05
    -8.495541396550835e-05
     2.7370617492124426e-05
];

% PICK solution obtained with the MoveIt IK service
q_pick = [
    -0.38050638088717653
    -0.008753653785249053
    -0.46129981068573
     1.1647952226565548e-09
    -1.1182501711547923
     0.38050638018104654
];

% PLACE solution obtained with the MoveIt IK service
q_place = [
     0.38050578298679605
    -0.008752906279149665
    -0.46130350259048086
     4.958048048952841e-11
    -1.1182457324839004
    -0.38050578321996226
];

%% 2. Reference transformations from ROS2 and MoveIt

% Precise HOME transformation obtained with:
% ros2 run tf2_ros tf2_echo base_link tool0 -p 12
T_home_ros = [
     0.000181377172  -0.000028756904   0.999999983138   0.890022561565
     0.000062254509  -0.999999997648  -0.000028768196  -0.000025601922
     0.999999981613   0.000062259726  -0.000181375381   1.249920152592
     0                 0                 0                 1
];

% PICK pose returned by the MoveIt FK service
p_pick_ros = [
     0.7499999971756399
    -0.3000000022671659
     0.8499999982271286
];

% ROS quaternion order: [x, y, z, w]
quat_pick_ros = [
     1.0
    -9.840767084469948e-11
    -6.770872278719478e-10
     2.9716020217570323e-10
];

% PLACE pose returned by the MoveIt FK service
p_place_ros = [
    0.7500001808264501
    0.299999555436531
    0.8499970587053737
];

quat_place_ros = [
     1.0
    -1.0574328904800532e-10
    -8.178005868548199e-10
     4.370490543854716e-10
];

T_pick_ros = poseToMatrix(p_pick_ros, quat_pick_ros);
T_place_ros = poseToMatrix(p_place_ros, quat_place_ros);

%% 3. Forward kinematics with the DH model

T_home_dh = forwardKinematicsDH(q_home);
T_pick_dh = forwardKinematicsDH(q_pick);
T_place_dh = forwardKinematicsDH(q_place);

%% 4. DH validation results

[homePositionError, homeRotationError] = transformationError(T_home_dh, T_home_ros);

[pickPositionError, pickRotationError] = transformationError(T_pick_dh, T_pick_ros);

[placePositionError, placeRotationError] = transformationError(T_place_dh, T_place_ros);

configuration = ["HOME"; "PICK"; "PLACE"];

positionError = [
    homePositionError
    pickPositionError
    placePositionError
];

rotationError = [
    homeRotationError
    pickRotationError
    placeRotationError
];

results = table( configuration, positionError, rotationError, 'VariableNames', ...
    {'Configuration', 'PositionError_m', 'RotationError'});

fprintf('\n============================================\n');
fprintf('DH MODEL VALIDATION\n');
fprintf('============================================\n');

disp(results);

if all(positionError < 1e-5) && all(rotationError < 1e-6)
    fprintf('Result: DH and ROS2 results agree.\n');
else
    fprintf('Result: the DH model must be checked.\n');
end

%% 5. Locate the Cartesian profile files

cubicRedFile = 'cubic_red.csv';
quinticRedFile = 'quintic_red.csv';

cubicBlueFile = 'cubic_blue.csv';
quinticBlueFile = 'quintic_blue.csv';

%% 6. Red profile comparison

if isfile(cubicRedFile) && isfile(quinticRedFile)

    cubicRed = readtable(cubicRedFile);
    quinticRed = readtable(quinticRedFile);

    plotProfiles( cubicRed, quinticRed, 'Red Cartesian profiles', 0.200, 0.300);

    printProfileResults( 'RED', cubicRed, quinticRed, 0.200, 0.300);

else
    fprintf('\nRED PROFILE FILES NOT FOUND\n');
    fprintf('Expected files:\n');
    fprintf('%s\n', cubicRedFile);
    fprintf('%s\n', quinticRedFile);
end

%% 7. Blue profile comparison

if isfile(cubicBlueFile) && isfile(quinticBlueFile)

    cubicBlue = readtable(cubicBlueFile);
    quinticBlue = readtable(quinticBlueFile);

    plotProfiles( cubicBlue, quinticBlue, 'Blue Cartesian profiles', 0.100, 0.020);

    printProfileResults( 'BLUE', cubicBlue, quinticBlue, 0.100, 0.020);

else
    fprintf('\nBLUE PROFILE FILES NOT FOUND\n');
    fprintf('Expected files:\n');
    fprintf('%s\n', cubicBlueFile);
    fprintf('%s\n', quinticBlueFile);
end

fprintf('\nMATLAB validation completed.\n');

%% 8. Analytical Jacobian and Velocity Verification (Part 5)

fprintf('\n============================================\n');
fprintf('PART 5: ANALYTICAL JACOBIAN VALIDATION\n');
fprintf('============================================\n');

% Calculate analytical Jacobian at PICK position
J_pick = analyticalJacobianDH(q_pick);
disp('Analytical Jacobian at PICK pose (J):');
disp(J_pick);

% Verification of the Cartesian velocity for the RED profile (0.200 m/s)
v_cartesian_target = [0; 0; -0.200; 0; 0; 0];
q_dot = pinv(J_pick) * v_cartesian_target;
v_cartesian_calc = J_pick * q_dot;

fprintf('Theoretical Joint Velocities (q_dot) to reach 0.200 m/s:\n');
disp(q_dot');

fprintf('Resulting Cartesian Z-Velocity (m/s): %.4f\n', v_cartesian_calc(3));

if abs(abs(v_cartesian_calc(3)) - 0.200) < 1e-4
    fprintf('SUCCESS: The analytical Jacobian confirms the 0.200 m/s limit.\n');
else
    fprintf('WARNING: Velocity mismatch.\n');
end

% --- Comparaison avec le solveur KDL (Consigne Partie 5) ---
try
    % 1. Importation du modèle URDF
    robot = importrobot('fanuc.urdf');
    robot.DataFormat = 'column'; % Format compatible avec le solveur

    % 2. Création de l'état articulaire pour la pose PICK
    config = q_pick; 

    % 3. Extraction du Jacobien via le solveur KDL
    J_kdl_brut = geometricJacobian(robot, config, 'tool0');
    
    % KDL met l'angulaire en premier. On inverse pour comparer avec notre analytique [V; W]
    J_kdl = [J_kdl_brut(4:6, :); J_kdl_brut(1:3, :)];
    
    fprintf('\n=== JACOBIEN DU SOLVEUR KDL ===\n');
    disp(J_kdl);
    
    fprintf('Différence maximale entre Analytique et KDL : %.2e\n', max(abs(J_pick(:) - J_kdl(:))));
catch ME
    disp('Erreur KDL : Assurez-vous que le fichier fanuc.urdf est bien dans le dossier courant.');
end
%% Local functions

function T = forwardKinematicsDH(q)
% Calculate the transformation from base_link to tool0.

    q = q(:);

    % Standard DH table:
    % [a, alpha, d, theta_offset]
    DH = [
        0.150, -pi/2,  0.450,  0
        0.600,  pi,    0,     -pi/2
        0.200, -pi/2,  0,      0
        0,      pi/2, -0.640,  0
        0,     -pi/2,  0,      0
        0,      0,    -0.100,  0
    ];

    T = eye(4);

    for i = 1:6
        a = DH(i,1);
        alpha = DH(i,2);
        d = DH(i,3);
        theta = q(i) + DH(i,4);

        A = [
            cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)

            sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)

            0, sin(alpha), cos(alpha), d

            0, 0, 0, 1
        ];

        T = T * A;
    end

    % Fixed rotation between the last DH frame and tool0
    T_6_tool0 = diag([1, -1, -1, 1]);

    T = T * T_6_tool0;
end

function T = poseToMatrix(position, quaternion)
% Convert a ROS pose into a homogeneous transformation.
%
% Input quaternion order:
% [x, y, z, w]

    position = position(:);
    quaternion = quaternion(:);

    % Normalize the quaternion
    quaternion = quaternion / norm(quaternion);

    x = quaternion(1);
    y = quaternion(2);
    z = quaternion(3);
    w = quaternion(4);

    R = [1-2*(y^2+z^2), 2*(x*y-z*w), 2*(x*z+y*w)

        2*(x*y+z*w), 1-2*(x^2+z^2), 2*(y*z-x*w)

        2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x^2+y^2)
    ];

    T = eye(4);
    T(1:3,1:3) = R;
    T(1:3,4) = position;
end

function [positionError, rotationError] = transformationError(T_dh, T_ros)
% Calculate position and orientation errors.

    positionError = norm( T_dh(1:3,4) - T_ros(1:3,4));

    rotationError = norm( T_dh(1:3,1:3) - T_ros(1:3,1:3), 'fro');
end

function plotProfiles( cubic, quintic, figureTitle, velocityLimit, accelerationLimit)
% Plot position, velocity and acceleration.

    figure( 'Name', figureTitle, 'Color', 'white');

    tiledlayout(3,1);

    % Position
    nexttile;

    plot( cubic.time_s, cubic.z_m, 'r-', 'LineWidth', 1.5);

    hold on;

    plot( quintic.time_s, quintic.z_m, 'b--', 'LineWidth', 1.5);

    grid on;
    ylabel('z [m]');
    title(figureTitle);

    legend( 'Cubic', 'Quintic', 'Location', 'best');

    % Velocity
    nexttile;

    plot( cubic.time_s, cubic.vz_m_s, 'r-', 'LineWidth', 1.5);

    hold on;

    plot( quintic.time_s, quintic.vz_m_s, 'b--', 'LineWidth', 1.5);

    yline( velocityLimit, 'k:', 'Velocity limit');

    yline( -velocityLimit, 'k:');

    grid on;
    ylabel('v_z [m/s]');

    % Acceleration
    nexttile;

    plot( cubic.time_s, cubic.az_m_s2, 'r-', 'LineWidth', 1.5);

    hold on;

    plot( quintic.time_s, quintic.az_m_s2, 'b--', 'LineWidth', 1.5);

    yline( accelerationLimit, 'k:', 'Acceleration limit');

    yline( -accelerationLimit, 'k:');

    grid on;
    ylabel('a_z [m/s^2]');
    xlabel('Time [s]');
end

function printProfileResults( profileName, cubic, quintic, velocityLimit, accelerationLimit)
% Print the main values of both profiles.

    cubicSpeed = max(abs(cubic.vz_m_s));
    cubicAcceleration = max(abs(cubic.az_m_s2));

    quinticSpeed = max(abs(quintic.vz_m_s));
    quinticAcceleration = max(abs(quintic.az_m_s2));

    fprintf('\n============================================\n');
    fprintf('%s PROFILE COMPARISON\n', profileName);
    fprintf('============================================\n');

    fprintf( 'Cubic:   speed = %.4f m/s, acceleration = %.4f m/s^2\n', cubicSpeed, cubicAcceleration);

    fprintf( 'Quintic: speed = %.4f m/s, acceleration = %.4f m/s^2\n', quinticSpeed, ...
        quinticAcceleration);

    fprintf( 'Limits:  speed = %.4f m/s, acceleration = %.4f m/s^2\n', ...
        velocityLimit, accelerationLimit);

    if cubicSpeed <= velocityLimit && cubicAcceleration <= accelerationLimit

        fprintf('Cubic profile: VALID\n');
    else
        fprintf('Cubic profile: LIMIT EXCEEDED\n');
    end

    if quinticSpeed <= velocityLimit && quinticAcceleration <= accelerationLimit

        fprintf('Quintic profile: VALID\n');
    else
        fprintf('Quintic profile: LIMIT EXCEEDED\n');
    end
end

function J = analyticalJacobianDH(q)
% Calculate the analytical Jacobian matrix for FANUC M-10iA
    q = q(:);

    % Standard DH table: [a, alpha, d, theta_offset]
    DH = [
        0.150, -pi/2,  0.450,  0
        0.600,  pi,    0,     -pi/2
        0.200, -pi/2,  0,      0
        0,      pi/2, -0.640,  0
        0,     -pi/2,  0,      0
        0,      0,    -0.100,  0
        ];

    T = eye(4);
    z = zeros(3, 7);
    o = zeros(3, 7);
    
    % Base frame (0)
    z(:, 1) = [0; 0; 1];
    o(:, 1) = [0; 0; 0];
    
    for i = 1:6
        a = DH(i,1); alpha = DH(i,2); d = DH(i,3); theta = q(i) + DH(i,4);
        A = [
            cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)
            sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)
            0,           sin(alpha),             cos(alpha),            d
            0,           0,                      0,                     1
            ];
        T = T * A;
        z(:, i+1) = T(1:3, 3); % Z-axis of current frame
        o(:, i+1) = T(1:3, 4); % Origin of current frame
    end
    
    o_n = o(:, 7); % End-effector origin
    J = zeros(6, 6);
    
    for i = 1:6
        % Linear velocity (cross product of Z_i-1 and vector to end-effector)
        J(1:3, i) = cross(z(:, i), o_n - o(:, i));
        % Angular velocity (Z_i-1)
        J(4:6, i) = z(:, i);
    end
end