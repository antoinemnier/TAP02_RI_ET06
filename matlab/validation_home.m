%% ET06 - FANUC M-10iA
% Validation of the DH model using ROS2 and MoveIt results.
%
% Part 2:
%   Compare the HOME transformation obtained with DH and TF2.
%
% Part 3:
%   Compare the PICK and PLACE transformations obtained with
%   DH and MoveIt forward kinematics.

clear;
clc;
format long g;

%% 1. Joint configurations

% Theoretical HOME configuration
q_home = zeros(6,1);

% Joint values measured at HOME using the /joint_states topic
q_home_measured = [
    -2.876514219678939e-05
     5.470863590016962e-06
    -9.094930989667774e-05
     3.4889108780771484e-05
    -8.495541396550835e-05
     2.7370617492124426e-05
];

% PICK joint solution obtained with the MoveIt IK service
q_pick = [
    -0.38050638088717653
    -0.008753653785249053
    -0.46129981068573
     1.1647952226565548e-09
    -1.1182501711547923
     0.38050638018104654
];

% PLACE joint solution obtained with the MoveIt IK service
q_place = [
     0.38050578298679605
    -0.008752906279149665
    -0.46130350259048086
     4.958048048952841e-11
    -1.1182457324839004
    -0.38050578321996226
];

%% 2. Reference transformations obtained from ROS2

% Precise transformation base_link -> tool0 obtained with tf2_echo
% while the robot was at q_home_measured
T_home_tf2 = [
     0.000181377172  -0.000028756904   0.999999983138   0.890022561565
     0.000062254509  -0.999999997648  -0.000028768196  -0.000025601922
     0.999999981613   0.000062259726  -0.000181375381   1.249920152592
     0                 0                 0                 1
];

% PICK pose returned by the MoveIt FK service
p_pick_moveit = [
     0.7499999971756399
    -0.3000000022671659
     0.8499999982271286
];

% Quaternion order used by ROS: [x, y, z, w]
q_pick_moveit = [
     1.0
    -9.840767084469948e-11
    -6.770872278719478e-10
     2.9716020217570323e-10
];

% PLACE pose returned by the MoveIt FK service
p_place_moveit = [
    0.7500001808264501
    0.299999555436531
    0.8499970587053737
];

q_place_moveit = [
     1.0
    -1.0574328904800532e-10
    -8.178005868548199e-10
     4.370490543854716e-10
];

% Convert the MoveIt poses into homogeneous transformations
T_pick_moveit = poseToMatrix(p_pick_moveit, q_pick_moveit);
T_place_moveit = poseToMatrix(p_place_moveit, q_place_moveit);

%% 3. Part 2: HOME transformation

T_home_theoretical = forwardKinematicsDH(q_home);
T_home_measured = forwardKinematicsDH(q_home_measured);

fprintf('\n========================================\n');
fprintf('PART 2 - HOME TRANSFORMATION\n');
fprintf('========================================\n');

disp('Theoretical DH transformation at HOME:');
disp(T_home_theoretical);

disp('DH transformation using the measured joint values:');
disp(T_home_measured);

disp('Transformation measured with TF2:');
disp(T_home_tf2);

compareTransformations(T_home_measured, T_home_tf2);

fprintf('Maximum joint deviation from theoretical HOME: %.6e rad\n', ...
    max(abs(q_home_measured - q_home)));

%% 4. Part 3: PICK transformation

T_pick_dh = forwardKinematicsDH(q_pick);

fprintf('\n========================================\n');
fprintf('PART 3 - PICK TRANSFORMATION\n');
fprintf('========================================\n');

disp('Transformation calculated with DH:');
disp(T_pick_dh);

disp('Transformation returned by MoveIt FK:');
disp(T_pick_moveit);

compareTransformations(T_pick_dh, T_pick_moveit);

%% 5. Part 3: PLACE transformation

T_place_dh = forwardKinematicsDH(q_place);

fprintf('\n========================================\n');
fprintf('PART 3 - PLACE TRANSFORMATION\n');
fprintf('========================================\n');

disp('Transformation calculated with DH:');
disp(T_place_dh);

disp('Transformation returned by MoveIt FK:');
disp(T_place_moveit);

compareTransformations(T_place_dh, T_place_moveit);

fprintf('\nAll DH comparisons were completed successfully.\n');

%% Local functions

function T = forwardKinematicsDH(q)
%FORWARDKINEMATICSDH Calculates base_link -> tool0.
%
% The model uses standard Denavit-Hartenberg transformations.
% Each row of the table contains:
%
%   [a, alpha, d, theta_offset]

    q = q(:);

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

        T = T * dhMatrix(a, alpha, d, theta);
    end

    % The final DH frame and the URDF tool0 frame have different
    % orientations. This fixed rotation aligns both frames.
    T_6_tool0 = diag([1, -1, -1, 1]);

    T = T * T_6_tool0;
end

function A = dhMatrix(a, alpha, d, theta)
%DHMATRIX Returns one standard DH homogeneous transformation.

    A = [
        cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)
        sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)
        0,           sin(alpha),             cos(alpha),            d
        0,           0,                      0,                     1
    ];
end

function T = poseToMatrix(position, quaternion)
%POSETOMATRIX Converts a ROS pose into a homogeneous transformation.
%
% The quaternion must use the ROS order:
%   [x, y, z, w]

    quaternion = quaternion(:) / norm(quaternion);

    x = quaternion(1);
    y = quaternion(2);
    z = quaternion(3);
    w = quaternion(4);

    R = [
        1-2*(y^2+z^2), 2*(x*y-z*w),   2*(x*z+y*w)
        2*(x*y+z*w),   1-2*(x^2+z^2), 2*(y*z-x*w)
        2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x^2+y^2)
    ];

    T = eye(4);
    T(1:3,1:3) = R;
    T(1:3,4) = position(:);
end

function compareTransformations(T_dh, T_reference)
%COMPARETRANSFORMATIONS Compares position and orientation.

    position_error = norm( ...
        T_dh(1:3,4) - T_reference(1:3,4));

    rotation_error = norm( ...
        T_dh(1:3,1:3) - T_reference(1:3,1:3), ...
        'fro');

    fprintf('Position error: %.6e m\n', position_error);
    fprintf('Rotation error, Frobenius norm: %.6e\n', ...
        rotation_error);

    % These thresholds verify numerical consistency between the models.
    % They do not represent the physical accuracy of the real robot.
    assert(position_error < 1e-5, ...
        'The position error is too large.');

    assert(rotation_error < 1e-6, ...
        'The rotation error is too large.');

    fprintf('Result: VALID\n');
end
