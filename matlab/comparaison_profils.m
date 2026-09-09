clear;
clc;

% Donnees generees par le script Python
C = readtable("cubic_red.csv");
Q = readtable("quintic_red.csv");
C_bleu = readtable("cubic_blue.csv");
Q_bleu = readtable("quintic_blue.csv");
figure('Name', 'Profils — limites rouges');
tiledlayout(3,1);

% Position
nexttile;
plot(C.time_s, C.z_m, 'r-', 'LineWidth', 1.5);
hold on;
plot(Q.time_s, Q.z_m, 'b--', 'LineWidth', 1.5);
grid on;
ylabel('z [m]');
title('Approche pre-pick -> pick : profils souhaites');
legend('Cubique', 'Quintique', 'Location', 'best');

% Vitesse
nexttile;
plot(C.time_s, C.vz_m_s, 'r-', 'LineWidth', 1.5);
hold on;
plot(Q.time_s, Q.vz_m_s, 'b--', 'LineWidth', 1.5);
yline(-0.200, 'k:', 'Limite de descente');
grid on;
ylabel('v_z [m/s]');

% Acceleration
nexttile;
plot(C.time_s, C.az_m_s2, 'r-', 'LineWidth', 1.5);
hold on;
plot(Q.time_s, Q.az_m_s2, 'b--', 'LineWidth', 1.5);
yline(0.300, 'k:');
yline(-0.300, 'k:');
grid on;
ylabel('a_z [m/s^2]');
xlabel('Temps [s]');

%% Cas bleu
figure('Name', 'Profils — limites bleues');
tiledlayout(3,1);

% Position
nexttile;
plot(C_bleu.time_s, C_bleu.z_m, 'r-', 'LineWidth', 1.5);
hold on;
plot(Q_bleu.time_s, Q_bleu.z_m, 'b--', 'LineWidth', 1.5);
grid on;
ylabel('z [m]');
title('Approche de 10 cm — limites bleues');
legend('Cubique', 'Quintique', 'Location', 'best');

% Vitesse
nexttile;
plot(C_bleu.time_s, C_bleu.vz_m_s, 'r-', 'LineWidth', 1.5);
hold on;
plot(Q_bleu.time_s, Q_bleu.vz_m_s, 'b--', 'LineWidth', 1.5);
yline(-0.100, 'k:', 'Limite de descente');
grid on;
ylabel('v_z [m/s]');

% Acceleration
nexttile;
plot(C_bleu.time_s, C_bleu.az_m_s2, 'r-', 'LineWidth', 1.5);
hold on;
plot(Q_bleu.time_s, Q_bleu.az_m_s2, 'b--', 'LineWidth', 1.5);
yline(0.020, 'k:');
yline(-0.020, 'k:');
grid on;
ylabel('a_z [m/s^2]');
xlabel('Temps [s]');


