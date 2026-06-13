close all; clc; clear all

%% =========================================================
%% ASSIGNMENT 1 – 3D Modeling: Frame Manipulation
%% =========================================================

%% 2.3 Frames definieren und zeichnen
figure; hold on; grid on; axis equal; view(3);
xlabel('x'); ylabel('y'); zlabel('z');
title('Assignment 1 – Frame Manipulation');

% Weltframe F0
drawFrame([0;0;0], eye(3));

% F1: Extrinsische Rotation -89x, -175y, 90z + Translation
R1  = rotZd(90) * rotYd(-175) * rotXd(-89);
O1  = [-0.78; 0.15; 0.21];
T01 = [R1, O1; 0 0 0 1];

% F2: Extrinsische Rotation -89x, -175y, 90z + Translation
R2  = rotZd(90) * rotYd(-175) * rotXd(-89);
O2  = [-0.78; -0.05; 0.21];
T02 = [R2, O2; 0 0 0 1];

% F3: Extrinsische Rotation 91x, -5y, 10z + Translation
R3  = rotZd(10) * rotYd(-5) * rotXd(91);
O3  = [-0.55; -0.2; 0.21];
T03 = [R3, O3; 0 0 0 1];

drawFrame(O1, R1);
drawFrame(O2, R2);
drawFrame(O3, R3);

%% 2.4 Transformationen zwischen Frames
T21 = inv(T02) * T01;   % Pose von F1 ausgedrueckt in F2
T32 = inv(T03) * T02;   % Pose von F2 ausgedrueckt in F3
T20 = inv(T02);          % Pose von F0 ausgedrueckt in F2

fprintf('--- Assignment 1: Transformationen ---\n');
fprintf('T01:\n'); disp(T01)
fprintf('T21:\n'); disp(T21)
fprintf('T32:\n'); disp(T32)
fprintf('T20:\n'); disp(T20)

%% 2.5 Punktkoordinaten transformieren
P3 = [1; 0; 0];    % Punkt P3 gegeben in F3
P2 = [-1; 0; -1];  % Punkt P2 gegeben in F2

P3_F0 = T03          * [P3; 1]; P3_F0 = P3_F0(1:3);
P3_F1 = inv(T01)*T03 * [P3; 1]; P3_F1 = P3_F1(1:3);
P2_F0 = T02          * [P2; 1]; P2_F0 = P2_F0(1:3);
P2_F1 = inv(T01)*T02 * [P2; 1]; P2_F1 = P2_F1(1:3);

fprintf('--- Assignment 1: Punkttransformationen ---\n');
fprintf('P3 in F0: [%.4f  %.4f  %.4f]\n', P3_F0(1), P3_F0(2), P3_F0(3));
fprintf('P3 in F1: [%.4f  %.4f  %.4f]\n', P3_F1(1), P3_F1(2), P3_F1(3));
fprintf('P2 in F0: [%.4f  %.4f  %.4f]\n', P2_F0(1), P2_F0(2), P2_F0(3));
fprintf('P2 in F1: [%.4f  %.4f  %.4f]\n', P2_F1(1), P2_F1(2), P2_F1(3));

%% =========================================================
%% ASSIGNMENT 2 – Kinematics: Forward Transformation & Manipulability
%% =========================================================

%% Roboter laden
gen3 = loadrobot("kinovaGen3");
gen3.DataFormat = 'column';
eeName = 'EndEffector_Link';

%% DH-Parameter Kinova Gen3 (offiziell, klassisches DH, a=0 fuer alle Links)
% Standard DH: T_i = Rot_z(q_i + theta_off) * Trans_z(d) * Rot_x(alpha)
%
%           alpha(rad)   d(m)                   theta_offset(rad)
DH = [
    pi,      0,                                0;   % Basis (fest, kein Gelenk)
    pi/2,   -(0.1564 + 0.1284),                0;   % Gelenk 1
    pi/2,   -(0.0054 + 0.0064),               pi;   % Gelenk 2
    pi/2,   -(0.2104 + 0.2104),               pi;   % Gelenk 3
    pi/2,   -(0.0064 + 0.0064),               pi;   % Gelenk 4
    pi/2,   -(0.2084 + 0.1059),               pi;   % Gelenk 5
    pi/2,    0,                               pi;   % Gelenk 6
    pi,     -(0.1059 + 0.0615),               pi;   % Gelenk 7 (Interface)
];

%% Konfigurationen (Grad -> Bogenmass)
q_home = [0;    15;    180;  -130;    0;    55;   90 ] * pi/180;
qF1    = [ 16.39;  299.74;   5.10; 268.15;  22.51;  63.48;  71.20] * pi/180;
qF2    = [351.73;  300.69;   8.11; 263.95; 356.98;  63.00;  79.94] * pi/180;
qF3    = [  1.18;  291.32;  18.47; 290.91;  94.36; 112.93;  46.00] * pi/180;

configs = {q_home, qF1, qF2, qF3};
labels  = {'Home', 'qF1', 'qF2', 'qF3'};

%% Figure (Assignment 2)
fig2 = figure;
ax2  = axes(fig2);
show(gen3, q_home, 'PreservePlot', false, 'Parent', ax2);
hold(ax2, 'on'); grid(ax2, 'on'); view(ax2, 3);
xlim(ax2, [-1.2  1.2]);
ylim(ax2, [-1.2  1.2]);
zlim(ax2, [-0.2  1.5]);
axis(ax2, 'vis3d');
xlabel(ax2, 'x'); ylabel(ax2, 'y'); zlabel(ax2, 'z');
title(ax2, 'Kinova Gen3 – Vorwärtskinematik + Manipulierbarkeit');

%% vm an Home-Konfiguration (Startkonfiguration)
J_home     = geometricJacobian(gen3, q_home, eeName);
[V_h, D_h] = eig(J_home(1:3,:) * J_home(1:3,:)');
[~, id_h]  = max(diag(D_h));
vm_home    = V_h(:, id_h);
fprintf('=== Home ===\n');
fprintf('vm: [%.4f  %.4f  %.4f]\n', vm_home(1), vm_home(2), vm_home(3));

%% Animation: Home -> qF1 -> qF2 -> qF3
steps = 40;
h_vm  = [];

for k = 1:3
    q_start = configs{k};
    q_end   = configs{k+1};
    dq      = wrapToPi(q_end - q_start);   % kuerzester Weg pro Gelenk (kein 360°-Umweg)

    for s = 1:steps
        alpha_t = s / steps;
        q       = q_start + alpha_t * dq;

        % Eigene FK (ohne Tool-Offset fuer EE-Frame)
        T08 = forwardKin(q, DH);
        p   = T08(1:3, 4);

        % Jacobian + Manipulierbarkeit (translatorischer Anteil)
        J   = geometricJacobian(gen3, q, eeName);
        J_t = J(1:3, :);
        [V, D]  = eig(J_t * J_t');
        [~, id] = max(diag(D));
        vm = V(:, id);

        % Roboter anzeigen
        show(gen3, q, 'PreservePlot', false, 'Parent', ax2);

        % vm-Vektor in Magenta
        if ~isempty(h_vm) && isvalid(h_vm)
            delete(h_vm);
        end
        h_vm = quiver3(ax2, p(1), p(2), p(3), ...
                       vm(1)*0.1, vm(2)*0.1, vm(3)*0.1, ...
                       0, 'Color', 'm', 'LineWidth', 2, 'MaxHeadSize', 2);
        drawnow;
      
    end

    % Vergleich eigene FK vs. MATLAB an Zielkonfiguration
    T08      = forwardKin(q_end, DH);
    T_matlab = getTransform(gen3, q_end, eeName);
    dp       = norm(T08(1:3,4) - T_matlab(1:3,4));
    R_diff   = T08(1:3,1:3)' * T_matlab(1:3,1:3);
    dtheta   = acos(min(1, (trace(R_diff) - 1) / 2)) * 180/pi;

    % vm an Zielkonfiguration
    J_end      = geometricJacobian(gen3, q_end, eeName);
    [V_e, D_e] = eig(J_end(1:3,:) * J_end(1:3,:)');
    [~, id_e]  = max(diag(D_e));
    vm_end     = V_e(:, id_e);

    fprintf('\n=== %s ===\n', labels{k+1});
    fprintf('Eigene FK  – Position:    [%.4f  %.4f  %.4f] m\n', T08(1,4),      T08(2,4),      T08(3,4));
    fprintf('MATLAB FK  – Position:    [%.4f  %.4f  %.4f] m\n', T_matlab(1,4), T_matlab(2,4), T_matlab(3,4));
    fprintf('Differenz (Position):     %.6f m\n', dp);
    fprintf('Differenz (Orientierung): %.4f deg\n', dtheta);
    fprintf('vm:                       [%.4f  %.4f  %.4f]\n', vm_end(1), vm_end(2), vm_end(3));
end

%% =========================================================
%% ASSIGNMENT 3 – Dynamics & Sensors: Pick & Place + Energieverbrauch
%% =========================================================

%% 4.3a Joint Space (Point-to-Point) Trajektorie
% Sequenz: Home -> F3 (B2 greifen) -> F2 (B2 ablegen) -> F1 (B1 greifen) -> F3 (B1 ablegen) -> Home
pp_configs = {q_home, qF3, qF2, qF1, qF3, q_home};
pp_labels  = {'Home', 'F3: B2 greifen', 'F2: B2 ablegen', 'F1: B1 greifen', 'F3: B1 ablegen', 'Home'};

n_wp = numel(pp_configs);
wayPoints = zeros(7, n_wp);
wayPoints(:,1) = pp_configs{1};
for i = 2:n_wp
    dq = wrapToPi(pp_configs{i} - pp_configs{i-1});   % kuerzester Weg pro Gelenk
    wayPoints(:,i) = wayPoints(:,i-1) + dq;
end

seg_time        = 2;                       % Sekunden pro Segment
n_seg           = n_wp - 1;
samples_per_seg = 50;
numSamples      = n_seg * samples_per_seg + 1;
[q_traj, qd_traj, qdd_traj, t_traj] = trapveltraj(wayPoints, numSamples, ...
                                                   'EndTime', seg_time);

%% Animation: Pick & Place (Joint Space)
fig3 = figure;
ax3  = axes(fig3);
show(gen3, q_home, 'PreservePlot', false, 'Parent', ax3);
hold(ax3, 'on'); grid(ax3, 'on'); view(ax3, 3);
xlim(ax3, [-1.2 1.2]); ylim(ax3, [-1.2 1.2]); zlim(ax3, [-0.2 1.5]);
axis(ax3, 'vis3d');
xlabel(ax3, 'x'); ylabel(ax3, 'y'); zlabel(ax3, 'z');
title(ax3, 'Assignment 3 – Pick & Place (Joint Space, Point-to-Point)');

for k = 1:numSamples
    show(gen3, q_traj(:,k), 'PreservePlot', false, 'Parent', ax3);
    drawnow;
end

%% 4.3b Cartesian Space (Continuous Path) – Beispiel F1 -> F3
T_F1 = getTransform(gen3, qF1, eeName);
T_F3 = getTransform(gen3, qF3, eeName);

numCart       = 50;
tSamples_cart = linspace(0, seg_time, numCart);
T_cart        = transformtraj(T_F1, T_F3, [0 seg_time], tSamples_cart);

ik      = inverseKinematics('RigidBodyTree', gen3);
weights = [1 1 1 1 1 1];
q_cart  = zeros(7, numCart);
q_init  = wrapToPi(qF1);   % Startwert innerhalb der Gelenkgrenzen (physikalisch identische Pose)
for k = 1:numCart
    q_cart(:,k) = ik(eeName, T_cart(:,:,k), weights, q_init);
    q_init      = q_cart(:,k);
end

fig4 = figure;
ax4  = axes(fig4);
show(gen3, qF1, 'PreservePlot', false, 'Parent', ax4);
hold(ax4, 'on'); grid(ax4, 'on'); view(ax4, 3);
xlim(ax4, [-1.2 1.2]); ylim(ax4, [-1.2 1.2]); zlim(ax4, [-0.2 1.5]);
axis(ax4, 'vis3d');
xlabel(ax4, 'x'); ylabel(ax4, 'y'); zlabel(ax4, 'z');
title(ax4, 'Assignment 3 – Cartesian Space Pfad (F1 -> F3)');

for k = 1:numCart
    show(gen3, q_cart(:,k), 'PreservePlot', false, 'Parent', ax4);
    drawnow;
end

%% 4.4 Energieverbrauch (aus Joint-Space-Trajektorie)
gen3.Gravity = [0 0 -9.81];

tau = zeros(7, numSamples);
for k = 1:numSamples
    tau(:,k) = inverseDynamics(gen3, q_traj(:,k), qd_traj(:,k), qdd_traj(:,k));
end

P = sum(tau .* qd_traj, 1);   % Gesamtleistung aller Gelenke
E = cumtrapz(t_traj, P);      % kumulierte Energie

fig5 = figure;
subplot(2,1,1);
plot(t_traj, P, 'LineWidth', 1.5); grid on;
xlabel('t [s]'); ylabel('P [W]');
title('Assignment 3 – Leistung P(t)');
for i = 1:n_seg
    xline(i*seg_time, '--', pp_labels{i+1});
end

subplot(2,1,2);
plot(t_traj, E, 'LineWidth', 1.5); grid on;
xlabel('t [s]'); ylabel('E [J]');
title(sprintf('Energie E(t) – Gesamt: %.2f J', E(end)));
for i = 1:n_seg
    xline(i*seg_time, '--', pp_labels{i+1});
end

%% Funktionen

function T = forwardKin(q, DH)
    % Basis-Frame (Zeile 1 der DH-Tabelle, fest, kein Gelenk)
    T = dhFrame(DH(1,3), DH(1,2), 0, DH(1,1));
    % Gelenke 1-7 (Zeilen 2-8)
    for i = 1:7
        theta = q(i) + DH(i+1, 3);          % q_i + theta-Offset
        T = T * dhFrame(theta, DH(i+1,2), 0, DH(i+1,1));
    end
    % Tool-Offset: v = [0 0 0.12]^T im Interface-Frame (Aufgabe 3.2.4)
    T = T * [eye(3), [0; 0; 0.12]; 0 0 0 1];
end

function T = dhFrame(theta, d, a, alpha)
    % Standard DH: Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);
    Rz = [ct -st 0; st ct 0; 0 0 1];
    Rx = [1 0 0;  0 ca -sa;  0 sa ca];
    t  = [a*ct; a*st; d];
    T  = [Rz*Rx, t; 0 0 0 1];
end

% --- Hilfsfunktionen fuer Assignment 1 ---

function drawFrame(O, R)
    scale = 0.1;
    colors = {'r', 'g', 'b'};
    for i = 1:3
        ep = O + scale * R(:,i);
        plot3([O(1) ep(1)], [O(2) ep(2)], [O(3) ep(3)], colors{i}, 'LineWidth', 2);
    end
end

function R = rotXd(a)
    R = [1 0 0; 0 cosd(a) -sind(a); 0 sind(a) cosd(a)];
end

function R = rotYd(a)
    R = [cosd(a) 0 sind(a); 0 1 0; -sind(a) 0 cosd(a)];
end

function R = rotZd(a)
    R = [cosd(a) -sind(a) 0; sind(a) cosd(a) 0; 0 0 1];
end
