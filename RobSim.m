close all; clc; clear all

%% =========================================================
%% ASSIGNMENT 1 – 3D Modeling: Frame Manipulation
%% =========================================================

%% 2.3 Frames definieren und zeichnen
figure; hold on; grid on; axis equal; view(3);
xlabel('x'); ylabel('y'); zlabel('z');
title('Assignment 1 - Frame Manipulation');

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

%% Greifer (Robotiq 2F-85) fuer die Visualisierung anhaengen
% gen3 (7 Gelenke) bleibt unveraendert fuer FK/Jacobian/IK/Dynamik.
% gen3_vis ist nur fuer show() gedacht und traegt zusaetzlich den Greifer.
gen3_vis = loadrobot("kinovaGen3");
gen3_vis.DataFormat = 'column';
gripper  = loadrobot("robotiq2F85");
gripper.DataFormat = 'column';

% Ausrichtung des Greifers korrigieren: die Basis des Robotiq-Greifers ist
% im geladenen Modell gegenueber dem Kinova-EndEffector_Link verdreht.
% Dazu wird ein zusaetzlicher, fester "Mount"-Koerper zwischen EndEffector_Link
% und dem Greifer eingefuegt, der die Korrektur-Rotation gripperMountTF traegt.
% Falls die Ausrichtung danach noch nicht passt, hier den Winkel/die Achse
% anpassen (z.B. rotZd(90), rotYd(90), Kombination aus mehreren Rotationen, ...).
gripperMountTF = [rotZd(90), [0;0;0]; 0 0 0 1];
mountJoint = rigidBodyJoint('gripper_mount_joint', 'fixed');
setFixedTransform(mountJoint, gripperMountTF);
gripperMount = rigidBody('gripper_mount');
gripperMount.Joint = mountJoint;
addBody(gen3_vis, gripperMount, eeName);

addSubtree(gen3_vis, 'gripper_mount', gripper, 'ReplaceBase', false);
qVisTemplate = homeConfiguration(gen3_vis);   % Greifer-Gelenke bleiben in Home-Pose

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
show(gen3_vis, withGripper(q_home, qVisTemplate), 'PreservePlot', false, 'Parent', ax2, 'Frames', 'off');
hold(ax2, 'on'); grid(ax2, 'on'); view(ax2, 3);
xlim(ax2, [-1.2  1.2]);
ylim(ax2, [-1.2  1.2]);
zlim(ax2, [-0.2  1.5]);
axis(ax2, 'vis3d');
xlabel(ax2, 'x'); ylabel(ax2, 'y'); zlabel(ax2, 'z');
title(ax2, 'Kinova Gen3 - Forward Kinematics + Manipulability');

%% vm an Home-Konfiguration (Startkonfiguration)
J_home     = geometricJacobian(gen3, q_home, eeName);
% geometricJacobian liefert [omega; v] = J*qdot, d.h.
% J(1:3,:) = rotatorischer Anteil, J(4:6,:) = translatorischer (linearer) Anteil.
% Aufgabe 3.3 verlangt die translatorische Richtung -> J(4:6,:).
[V_h, D_h] = eig(J_home(4:6,:) * J_home(4:6,:)');
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

        % Eigene FK ohne Tool-Offset -> Interface-/EE-Frame (3.3.2: vm-Pfeil am EE-Frame)
        T08 = forwardKin(q, DH, false);
        p   = T08(1:3, 4);

        % Jacobian + Manipulierbarkeit (translatorischer Anteil)
        % J(1:3,:) = rotatorisch, J(4:6,:) = translatorisch -> hier J(4:6,:) verwenden
        J   = geometricJacobian(gen3, q, eeName);
        J_t = J(4:6, :);
        [V, D]  = eig(J_t * J_t');
        [~, id] = max(diag(D));
        vm = V(:, id);

        % Roboter anzeigen
        show(gen3_vis, withGripper(q, qVisTemplate), 'PreservePlot', false, 'Parent', ax2, 'Frames', 'off');

        % vm-Vektor in Magenta
        if ~isempty(h_vm) && isvalid(h_vm)
            delete(h_vm);
        end
        h_vm = quiver3(ax2, p(1), p(2), p(3), ...
                       vm(1)*0.1, vm(2)*0.1, vm(3)*0.1, ...
                       0, 'Color', 'm', 'LineWidth', 2, 'MaxHeadSize', 2);
        drawnow;
      
    end

    % Vergleich eigene FK vs. MATLAB an Zielkonfiguration (3.2.6: "frames line up")
    % OHNE Tool-Offset, da getTransform(...,'EndEffector_Link') den
    % Interface-Frame liefert (12 cm vor der Werkzeugspitze).
    T08      = forwardKin(q_end, DH, false);
    T_matlab = getTransform(gen3, q_end, eeName);
    dp       = norm(T08(1:3,4) - T_matlab(1:3,4));
    R_diff   = T08(1:3,1:3)' * T_matlab(1:3,1:3);
    dtheta   = acos(min(1, (trace(R_diff) - 1) / 2)) * 180/pi;

    % Separates Deliverable (3.2.4): Werkzeugspitze MIT Tool-Offset (12 cm in z)
    T08_tool = forwardKin(q_end, DH, true);

    % vm an Zielkonfiguration (J(4:6,:) = translatorischer Anteil)
    J_end      = geometricJacobian(gen3, q_end, eeName);
    [V_e, D_e] = eig(J_end(4:6,:) * J_end(4:6,:)');
    [~, id_e]  = max(diag(D_e));
    vm_end     = V_e(:, id_e);

    fprintf('\n=== %s ===\n', labels{k+1});
    fprintf('Eigene FK  – Position:    [%.4f  %.4f  %.4f] m\n', T08(1,4),      T08(2,4),      T08(3,4));
    fprintf('MATLAB FK  – Position:    [%.4f  %.4f  %.4f] m\n', T_matlab(1,4), T_matlab(2,4), T_matlab(3,4));
    fprintf('Differenz (Position):     %.6f m\n', dp);
    fprintf('Differenz (Orientierung): %.4f deg\n', dtheta);
    fprintf('Werkzeugspitze (3.2.4, mit Tool-Offset 12 cm): [%.4f  %.4f  %.4f] m\n', T08_tool(1,4), T08_tool(2,4), T08_tool(3,4));
    fprintf('vm:                       [%.4f  %.4f  %.4f]\n', vm_end(1), vm_end(2), vm_end(3));
end

%% =========================================================
%% ASSIGNMENT 3 – Dynamics & Sensors: Pick & Place + Energieverbrauch
%% =========================================================

%% 4.3.0 Anfahr-/Rueckzugsposen ("_up") per inverse Kinematik
% Ueber jeder Greif-/Ablegepose liegt eine um zLift in Welt-z angehobene
% "_up"-Pose. Der Greifer faehrt darueber an bzw. zieht sich darueber
% zurueck, statt seitlich durch die bereits abgelegte Flasche zu fahren
% -> Kollisionsvermeidung. Nur T(3,4) wird angehoben, die Orientierung
% T(1:3,1:3) bleibt unveraendert. Warmstart mit der Greifpose q selbst
% haelt die IK-Loesung nah an der Ausgangskonfiguration (kein Ellbogen-
% Umklappen). weights = [Orientierung(1:3) Position(1:3)]: Position wird
% mit [1 1 1] staerker gewichtet als die Orientierung [0.25 0.25 0.25],
% da beim Anheben primaer die Hoehe (Kollisionsfreiheit) zaehlt.
zLift   = 0.10;                     % 10 cm senkrecht ueber der Greifpose
ik      = inverseKinematics('RigidBodyTree', gen3);
weights = [0.25 0.25 0.25 1 1 1];

qF1_up = liftConfig(gen3, ik, eeName, weights, qF1, zLift);
qF2_up = liftConfig(gen3, ik, eeName, weights, qF2, zLift);
qF3_up = liftConfig(gen3, ik, eeName, weights, qF3, zLift);

%% 4.3a Joint Space (Point-to-Point) Trajektorie
% PTP (4.3, joint space): reine Gelenkraum-Interpolation (trapveltraj)
% zwischen vorab/offline berechneten Konfigurationen. Waehrend der Fahrt
% findet KEINE IK statt. Sequenz inkl. Anfahr-/Rueckzugsposen:
% Home -> F3_up -> F3 -> F3_up (B2 greifen) -> F2_up -> F2 -> F2_up (B2 ablegen)
%      -> F1_up -> F1 -> F1_up (B1 greifen) -> F3_up -> F3 -> F3_up (B1 ablegen) -> Home
pp_configs = {q_home, ...
              qF3_up, qF3, qF3_up, ...   % B2 an F3 greifen
              qF2_up, qF2, qF2_up, ...   % B2 an F2 ablegen
              qF1_up, qF1, qF1_up, ...   % B1 an F1 greifen
              qF3_up, qF3, qF3_up, ...   % B1 an F3 ablegen
              q_home};

% Wegpunkt-Indizes der eigentlichen Greif-/Ablege-Ereignisse (fuer Energie-Plot)
event_idx    = [3, 6, 9, 12];
event_labels = {'F3: pick B2', 'F2: place B2', 'F1: pick B1', 'F3: place B1'};

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

event_times = (event_idx - 1) * seg_time;   % Zeiten der Greif-/Ablege-Ereignisse

%% Animation: Pick & Place (Joint Space)
fig3 = figure;
ax3  = axes(fig3);
show(gen3_vis, withGripper(q_home, qVisTemplate), 'PreservePlot', false, 'Parent', ax3, 'Frames', 'off');
hold(ax3, 'on'); grid(ax3, 'on'); view(ax3, 3);
xlim(ax3, [-1.2 1.2]); ylim(ax3, [-1.2 1.2]); zlim(ax3, [-0.2 1.5]);
axis(ax3, 'vis3d');
xlabel(ax3, 'x'); ylabel(ax3, 'y'); zlabel(ax3, 'z');
title(ax3, 'Assignment 3 - Pick & Place (Joint Space, Point-to-Point)');

% Frames zur Orientierung einzeichnen: Weltframe F0 (Roboterbasis) und
% die Flaschenpositionen F1, F2, F3 aus Assignment 1
drawFrame([0;0;0], eye(3), 'F0');
drawFrame(O1, R1, 'F1');
drawFrame(O2, R2, 'F2');
drawFrame(O3, R3, 'F3');

for k = 1:numSamples
    show(gen3_vis, withGripper(q_traj(:,k), qVisTemplate), 'PreservePlot', false, 'Parent', ax3, 'Frames', 'off');
    drawnow;
end

%% 4.3b Cartesian Space (Continuous Path) – B1: F1 -> F3 (inkl. Lift)
% CP (4.3, cartesian space): kartesischer Pfad (transformtraj: lineare
% Positionsinterpolation + SLERP der Orientierung) ueber F1 -> F1_up ->
% F3_up -> F3, pro Zeitschritt per inverseKinematics in den Gelenkraum
% zurueckgerechnet (Warmstart aus dem vorigen Schritt). Entspricht
% denselben Eckposen wie die PTP-Wegpunkte 9-12 (B1: F1 -> F3, mit Lift)
% und ist damit direkt mit dem PTP-Segment vergleichbar (siehe 4.4 Bonus).
T_F1    = getTransform(gen3, qF1,    eeName);
T_F1_up = getTransform(gen3, qF1_up, eeName);
T_F3_up = getTransform(gen3, qF3_up, eeName);
T_F3    = getTransform(gen3, qF3,    eeName);

cartWaypointsT      = {T_F1, T_F1_up, T_F3_up, T_F3};
numCartSeg          = numel(cartWaypointsT) - 1;
samples_per_cartseg = 50;
weights_cart        = [1 1 1 1 1 1];

q_init = wrapToPi(qF1);   % Startwert innerhalb der Gelenkgrenzen (physikalisch identische Pose)
for s = 1:numCartSeg
    tSamples = linspace(0, seg_time, samples_per_cartseg + 1);
    T_cart   = transformtraj(cartWaypointsT{s}, cartWaypointsT{s+1}, [0 seg_time], tSamples);

    q_seg = zeros(7, numel(tSamples));
    for k = 1:numel(tSamples)
        q_seg(:,k) = ik(eeName, T_cart(:,:,k), weights_cart, q_init);
        q_init     = q_seg(:,k);
    end

    if s == 1
        q_cart = q_seg;
        t_cart = tSamples;
    else
        q_cart = [q_cart, q_seg(:,2:end)];                    %#ok<AGROW>
        t_cart = [t_cart, (s-1)*seg_time + tSamples(2:end)];  %#ok<AGROW>
    end
end
numCart = size(q_cart, 2);

fig4 = figure;
ax4  = axes(fig4);
show(gen3_vis, withGripper(qF1, qVisTemplate), 'PreservePlot', false, 'Parent', ax4, 'Frames', 'off');
hold(ax4, 'on'); grid(ax4, 'on'); view(ax4, 3);
xlim(ax4, [-1.2 1.2]); ylim(ax4, [-1.2 1.2]); zlim(ax4, [-0.2 1.5]);
axis(ax4, 'vis3d');
xlabel(ax4, 'x'); ylabel(ax4, 'y'); zlabel(ax4, 'z');
title(ax4, 'Assignment 3 - Cartesian Space Path (B1: F1 -> F3, with lift)');

% Frames zur Orientierung einzeichnen: Weltframe F0 (Roboterbasis) und
% die Flaschenpositionen F1, F2, F3 aus Assignment 1
drawFrame([0;0;0], eye(3), 'F0');
drawFrame(O1, R1, 'F1');
drawFrame(O2, R2, 'F2');
drawFrame(O3, R3, 'F3');

for k = 1:numCart
    show(gen3_vis, withGripper(q_cart(:,k), qVisTemplate), 'PreservePlot', false, 'Parent', ax4, 'Frames', 'off');
    drawnow;
end

%% 4.3.x Kollisionspruefung (Selbstkollision) – "collision-free motion"
% Aufgabe 4.3 verlangt eine kollisionsfreie Bewegung. Hier wird JEDE
% Konfiguration einzeln per checkCollision auf Selbstkollision geprueft:
% Wegpunkte (pp_configs), PTP-Bahn (q_traj) und CP-Bahn (q_cart).
% 'SkippedSelfCollisions','parent' ignoriert benachbarte Glieder, die sich
% am gemeinsamen Gelenk ohnehin beruehren wuerden (sonst Fehlalarm).

% 0) Hat das Modell ueberhaupt Kollisionsgeometrie?
hasCollisionGeom = false;
for i = 1:gen3.NumBodies
    if ~isempty(gen3.Bodies{i}.Collisions)
        hasCollisionGeom = true;
        break;
    end
end
if ~hasCollisionGeom
    warning(['Robotermodell "gen3" hat keine Kollisionsgeometrie (Collisions). ' ...
             'checkCollision liefert dann immer "keine Kollision", ohne ' ...
             'Aussagekraft. Ggf. ein Robotermodell mit Kollisionskoerpern laden.']);
end

% 1) Wegpunkte einzeln pruefen: ist schon eine Ziel-/Anfahrpose problematisch?
fprintf('\n--- Kollisionspruefung: Wegpunkte (pp_configs) ---\n');
for i = 1:n_wp
    if isSelfCollision(gen3, pp_configs{i})
        warning('Selbstkollision am Wegpunkt %d von %d', i, n_wp);
    end
end
fprintf('Alle %d Wegpunkte geprueft.\n', n_wp);

% 2) PTP-Trajektorie (q_traj) Sample fuer Sample pruefen
fprintf('\n--- Kollisionspruefung: PTP-Trajektorie (q_traj) ---\n');
nCollPTP = 0;
for k = 1:numSamples
    if isSelfCollision(gen3, q_traj(:,k))
        warning('Selbstkollision bei PTP-Sample %d (t = %.2f s)', k, t_traj(k));
        if nCollPTP == 0
            reportSelfCollision(gen3, q_traj(:,k), sprintf('PTP-Sample %d (t = %.2f s)', k, t_traj(k)));
        end
        nCollPTP = nCollPTP + 1;
    end
end
if nCollPTP == 0
    fprintf('PTP-Trajektorie ist kollisionsfrei (%d Samples geprueft).\n', numSamples);
else
    fprintf('PTP-Trajektorie: %d von %d Samples kollidieren!\n', nCollPTP, numSamples);
    fprintf(['Einfache Gegenmassnahmen: (a) zLift (oben, aktuell %.2f m) erhoehen, ' ...
             'damit die "_up"-Posen weiter weg von der Flasche liegen; ' ...
             '(b) zwischen den betroffenen Wegpunkten in pp_configs eine ' ...
             'zusaetzliche, kollisionsfreie Zwischenkonfiguration einfuegen.\n'], zLift);
end

% 3) CP-Trajektorie (q_cart) Sample fuer Sample pruefen
fprintf('\n--- Kollisionspruefung: CP-Trajektorie (q_cart) ---\n');
nCollCP = 0;
for k = 1:numCart
    if isSelfCollision(gen3, q_cart(:,k))
        warning('Selbstkollision bei CP-Sample %d (t = %.2f s)', k, t_cart(k));
        if nCollCP == 0
            reportSelfCollision(gen3, q_cart(:,k), sprintf('CP-Sample %d (t = %.2f s)', k, t_cart(k)));
        end
        nCollCP = nCollCP + 1;
    end
end
if nCollCP == 0
    fprintf('CP-Trajektorie ist kollisionsfrei (%d Samples geprueft).\n', numCart);
else
    fprintf('CP-Trajektorie: %d von %d Samples kollidieren!\n', nCollCP, numCart);
    fprintf(['Einfache Gegenmassnahme: zLift (oben, aktuell %.2f m) erhoehen, ' ...
             'damit die "_up"-Eckposen der CP-Bahn weiter von der Flasche entfernt sind.\n'], zLift);
end

%% 4.4 Energieverbrauch (aus Joint-Space-Trajektorie)
gen3.Gravity = [0 0 -9.81];

tau = zeros(7, numSamples);
for k = 1:numSamples
    tau(:,k) = inverseDynamics(gen3, q_traj(:,k), qd_traj(:,k), qdd_traj(:,k));
end

P = sum(tau .* qd_traj, 1);   % Gesamtleistung aller Gelenke
E = cumtrapz(t_traj, P);      % kumulierte Netto-Energie

% Die Trajektorie ist ein geschlossener Kreislauf (Start = Ende = q_home,
% jeweils mit qd = 0). Daher gleichen sich Heben/Beschleunigen (P>0) und
% Senken/Abbremsen (P<0) in der Netto-Bilanz E(end) ~ 0 exakt aus - das
% entspricht einem Antrieb mit perfekter Rueckspeisung (Rekuperation).
% Ein realer Antrieb kann die Bremsenergie meist nicht zurueckgewinnen.
% Realistischer "Energieverbrauch" = nur die positiven Leistungsanteile.
P_pos = max(P, 0);
E_pos = cumtrapz(t_traj, P_pos);

fprintf('\n=== Energiebilanz (Assignment 3, Gesamttrajektorie) ===\n');
fprintf('Netto-Energie E(end):                              %.2f J  (Start = Ende -> ~0)\n', E(end));
fprintf('Energieverbrauch (nur Antreiben, ohne Rueckspeisung): %.2f J\n', E_pos(end));

fig5 = figure;
subplot(2,1,1);
plot(t_traj, P, 'LineWidth', 1.5); grid on;
xlabel('t [s]'); ylabel('P [W]');
title('Assignment 3 - Power P(t)');
for i = 1:numel(event_times)
    xline(event_times(i), '--', event_labels{i});
end

subplot(2,1,2);
plot(t_traj, E_pos, 'LineWidth', 1.5); grid on;
xlabel('t [s]'); ylabel('E [J]');
title(sprintf('Energy consumption (no regeneration) - Total: %.2f J', E_pos(end)));
for i = 1:numel(event_times)
    xline(event_times(i), '--', event_labels{i});
end

%% 4.4 Bonus: Energievergleich PTP vs. CP (B1: F1 -> F3, mit Lift)
% Gleiches Segment wie die CP-Bahn oben: PTP-Wegpunkte 9-12
% (qF1 -> qF1_up -> qF3_up -> qF3), Dauer numCartSeg*seg_time.
idxPTP    = (t_traj >= event_times(3)) & (t_traj <= event_times(4));
E_ptp_seg = trapz(t_traj(idxPTP), P(idxPTP));

dt_cart  = t_cart(2) - t_cart(1);             % gleichmaessiges Zeitgitter
qd_cart  = zeros(size(q_cart));
qdd_cart = zeros(size(q_cart));
for j = 1:7
    qd_cart(j,:)  = gradient(q_cart(j,:), dt_cart);
    qdd_cart(j,:) = gradient(qd_cart(j,:), dt_cart);
end

tau_cart = zeros(7, numCart);
for k = 1:numCart
    tau_cart(:,k) = inverseDynamics(gen3, q_cart(:,k), qd_cart(:,k), qdd_cart(:,k));
end
P_cart     = sum(tau_cart .* qd_cart, 1);
E_cart_seg = trapz(t_cart, P_cart);

fprintf('\n=== Energievergleich B1: F1 -> F3 (mit Lift) ===\n');
fprintf('PTP (Joint Space):     %.2f J\n', E_ptp_seg);
fprintf('CP  (Cartesian Space): %.2f J\n', E_cart_seg);

%% Funktionen

function T = forwardKin(q, DH, useTool)
    % useTool (Default: true) steuert den Tool-Offset (Aufgabe 3.2.4):
    %   true  -> T zeigt auf die Werkzeugspitze (12 cm vor dem Interface-Frame)
    %   false -> T zeigt auf den Interface-Frame ('EndEffector_Link'),
    %            zum Vergleich mit getTransform (3.2.6)
    if nargin < 3
        useTool = true;
    end

    % Basis-Frame (Zeile 1 der DH-Tabelle, fest, kein Gelenk)
    T = dhFrame(DH(1,3), DH(1,2), 0, DH(1,1));
    % Gelenke 1-7 (Zeilen 2-8)
    for i = 1:7
        theta = q(i) + DH(i+1, 3);          % q_i + theta-Offset
        T = T * dhFrame(theta, DH(i+1,2), 0, DH(i+1,1));
    end

    if useTool
        % Tool-Offset: v = [0 0 0.12]^T im Interface-Frame (Aufgabe 3.2.4)
        T = T * [eye(3), [0; 0; 0.12]; 0 0 0 1];
    end
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

function tf = isSelfCollision(robot, q)
    % Prueft EINE Konfiguration auf Selbstkollision.
    % 'SkippedSelfCollisions','parent' ignoriert Kollisionen zwischen direkt
    % benachbarten Gliedern, die sich am gemeinsamen Gelenk immer beruehren
    % (sonst wuerde checkCollision staendig Fehlalarm geben).
    tf = checkCollision(robot, q, 'SkippedSelfCollisions', 'parent');
end

function reportSelfCollision(robot, q, label)
    % Diagnose-Hilfsfunktion: gibt aus, WELCHE Koerperpaare bei der
    % Konfiguration q kollidieren und wie tief sie sich durchdringen.
    % sepDist(a,b) < 0 bedeutet Durchdringung um |sepDist(a,b)| Meter.
    % Hilft zu unterscheiden, ob es eine "echte" grosse Kollision ist
    % oder nur ein winziges Ueberlappen der (konservativen) Kollisionsmeshes.
    [isColl, sepDist] = checkCollision(robot, q, 'SkippedSelfCollisions', 'parent');
    if ~isColl
        return;
    end
    names = [{'world'}, robot.BodyNames];
    fprintf('  -> Kollidierende Koerperpaare bei %s:\n', label);
    n = size(sepDist, 1);
    for a = 1:n
        for b = a+1:n
            d = sepDist(a,b);
            if ~isnan(d) && d < 0
                fprintf('     %s <-> %s : Durchdringung %.4f m\n', names{a}, names{b}, -d);
            end
        end
    end
end

function q_up = liftConfig(robot, ik, eeName, weights, q, zLift)
    % Berechnet eine um zLift in Welt-z angehobene "Darueber"-Pose per IK.
    % Orientierung bleibt erhalten (nur T(3,4) += zLift); q dient als
    % Warmstart, damit die Loesung nah an der Ausgangskonfiguration bleibt.
    T = getTransform(robot, q, eeName);
    T(3,4) = T(3,4) + zLift;
    % wrapToPi(q) als IK-Startwert: qF1/qF2/qF3 liegen teilweise ausserhalb
    % von [-pi, pi] (z.B. 299.74 Grad), was sonst die Gelenkgrenzen-Warnung
    % von inverseKinematics ausloest. wrapToPi liefert dieselbe Pose
    % (physikalisch identisches Gelenk), aber innerhalb der Grenzen.
    [q_up, info] = ik(eeName, T, weights, wrapToPi(q));
    if ~strcmp(info.Status, "success")
        warning('liftConfig: IK fuer angehobene Pose nicht konvergiert (Status: %s)', info.Status);
    end
end

function qv = withGripper(q, qVisTemplate)
    % Fuegt die 7 Arm-Gelenkwerte q in die Konfiguration von gen3_vis ein;
    % die Greifer-Gelenke bleiben dabei auf ihrem Home-Wert (qVisTemplate)
    qv = qVisTemplate;
    qv(1:numel(q)) = q;
end

% --- Hilfsfunktionen fuer Assignment 1 ---

function drawFrame(O, R, label)
    scale = 0.1;
    colors = {'r', 'g', 'b'};
    for i = 1:3
        ep = O + scale * R(:,i);
        plot3([O(1) ep(1)], [O(2) ep(2)], [O(3) ep(3)], colors{i}, 'LineWidth', 2);
    end
    if nargin >= 3
        text(O(1), O(2), O(3), ['  ' label], 'FontWeight', 'bold');
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
