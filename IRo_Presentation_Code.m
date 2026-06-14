%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% First Assignment - 3D Modeling: Frame Manipulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear; clc

% -------------------------------------------------------------------------
% 2.2 Preliminaries

% Rotations around [x-, y-, z-axis] in °
O0_rot = [0; 0; 0];
O1_rot = [-89; -175; 90];
O2_rot = [-89; -175; 90];
O3_rot = [91; -5; 10];

% Initial point
O0 = [0; 0; 0];
% Target points
O1 = [-0.78; 0.15; 0.21];
O2 = [-0.78; -0.05; 0.21];
O3 = [-0.55; -0.2; 0.21];

% Calculate translations (here = target point: target point - initial point)
O01 = O1 - O0;
O02 = O2 - O0;
O03 = O3 - O0;


% -------------------------------------------------------------------------
% 2.3 Drawing frames

% Create figure
figure('Name', 'Drawing of frames', 'NumberTitle','off')
hold on
grid on
axis equal
view(3);

% Call function drawFrame(spatial vector for rotation,
% spatial vector for translation, Name of frame, Scale) and retrieve the
% transformation functions (forward & inverse) of the respective target 
% frames with the initial frame
[~, ~] = make_n_drawFrame(O0_rot, O0, 'F0', 0.15);              % Create Frame F0
[T01, T01_inv] = make_n_drawFrame(O1_rot, O01, 'F1', 0.15);     % Create Frame F1
[T02, T02_inv] = make_n_drawFrame(O2_rot, O02, 'F2', 0.15);     % Create Frame F2
[T03, T03_inv] = make_n_drawFrame(O3_rot, O03, 'F3', 0.15);     % Create Frame F3
title('Drawings of F0 to F3')
xlabel('X')
ylabel('Y')
zlabel('Z')
ylim([-0.4 0.4])
zlim([0 0.5]);


% -------------------------------------------------------------------------
% 2.4 Spatial transformation between frames

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('Homogeneous Transformation matrices')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')

% T_01: from F0 to F1 / position of F1 described with F0
disp('T01=')
disp(T01)

% T_21: from F2 to F1 / position of F1 described with F2
T21 = T02_inv * T01;
disp('T21=')
disp(T21)

% T_32: from F3 to F2 / position of F2 described with F3
T32 = T03_inv * T02;
disp('T32=')
disp(T32)

% T_20: from F2 to F0 / position of F0 described with F2
T20 = T02_inv;
disp('T20=')
disp(T20)


% -------------------------------------------------------------------------
% 2.5 Transformation of point coordinates from a frame to another

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('Transformation of point coordinates')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')

% Point coordinates
% add last '1' to meet the spatial length of transformation matrix
P3 = [1; 0; 0; 1];          % "O3P3H": P3 given in respect to F3
P2 = [-1; 0; -1; 1];        % "O2P2H": P2 given in respect to F2

% calculate "O0P3H": P3 given in terms of F0
P3_F0 = T03 * P3;
disp('P3_F0=')
disp(P3_F0(1:3, 1))

% calculate "O1P3H": P3 given in terms of F1
P3_F1 = T01_inv * T03 * P3;
disp('P3_F1=')
disp(P3_F1(1:3, 1))

% calculate "O0P2H": P2 given in terms of F0
P2_F0 = T02 * P2;
disp('P2_F0=')
disp(P2_F0(1:3, 1))

% calculate "O1P2H": P2 given in terms of F1
P2_F1 = T01_inv * T02 * P2;
disp('P2_F1=')
disp(P2_F1(1:3, 1))


%% =========================================================
%  ASSIGNMENT 2 - Forward Kinematics & Manipulability
%  Kinova Gen3 7-DoF Robot
%  Covers ALL required points from the lab sheet:
%  3.2 (1-6): Build FK, implement it, compare with Kinova
%  3.3 (1-3): Compute vm, plot it, interpret it
% ==========================================================
clear; clc; close all;
%% ---- Load the robot digital twin -------------------------
gen3 = loadrobot("kinovaGen3");
gen3.DataFormat = 'column';
eeName = 'EndEffector_Link';
%% ---- Define the three target configurations (degrees→rad)-
qF1 = wrapToPi([16.39; 299.74;  5.10; 268.15;  22.51;  63.48;  71.20] * pi/180);
qF2 = wrapToPi([351.73; 300.69;  8.11; 263.95; 356.98;  63.00;  79.94] * pi/180);
qF3 = wrapToPi([  1.18; 291.32; 18.47; 290.91;  94.36; 112.93;  46.00] * pi/180);
configs  = {qF1, qF2, qF3};
cfgNames = {'qF1', 'qF2', 'qF3'};
%% =========================================================
%  ASSIGNMENT 3.2 - Point 6:
%  Compare our FK with Kinova's built-in FK at each config
% ==========================================================
fprintf('===================================================\n');
fprintf(' 3.2 (6): FK COMPARISON - Our FK vs Kinova FK\n');
fprintf('===================================================\n');
for i = 1:3
    q = configs{i};
    T_myFK    = kinovafk(q);                      % my derived FK
    T_kinova = getTransform(gen3, q, eeName);    % Kinova built-in FK
    
    R_myFK = T_myFK(1:3, 1:3);
    R_kinova = T_kinova(1:3, 1:3);
    pos_myFK    = T_myFK(1:3, 4);
    pos_kinova = T_kinova(1:3, 4);
    
    R_err    = R_myFK' * R_kinova;
    ang_err  = acos((trace(R_err) - 1) / 2);  % angle of the residual rotation [rad]
    err        = norm(pos_myFK - pos_kinova);
    % if ang_err < 0.01
    %     fprintf('  ✓ ORIENTATION MATCH\n');
    % else
    %     fprintf('  ✗ ORIENTATION MISMATCH\n');
    % end
    fprintf('\n  Config %s:\n', cfgNames{i});
    fprintf('My FK    position [x,y,z]: %7.4f  %7.4f  %7.4f  m\n', pos_myFK);
    fprintf('Kinova FK position [x,y,z]: %7.4f  %7.4f  %7.4f  m\n', pos_kinova);
    fprintf('Orientation error (angle): %.6f rad  (%.4f deg)\n', ang_err, ang_err*180/pi);
    fprintf('Position error (norm):      %.6f m', err);
    % if err < 0.002
    %     fprintf('  ✓ MATCH\n');
    % else
    %     fprintf('  ✗ MISMATCH - check T matrices!\n');
    % end
end


%% =========================================================
%  ASSIGNMENT 3.3 - Manipulability monitoring
%  Continuous motion: home → qF1 → qF2 → qF3 → home
%  At every step: compute and plot vm (magenta arrow)
%  At each waypoint: print Yoshikawa measure mu
% ==========================================================
fprintf('\n===================================================\n');
fprintf(' 3.3: MANIPULABILITY MONITORING\n');
fprintf('===================================================\n');
figure('Name','Assignment 2 - FK and Manipulability Monitoring', ...
       'NumberTitle','off','Position',[50 50 1200 900]);
% -- full sequence of waypoints: home -> qF1 -> qF2 -> qF3 -> home --
q_home    = gen3.homeConfiguration;
waypoints = [q_home, qF1, qF2, qF3, q_home];
wpNames   = {'home', 'qF1', 'qF2', 'qF3', 'home'};
N = 60;   % steps per segment
mu_at_waypoint = zeros(1, size(waypoints,2));  % mu recorded at each waypoint
for seg = 1:size(waypoints,2)-1
    q_start = waypoints(:, seg);
    q_end   = waypoints(:, seg+1);
    % -- build linear joint-space interpolation for this segment --
    Q = zeros(7, N);
    for k = 1:7
        Q(k,:) = linspace(q_start(k), q_end(k), N);
    end
    fprintf('\n  Animating motion %s -> %s ...\n', wpNames{seg}, wpNames{seg+1});
    mu_values = zeros(1, N);   % store Yoshikawa measure over this segment
    
    hArrow = []; % Initialize persistent arrow handle for this segment
    
    for step = 1:N
        q_cur = Q(:, step);
        % ---- show robot posture ----------------------------------
        show(gen3, q_cur, 'PreservePlot', false, 'FastUpdate', true);
        hold on; grid on; axis equal; view(3);
        
        % Lock axis boundaries so it stays non-adaptive
        xlim([-0.8 0.8]);
        ylim([-0.8 0.8]);
        zlim([-0.1 1.2]);
        
        % ---- PLOT PERSISTENT WAYPOINTS (qF1, qF2, qF3) -----------
        % Get positions of target configs to anchor the plot points
        p_F1 = getTransform(gen3, qF1, eeName); p_F1 = p_F1(1:3, 4);
        p_F2 = getTransform(gen3, qF2, eeName); p_F2 = p_F2(1:3, 4);
        p_F3 = getTransform(gen3, qF3, eeName); p_F3 = p_F3(1:3, 4);
        
        % Draw green circles at target coordinates
        plot3(p_F1(1), p_F1(2), p_F1(3), 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
        text(p_F1(1), p_F1(2), p_F1(3), '  qF1', 'Color', 'g', 'FontWeight', 'bold');
        
        plot3(p_F2(1), p_F2(2), p_F2(3), 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
        text(p_F2(1), p_F2(2), p_F2(3), '  qF2', 'Color', 'g', 'FontWeight', 'bold');
        
        plot3(p_F3(1), p_F3(2), p_F3(3), 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
        text(p_F3(1), p_F3(2), p_F3(3), '  qF3', 'Color', 'g', 'FontWeight', 'bold');
        % ----------------------------------------------------------
        
        xlabel('x'); ylabel('y'); zlabel('z');
        % ---- compute Jacobian and manipulability -----------------
        J    = geometricJacobian(gen3, q_cur, eeName);
        Jv   = J(4:6, :);          
        [U, S, ~] = svd(Jv);
        vm = U(:, 1);              
        % mu_values(step) = prod(diag(S));
        mu_values(step) = S(1,1);
        % ---- get end-effector position ---------------------------
        T_ee = getTransform(gen3, q_cur, eeName);
        p_ee = T_ee(1:3, 4);
        % ---- plot vm in MAGENTA from end-effector origin ---------
        if ~isempty(hArrow) && ishandle(hArrow)
            delete(hArrow);
        end
        scale = 0.15;
        hArrow = quiver3(p_ee(1), p_ee(2), p_ee(3), ...
                vm(1)*scale, vm(2)*scale, vm(3)*scale, ...
                0, 'Color', 'm', 'LineWidth', 2.5, 'MaxHeadSize', 0.8);
        title(sprintf('Moving %s -> %s  |  step %d/%d  |  \\mu = %.4f', ...
              wpNames{seg}, wpNames{seg+1}, step, N, mu_values(step)), 'FontSize', 10);
        hold off;
        drawnow;
        pause(0.02);
    end    
    if ~isempty(hArrow) && ishandle(hArrow)
        delete(hArrow);
    end
    
    % -- record mu at start/end of this segment --
    mu_at_waypoint(seg)   = mu_values(1);
    mu_at_waypoint(seg+1) = mu_values(end);
    pause(0.5);   % brief pause at each waypoint
end
% -- print interpretation across the whole sequence --
fprintf('\n  --- Interpretation across the full sequence ---\n');
for w = 1:numel(wpNames)
    fprintf('  mu at %-5s: %.4f\n', wpNames{w}, mu_at_waypoint(w));
end
fprintf('\n  Robot starts and ends at "home" with mu = %.4f.\n', mu_at_waypoint(1));
fprintf('\n========== Assignment 2 complete ==========\n');


%% =========================================================
%% ASSIGNMENT 3 – Dynamics & Sensors: Pick & Place + Energieverbrauch
%% =========================================================
close all; clear; clc

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

%% Konfigurationen (Grad -> Bogenmass)
q_home = [0;    15;    180;  -130;    0;    55;   90 ] * pi/180;
qF1    = [ 16.39;  299.74;   5.10; 268.15;  22.51;  63.48;  71.20] * pi/180;
qF2    = [351.73;  300.69;   8.11; 263.95; 356.98;  63.00;  79.94] * pi/180;
qF3    = [  1.18;  291.32;  18.47; 290.91;  94.36; 112.93;  46.00] * pi/180;

configs = {q_home, qF1, qF2, qF3};
labels  = {'Home', 'qF1', 'qF2', 'qF3'};

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

%% Functions

%--------------------------------------------------------------------------
% For Assignment 1
%--------------------------------------------------------------------------

% Create a homogeneous transformation matrix (HT-matrix)
function [T, T_inv] = makeFrame(rotX_deg, rotY_deg, rotZ_deg, translation)

    % multiplication from right to left => R = Rz * Ry * Rx    
    R = rotZ(rotZ_deg) * rotY(rotY_deg) * rotX(rotX_deg);   % achieve complete rotation

    % create HT-matrix by putting together the rotation matrix, translation
    % vector & the lowest row: 0 0 0 1
    T = [R, translation; 0 0 0 1]; 
    
    % create an inverse HT-matrix
    T_inv = [R', -R' * translation; 
             0 0 0 1];
end

% Creation & plot of a Frame
function [T, T_inv] = make_n_drawFrame(R,O, Framename, scale)

    % scale: optional factor to adjust length of frame-axis (default: 1)
    if nargin < 4   % MATLAB-func. to check, if <4 inputs were given
        scale = 1;  % default value, if no scale input is given
    end

    % make the frame
    % R(1): Rotation [in °] around x-axis
    % R(2): Rotation [in °] around y-axis
    % R(3): Rotation [in °] around z-axis
    % O: translation vector
    % Frame: Name of Frame
    [T,T_inv] = makeFrame(R(1), R(2), R(3), O);
    
    % draw the frame
    X = scale * T(1:3, 1) + O;  % Aufmultiplizierte Verdrehung um x-Achse (neue Punktkoordinaten) + die zugrundeliegende Translation (ansonsten liegen Punktkoordinaten woanders
    Y = scale * T(1:3, 2) + O;  % Aufmultiplizierte Verdrehung um y-Achse (neue Punktkoordinaten) + die zugrundeliegende Translation (ansonsten liegen Punktkoordinaten woanders
    Z = scale * T(1:3, 3) + O;  % Aufmultiplizierte Verdrehung um z-Achse (neue Punktkoordinaten) + die zugrundeliegende Translation (ansonsten liegen Punktkoordinaten woanders

    grid on; axis equal; view(3);
    C1 = plot3([O(1) X(1)], [O(2) X(2)], [O(3) X(3)], 'r', 'LineWidth', 2);
    C2 = plot3([O(1) Y(1)], [O(2) Y(2)], [O(3) Y(3)], 'g', 'LineWidth', 2);
    C3 = plot3([O(1) Z(1)], [O(2) Z(2)], [O(3) Z(3)], 'b', 'LineWidth', 2);
    text(O(1), O(2), O(3), Framename)
    legend([C1 C2 C3],{'X Scalar', 'Y Scalar', 'Z Scalar'})
end

% Rotation of frame around x-axis
function R = rotX(angle_deg)
    a = angle_deg;
    R = [1      0       0;
         0  cosd(a) -sind(a);
         0  sind(a) cosd(a)];
end

% Rotation of frame around y-axis
function R = rotY(angle_deg)
    a = angle_deg;
    R = [cosd(a)    0   sind(a);
         0          1   0;
         -sind(a)   0   cosd(a)];
end

% Rotation of frame around z-axis
function R = rotZ(angle_deg)
    a = angle_deg;
    R = [cosd(a)    -sind(a)    0;
         sind(a)    cosd(a)     0;
         0          0           1];
end


%--------------------------------------------------------------------------
% For Assignment 2
%--------------------------------------------------------------------------

%  LOCAL FUNCTION: kinovafk
%  Computes T08 = T01*T12*T23*T34*T45*T56*T67*T78
%  Each Ti encodes Rz(qi) + fixed geometric offset from
%  the Kinova Gen3 frame-placement diagram
function FK = kinovafk(q)
q1=q(1); q2=q(2); q3=q(3); q4=q(4); q5=q(5); q6=q(6); q7=q(7);
c1=cos(q1); s1=sin(q1);
c2=cos(q2); s2=sin(q2);
c3=cos(q3); s3=sin(q3);
c4=cos(q4); s4=sin(q4);
c5=cos(q5); s5=sin(q5);
c6=cos(q6); s6=sin(q6);
c7=cos(q7); s7=sin(q7);
% T01: base → actuator 1
% z-offset = 0.1564 m (base height), Rx(180°) flips y and z axes
T1 = [ c1  -s1   0    0   ;
      -s1  -c1   0    0   ;
        0    0  -1   0.1564;
        0    0   0    1   ];
% T12: actuator 1 → actuator 2
% y-offset = 0.0054 m (lateral), z-offset = -0.1284 m
T2 = [ c2  -s2   0    0   ;
        0    0  -1   0.0054;
       s2   c2   0  -0.1284;
        0    0   0    1   ];
% T23: actuator 2 → actuator 3
% y-offset = -0.2104 m (arm length), z = -0.0064 m (lateral)
T3 = [ c3  -s3   0    0   ;
        0    0   1  -0.2104;
      -s3  -c3   0  -0.0064;
        0    0   0    1   ];
% T34: actuator 3 → actuator 4
% y = +0.0064 m (lateral), z = -0.2104 m (arm length)
% O4 is physically above-right of O3, but frame 3's y- and z-axes
T4 = [ c4  -s4   0    0   ;
        0    0  -1  0.0064;
       s4   c4   0  -0.2104;
        0    0   0    1   ];
% T45: actuator 4 → actuator 5
T5 = [ c5  -s5   0    0   ;
        0    0   1  -0.2084;
      -s5  -c5   0  -0.0064;
        0    0   0    1   ];
% T56: actuator 5 → actuator 6
T6 = [ c6  -s6   0    0   ;
        0    0  -1    0   ;
       s6   c6   0  -0.1059;
        0    0   0    1   ];
% T67: actuator 6 → actuator 7
T7 = [ c7  -s7   0    0   ;
        0    0   1  -0.1059;
      -s7  -c7   0    0   ;
        0    0   0    1   ];
% T78: actuator 7 → interface module (no joint angle, pure offset)
T8 = [  1   0   0    0   ;
        0  -1   0    0   ;
        0   0  -1  -0.0615;
        0   0   0    1   ];
% ownOffset: an additional 12 mm tool-tip offset along local z.
% Defined here for completeness/reference, but NOT applied below -
% the Kinova controller's reported EE frame (getTransform) stops at
% T8, so we exclude ownOffset to match it (see 3.2 point 6).
ownOffset = [
    1  0   0  0;
    0  1   0  0;
    0  0   1  0.012;
    0  0   0  1;
    ];
FK = T1*T2*T3*T4*T5*T6*T7*T8;
end


%--------------------------------------------------------------------------
% For Assignment 3
%--------------------------------------------------------------------------

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