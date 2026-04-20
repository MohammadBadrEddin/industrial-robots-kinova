close all; clc;

%% Roboter vorbereiten
gen3 = loadrobot("kinovaGen3");                % Kinova Gen3 Roboter laden
gen3.DataFormat = 'column';                    % Datenformat auf Spaltenvektor setzen
q_home = [0 15 180 -130 0 55 90]' * pi/180;    % Home-Konfiguration in Bogenmaß
eeName = 'EndEffector_Link';                   % Name des Endeffektors
T_home = getTransform(gen3, q_home, eeName);   % Transformation vom Base zum EE

%% Inverse Kinematik vorbereiten
ik = inverseKinematics('RigidBodyTree', gen3);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1]; % IK-Gewichtung (Translation & Rotation)
q_init = q_home;         % Startkonfiguration für IK

%% Manuelle Zielpositionen eingeben
points = [
    0.5  0.0  0.4;
    0.5  0.1  0.45;
    0.4 -0.1  0.35;
    0.45 0.0  0.3
];

%% Inverse Kinematik berechnen
numJoints = size(q_home,1);
numWaypoints = size(points,1);
qs = zeros(numWaypoints, numJoints); % IK-Lösungen speichern

for i = 1:numWaypoints
    T_des = T_home;
    T_des(1:3,4) = points(i,:)'; % Zielposition einfügen

    [q_sol, ~] = ik(eeName, T_des, weights, q_init); % IK lösen
    qs(i,:) = q_sol(1:numJoints); % Gelenkwinkel speichern
    q_init = q_sol;               % Startwert fürs nächste Ziel
end

%% Visualisierung + Animation
figure;
ax = show(gen3, qs(1,:)');
ax.CameraPositionMode = 'auto';
hold on;

% Zielpunkte visualisieren
plot3(points(:,1), points(:,2), points(:,3), '-o', ...
      'LineWidth', 2, 'MarkerSize', 6, 'Color', [0.1 0.6 0.1]);

xlabel('x'); ylabel('y'); zlabel('z');
title('Manuell definierte Zielpositionen');
grid on;

% === Verbesserte Kameraperspektive ===
axis equal;
xlim([min(points(:,1))-0.1, max(points(:,1))+0.1]);
ylim([min(points(:,2))-0.2, max(points(:,2))+0.2]);
zlim([0, max(points(:,3))+0.2]);
view([60 30]);

% === Animation ===
framesPerSecond = 30;
r = robotics.Rate(framesPerSecond);

for i = 1:numWaypoints
    show(gen3, qs(i,:)', 'PreservePlot', false);
    drawnow;
    waitfor(r);
    pause(1);
end
