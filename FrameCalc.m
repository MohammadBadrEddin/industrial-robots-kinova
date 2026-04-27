% =========================================================
% Assignment 1 - 3D Modeling: Frame Manipulation
% =========================================================
clc, clear all, close all;

scale_Frame = 0.1;  % Skalierung der Frame-Achsen

figure; hold on; grid on; axis equal; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Frames F0, F1, F2, F3');

% ---------------------------------------------------------
% Section 2.2 - Weltframe F0 (Ursprung, keine Transformation)
% ---------------------------------------------------------
T0 = eye(4);
drawFrame(T0, 'F0', scale_Frame);
disp('T0:'); disp(T0)

% ---------------------------------------------------------
% Section 2.2 - Frame F1
% Rotation: -89° um X, -175° um Y, 90° um Z
% Translation: [-0.78; 0.15; 0.21]
% ---------------------------------------------------------
[T01, T01_inv] = makeFrame(-89, -175, 90, [-0.78; 0.15; 0.21]);
drawFrame(T01, 'F1', scale_Frame);
disp('T01:'); disp(T01)

% ---------------------------------------------------------
% Section 2.2 - Frame F2
% Rotation: -89° um X, -175° um Y, 90° um Z
% Translation: [-0.78; -0.05; 0.21]
% ---------------------------------------------------------
[T02, T02_inv] = makeFrame(-89, -175, 90, [-0.78; -0.05; 0.21]);
drawFrame(T02, 'F2', scale_Frame);
disp('T02:'); disp(T02)

% ---------------------------------------------------------
% Section 2.2 - Frame F3
% Rotation: 91° um X, -5° um Y, 10° um Z
% Translation: [-0.55; -0.2; 0.21]
% ---------------------------------------------------------
[T03, T03_inv] = makeFrame(91, -5, 10, [-0.55; -0.2; 0.21]);
drawFrame(T03, 'F3', scale_Frame);
disp('T03:'); disp(T03)

% ---------------------------------------------------------
% Section 2.4 - Homogene Transformationen zwischen Frames
% Tij = Transformation von Fj nach Fi
% ---------------------------------------------------------

% T21: von F2 nach F1 (F2_inv * F1)
T21 = T02_inv * T01;
disp('T21 (F2 -> F1):'); disp(T21)

% T32: von F3 nach F2 (F3_inv * F2)
T32 = T03_inv * T02;
disp('T32 (F3 -> F2):'); disp(T32)

% T20: von F2 nach F0 (Inverse von T02)
T20 = T02_inv;
disp('T20 (F2 -> F0):'); disp(T20)

% ---------------------------------------------------------
% Section 2.5 - Punkttransformationen
% P3 = [1;0;0] in F3, P2 = [-1;0;-1] in F2
% Gesucht: Koordinaten in F1 und F0
% ---------------------------------------------------------

% Punkte in homogenen Koordinaten
P3 = [1; 0; 0; 1];
P2 = [-1; 0; -1; 1];

% P3 in F0: T03 * P3
P3_F0 = T03 * P3;
disp('P3 in F0:'); disp(P3_F0(1:3))

% P2 in F0: T02 * P2
P2_F0 = T02 * P2;
disp('P2 in F0:'); disp(P2_F0(1:3))

% P3 in F1: T01_inv * T03 * P3
P3_F1 = T01_inv * T03 * P3;
disp('P3 in F1:'); disp(P3_F1(1:3))

% P2 in F1: T01_inv * T02 * P2
P2_F1 = T01_inv * T02 * P2;
disp('P2 in F1:'); disp(P2_F1(1:3))

% Punkte im Plot darstellen
plot3(P3_F0(1), P3_F0(2), P3_F0(3), 'ms', 'MarkerSize', 10, 'MarkerFaceColor', 'm')
text(P3_F0(1), P3_F0(2), P3_F0(3), '  P3(F0)', 'Color', 'm')

plot3(P2_F0(1), P2_F0(2), P2_F0(3), 'cs', 'MarkerSize', 10, 'MarkerFaceColor', 'c')
text(P2_F0(1), P2_F0(2), P2_F0(3), '  P2(F0)', 'Color', 'c')

plot3(P3_F1(1), P3_F1(2), P3_F1(3), 'ms', 'MarkerSize', 10, 'MarkerFaceColor', 'm')
text(P3_F1(1), P3_F1(2), P3_F1(3), '  P3(F1)', 'Color', 'm')

plot3(P2_F1(1), P2_F1(2), P2_F1(3), 'cs', 'MarkerSize', 10, 'MarkerFaceColor', 'c')
text(P2_F1(1), P2_F1(2), P2_F1(3), '  P2(F1)', 'Color', 'c')

% =========================================================
% Funktionen
% =========================================================

function R = rotX(angle_deg)
    R = [1      0            0;
         0  cosd(angle_deg) -sind(angle_deg);
         0  sind(angle_deg)  cosd(angle_deg)];
end

function R = rotY(angle_deg)
    R = [ cosd(angle_deg) 0 sind(angle_deg);
                0         1       0;
         -sind(angle_deg) 0 cosd(angle_deg)];
end

function R = rotZ(angle_deg)
    R = [cosd(angle_deg) -sind(angle_deg) 0;
         sind(angle_deg)  cosd(angle_deg) 0;
                0               0         1];
end

function drawFrame(T, name, scale)
    % Ursprung und Achsen aus T extrahieren
    O = T(1:3, 4);
    X = O + scale * T(1:3, 1);  % X-Achse (rot)
    Y = O + scale * T(1:3, 2);  % Y-Achse (grün)
    Z = O + scale * T(1:3, 3);  % Z-Achse (blau)

    plot3([O(1) X(1)], [O(2) X(2)], [O(3) X(3)], 'r', 'LineWidth', 2)
    plot3([O(1) Y(1)], [O(2) Y(2)], [O(3) Y(3)], 'g', 'LineWidth', 2)
    plot3([O(1) Z(1)], [O(2) Z(2)], [O(3) Z(3)], 'b', 'LineWidth', 2)
    plot3(O(1), O(2), O(3), 'ko', 'MarkerFaceColor', 'k')
    text(O(1), O(2), O(3), ['  ' name], 'FontSize', 10, 'FontWeight', 'bold')
end

function [T, T_inv] = makeFrame(rotX_deg, rotY_deg, rotZ_deg, translation)
    % Rotation: erst X, dann Y, dann Z (von rechts nach links)
    R = rotZ(rotZ_deg) * rotY(rotY_deg) * rotX(rotX_deg);
    T = [R, translation; 0 0 0 1];
    % Inverse einer homogenen Matrix: [R' , -R'*t]
    T_inv = [R', -R' * translation; 0 0 0 1];
end