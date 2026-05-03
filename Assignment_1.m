clc; clear; close all;

figure; 
hold on; grid on; axis equal; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Frames F0, F1, F2, F3');

scale = 0.1;

T0 = eye(4);
drawFrame(T0, 'F0', scale);
disp('T0')
disp(T0)

T01 = makeFrame(-89, -175, 90, [-0.78; 0.15; 0.21]);
drawFrame(T01, 'F1', scale);
disp('T01')
disp(T01)

[T02 ,T02_inv] = makeFrame(-89, -175, 90, [-0.78; -0.05; 0.21]);
drawFrame(T02, 'F2', scale);
disp('T02')
disp(T02)

T03 = makeFrame(91, -5, 10, [-0.55; -0.2; 0.21]);
drawFrame(T03, 'F3', scale);
disp('T03')
disp(T03)

%% Assignment 2.4: Spatial transformations between frames

% T21: transformation from F1 to F2
T21 = invT(T02) * T01;
disp('T21')
disp(T21)

% T32: transformation from F2 to F3
T32 = invT(T03) * T02;
disp('T32')
disp(T32)

% T20: transformation from F0 to F2
T20 = invT(T02);
disp('T20')
disp(T20)

%% Assignment 2.5: Transform point coordinates

% Given point P3 expressed in frame F3
P3_F3 = [1; 0; 0; 1];

% Given point P2 expressed in frame F2
P2_F2 = [-1; 0; -1; 1];

% -----------------------------------------
% Transform P3 from F3 to F0
% Since T03 maps from F3 to F0:
P3_F0 = T03 * P3_F3;

% Transform P3 from F0 to F1
P3_F1 = invT(T01) * P3_F0;

% -----------------------------------------
% Transform P2 from F2 to F0
% Since T02 maps from F2 to F0:
P2_F0 = T02 * P2_F2;

% Transform P2 from F0 to F1
P2_F1 = invT(T01) * P2_F0;

% Display results
disp('P3 expressed in F0:')
disp(P3_F0(1:3))

disp('P3 expressed in F1:')
disp(P3_F1(1:3))

disp('P2 expressed in F0:')
disp(P2_F0(1:3))

disp('P2 expressed in F1:')
disp(P2_F1(1:3))

xlim([-1 0.2]);
ylim([-0.5 0.5]);
zlim([0 0.5]);

%% Functions

function R = rotX(angle_deg)
    a = angle_deg;
    R = [1      0       0;
         0  cosd(a) -sind(a);
         0  sind(a)  cosd(a)];
end

function R = rotY(angle_deg)
    a = angle_deg;
    R = [ cosd(a) 0 sind(a);
              0   1     0;
         -sind(a) 0 cosd(a)];
end

function R = rotZ(angle_deg)
    a = angle_deg;
    R = [cosd(a) -sind(a) 0;
         sind(a)  cosd(a) 0;
             0       0    1];
end

function [T,T_inv] = makeFrame(rotX_deg, rotY_deg, rotZ_deg, translation)
    R = rotZ(rotZ_deg) * rotY(rotY_deg) * rotX(rotX_deg);

    % Manual homogeneous transformation
    T = [R, translation;
         0, 0, 0, 1];
    T_inv = [R', -R' * translation;
         0, 0, 0, 1];
end

function T_inv = invT(T)
    R = T(1:3, 1:3);
    t = T(1:3, 4);

    % Manual inverse of homogeneous transformation
    T_inv = [R', -R' * t;
             0,   0,    0, 1];
end

function drawFrame(T, name, scale)
    O = T(1:3, 4);

    X = O + scale * T(1:3, 1);
    Y = O + scale * T(1:3, 2);
    Z = O + scale * T(1:3, 3);

    plot3([O(1) X(1)], [O(2) X(2)], [O(3) X(3)], ...
          'r', 'LineWidth', 2)

    plot3([O(1) Y(1)], [O(2) Y(2)], [O(3) Y(3)], ...
          'g', 'LineWidth', 2)

    plot3([O(1) Z(1)], [O(2) Z(2)], [O(3) Z(3)], ...
          'b', 'LineWidth', 2)

    plot3(O(1), O(2), O(3), 'ko', 'MarkerFaceColor', 'k')

    text(O(1), O(2), O(3), ['  ' name], ...
         'FontSize', 10, 'FontWeight', 'bold');
end