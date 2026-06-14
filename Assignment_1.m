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
