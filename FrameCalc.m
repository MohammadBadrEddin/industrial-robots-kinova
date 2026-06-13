clc, clear all, close all;

figure; hold on; grid on; axis equal; view(3);
T0 = eye(4);
drawFrame(T0);  
disp('T0')
disp(T0)

T01 = makeFrame(-89, -175, 90, [-0.78; 0.15; 0.21]);
drawFrame(T01);     
disp('T01')
disp(T01)

T02 = makeFrame(-89, -175, 90, [-0.78; -0.05; 0.21]);
drawFrame(T02);     
disp('T02')
disp(T02)

T03 = makeFrame(91, -5, 10, [-0.55; -0.2; 0.21]);
drawFrame(T03);     
disp('T03')
disp(T03)
 

% Funktionen
function R = rotX(angle_deg)
    a = angle_deg;
    R = [1      0       0;
         0  cosd(a) -sind(a);
         0  sind(a)  cosd(a)];
end

function R = rotY(angle_deg)
    a = angle_deg;
    R = [ cosd(a) 0 sind(a);
              0  1      0;
         -sind(a) 0 cosd(a)];
end

function R = rotZ(angle_deg)
    a = angle_deg;
    R = [cosd(a) -sind(a) 0;
         sind(a)  cosd(a) 0;
             0       0  1];
end

function drawFrame(T)
    O = T(1:3, 4);
    X = O + T(1:3, 1);
    Y = O + T(1:3, 2);
    Z = O + T(1:3, 3);
   
    plot3([O(1) X(1)], [O(2) X(2)], [O(3) X(3)], 'r', 'LineWidth', 2)
    plot3([O(1) Y(1)], [O(2) Y(2)], [O(3) Y(3)], 'g', 'LineWidth', 2)
    plot3([O(1) Z(1)], [O(2) Z(2)], [O(3) Z(3)], 'b', 'LineWidth', 2)
end

function T = makeFrame(rotX_deg, rotY_deg, rotZ_deg, translation)
    R = rotZ(rotZ_deg) * rotY(rotY_deg) * rotX(rotX_deg);
    T = [R, translation; 0 0 0 1];
end