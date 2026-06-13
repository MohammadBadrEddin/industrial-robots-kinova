% Rotation of frame around x-axis

function R = rotX(angle_deg)
    a = angle_deg;
    R = [1      0       0;
         0  cosd(a) -sind(a);
         0  sind(a) cosd(a)];
end









