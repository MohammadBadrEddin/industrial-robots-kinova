% Rotation of frame around y-axis

function R = rotY(angle_deg)
    a = angle_deg;
    R = [cosd(a)    0   sind(a);
         0          1   0;
         -sind(a)   0   cosd(a)];
end