% Rotation of frame around z-axis

function R = rotZ(angle_deg)
    a = angle_deg;
    R = [cosd(a)    -sind(a)    0;
         sind(a)    cosd(a)     0;
         0          0           1];
end