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