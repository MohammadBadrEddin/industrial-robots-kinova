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