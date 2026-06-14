%% =========================================================
%  ASSIGNMENT 2 - Forward Kinematics & Manipulability
%  Kinova Gen3 7-DoF Robot
%  Covers ALL required points from the lab sheet:
%  3.2 (1-6): Build FK, implement it, compare with Kinova
%  3.3 (1-3): Compute vm, plot it, interpret it
% ==========================================================
clear; clc; close all;
%% ---- Load the robot digital twin -------------------------
gen3 = loadrobot("kinovaGen3");
gen3.DataFormat = 'column';
eeName = 'EndEffector_Link';
%% ---- Define the three target configurations (degrees→rad)-
qF1 = wrapToPi([16.39; 299.74;  5.10; 268.15;  22.51;  63.48;  71.20] * pi/180);
qF2 = wrapToPi([351.73; 300.69;  8.11; 263.95; 356.98;  63.00;  79.94] * pi/180);
qF3 = wrapToPi([  1.18; 291.32; 18.47; 290.91;  94.36; 112.93;  46.00] * pi/180);
configs  = {qF1, qF2, qF3};
cfgNames = {'qF1', 'qF2', 'qF3'};
%% =========================================================
%  ASSIGNMENT 3.2 - Point 6:
%  Compare our FK with Kinova's built-in FK at each config
% ==========================================================
fprintf('===================================================\n');
fprintf(' 3.2 (6): FK COMPARISON - Our FK vs Kinova FK\n');
fprintf('===================================================\n');
for i = 1:3
    q = configs{i};
    T_myFK    = kinovafk(q);                      % my derived FK
    T_kinova = getTransform(gen3, q, eeName);    % Kinova built-in FK
    
    R_myFK = T_myFK(1:3, 1:3);
    R_kinova = T_kinova(1:3, 1:3);
    pos_myFK    = T_myFK(1:3, 4);
    pos_kinova = T_kinova(1:3, 4);
    
    R_err    = R_myFK' * R_kinova;
    ang_err  = acos((trace(R_err) - 1) / 2);  % angle of the residual rotation [rad]
    err        = norm(pos_myFK - pos_kinova);
    % if ang_err < 0.01
    %     fprintf('  ✓ ORIENTATION MATCH\n');
    % else
    %     fprintf('  ✗ ORIENTATION MISMATCH\n');
    % end
    fprintf('\n  Config %s:\n', cfgNames{i});
    fprintf('My FK    position [x,y,z]: %7.4f  %7.4f  %7.4f  m\n', pos_myFK);
    fprintf('Kinova FK position [x,y,z]: %7.4f  %7.4f  %7.4f  m\n', pos_kinova);
    fprintf('Orientation error (angle): %.6f rad  (%.4f deg)\n', ang_err, ang_err*180/pi);
    fprintf('Position error (norm):      %.6f m', err);
    % if err < 0.002
    %     fprintf('  ✓ MATCH\n');
    % else
    %     fprintf('  ✗ MISMATCH - check T matrices!\n');
    % end
end
%% =========================================================
%  ASSIGNMENT 3.3 - Manipulability monitoring
%  Continuous motion: home → qF1 → qF2 → qF3 → home
%  At every step: compute and plot vm (magenta arrow)
%  At each waypoint: print Yoshikawa measure mu
% ==========================================================
fprintf('\n===================================================\n');
fprintf(' 3.3: MANIPULABILITY MONITORING\n');
fprintf('===================================================\n');
figure('Name','Assignment 2 - FK and Manipulability Monitoring', ...
       'NumberTitle','off','Position',[50 50 1200 900]);
% -- full sequence of waypoints: home -> qF1 -> qF2 -> qF3 -> home --
q_home    = gen3.homeConfiguration;
waypoints = [q_home, qF1, qF2, qF3, q_home];
wpNames   = {'home', 'qF1', 'qF2', 'qF3', 'home'};
N = 60;   % steps per segment
mu_at_waypoint = zeros(1, size(waypoints,2));  % mu recorded at each waypoint
for seg = 1:size(waypoints,2)-1
    q_start = waypoints(:, seg);
    q_end   = waypoints(:, seg+1);
    % -- build linear joint-space interpolation for this segment --
    Q = zeros(7, N);
    for k = 1:7
        Q(k,:) = linspace(q_start(k), q_end(k), N);
    end
    fprintf('\n  Animating motion %s -> %s ...\n', wpNames{seg}, wpNames{seg+1});
    mu_values = zeros(1, N);   % store Yoshikawa measure over this segment
    
    hArrow = []; % Initialize persistent arrow handle for this segment
    
    for step = 1:N
        q_cur = Q(:, step);
        % ---- show robot posture ----------------------------------
        show(gen3, q_cur, 'PreservePlot', false, 'FastUpdate', true);
        hold on; grid on; axis equal; view(3);
        
        % Lock axis boundaries so it stays non-adaptive
        xlim([-0.8 0.8]);
        ylim([-0.8 0.8]);
        zlim([-0.1 1.2]);
        
        % ---- PLOT PERSISTENT WAYPOINTS (qF1, qF2, qF3) -----------
        % Get positions of target configs to anchor the plot points
        p_F1 = getTransform(gen3, qF1, eeName); p_F1 = p_F1(1:3, 4);
        p_F2 = getTransform(gen3, qF2, eeName); p_F2 = p_F2(1:3, 4);
        p_F3 = getTransform(gen3, qF3, eeName); p_F3 = p_F3(1:3, 4);
        
        % Draw green circles at target coordinates
        plot3(p_F1(1), p_F1(2), p_F1(3), 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
        text(p_F1(1), p_F1(2), p_F1(3), '  qF1', 'Color', 'g', 'FontWeight', 'bold');
        
        plot3(p_F2(1), p_F2(2), p_F2(3), 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
        text(p_F2(1), p_F2(2), p_F2(3), '  qF2', 'Color', 'g', 'FontWeight', 'bold');
        
        plot3(p_F3(1), p_F3(2), p_F3(3), 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
        text(p_F3(1), p_F3(2), p_F3(3), '  qF3', 'Color', 'g', 'FontWeight', 'bold');
        % ----------------------------------------------------------
        
        xlabel('x'); ylabel('y'); zlabel('z');
        % ---- compute Jacobian and manipulability -----------------
        J    = geometricJacobian(gen3, q_cur, eeName);
        Jv   = J(4:6, :);          
        [U, S, ~] = svd(Jv);
        vm = U(:, 1);              
        % mu_values(step) = prod(diag(S));
        mu_values(step) = S(1,1);
        % ---- get end-effector position ---------------------------
        T_ee = getTransform(gen3, q_cur, eeName);
        p_ee = T_ee(1:3, 4);
        % ---- plot vm in MAGENTA from end-effector origin ---------
        if ~isempty(hArrow) && ishandle(hArrow)
            delete(hArrow);
        end
        scale = 0.15;
        hArrow = quiver3(p_ee(1), p_ee(2), p_ee(3), ...
                vm(1)*scale, vm(2)*scale, vm(3)*scale, ...
                0, 'Color', 'm', 'LineWidth', 2.5, 'MaxHeadSize', 0.8);
        title(sprintf('Moving %s -> %s  |  step %d/%d  |  \\mu = %.4f', ...
              wpNames{seg}, wpNames{seg+1}, step, N, mu_values(step)), 'FontSize', 10);
        hold off;
        drawnow;
        pause(0.02);
    end    
    if ~isempty(hArrow) && ishandle(hArrow)
        delete(hArrow);
    end
    
    % -- record mu at start/end of this segment --
    mu_at_waypoint(seg)   = mu_values(1);
    mu_at_waypoint(seg+1) = mu_values(end);
    pause(0.5);   % brief pause at each waypoint
end
% -- print interpretation across the whole sequence --
fprintf('\n  --- Interpretation across the full sequence ---\n');
for w = 1:numel(wpNames)
    fprintf('  mu at %-5s: %.4f\n', wpNames{w}, mu_at_waypoint(w));
end
fprintf('\n  Robot starts and ends at "home" with mu = %.4f.\n', mu_at_waypoint(1));
fprintf('\n========== Assignment 2 complete ==========\n');
%% =========================================================
%  LOCAL FUNCTION: kinovafk
%  Computes T08 = T01*T12*T23*T34*T45*T56*T67*T78
%  Each Ti encodes Rz(qi) + fixed geometric offset from
%  the Kinova Gen3 frame-placement diagram
% ==========================================================
function FK = kinovafk(q)
q1=q(1); q2=q(2); q3=q(3); q4=q(4); q5=q(5); q6=q(6); q7=q(7);
c1=cos(q1); s1=sin(q1);
c2=cos(q2); s2=sin(q2);
c3=cos(q3); s3=sin(q3);
c4=cos(q4); s4=sin(q4);
c5=cos(q5); s5=sin(q5);
c6=cos(q6); s6=sin(q6);
c7=cos(q7); s7=sin(q7);
% T01: base → actuator 1
% z-offset = 0.1564 m (base height), Rx(180°) flips y and z axes
T1 = [ c1  -s1   0    0   ;
      -s1  -c1   0    0   ;
        0    0  -1   0.1564;
        0    0   0    1   ];
% T12: actuator 1 → actuator 2
% y-offset = 0.0054 m (lateral), z-offset = -0.1284 m
T2 = [ c2  -s2   0    0   ;
        0    0  -1   0.0054;
       s2   c2   0  -0.1284;
        0    0   0    1   ];
% T23: actuator 2 → actuator 3
% y-offset = -0.2104 m (arm length), z = -0.0064 m (lateral)
T3 = [ c3  -s3   0    0   ;
        0    0   1  -0.2104;
      -s3  -c3   0  -0.0064;
        0    0   0    1   ];
% T34: actuator 3 → actuator 4
% y = +0.0064 m (lateral), z = -0.2104 m (arm length)
% O4 is physically above-right of O3, but frame 3's y- and z-axes
T4 = [ c4  -s4   0    0   ;
        0    0  -1  0.0064;
       s4   c4   0  -0.2104;
        0    0   0    1   ];
% T45: actuator 4 → actuator 5
T5 = [ c5  -s5   0    0   ;
        0    0   1  -0.2084;
      -s5  -c5   0  -0.0064;
        0    0   0    1   ];
% T56: actuator 5 → actuator 6
T6 = [ c6  -s6   0    0   ;
        0    0  -1    0   ;
       s6   c6   0  -0.1059;
        0    0   0    1   ];
% T67: actuator 6 → actuator 7
T7 = [ c7  -s7   0    0   ;
        0    0   1  -0.1059;
      -s7  -c7   0    0   ;
        0    0   0    1   ];
% T78: actuator 7 → interface module (no joint angle, pure offset)
T8 = [  1   0   0    0   ;
        0  -1   0    0   ;
        0   0  -1  -0.0615;
        0   0   0    1   ];
% ownOffset: an additional 12 mm tool-tip offset along local z.
% Defined here for completeness/reference, but NOT applied below -
% the Kinova controller's reported EE frame (getTransform) stops at
% T8, so we exclude ownOffset to match it (see 3.2 point 6).
ownOffset = [
    1  0   0  0;
    0  1   0  0;
    0  0   1  0.012;
    0  0   0  1;
    ];
FK = T1*T2*T3*T4*T5*T6*T7*T8;
end