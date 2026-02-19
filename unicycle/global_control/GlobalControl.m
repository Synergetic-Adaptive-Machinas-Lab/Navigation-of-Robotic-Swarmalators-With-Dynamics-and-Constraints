%%TotalMovementAndPhaseUpdateCalculation with CBF for Target Point

% Generate a taget trajectory
px = traj(timeStep, 1);
py = traj(timeStep, 2);

% Parameters for CBF
eps = 10;   % Control gain for Lyapunov function
k = 1;     % Control gain for CBF

for i = 1:numBots
    dPos(i) = 0;
    dPosX(i) = 0;
    dPosY(i) = 0;
    dOrientation(i) = 0;
    dPhase(i) = 0;

    Fx = 0;
    Fy = 0;
    for j = 1:numBots
        if(i ~= j)
            % Calculate the distance between two bots
            dx = center(j,1) - center(i,1);
            dy = center(j,2) - center(i,2);
            dist = sqrt(dx^2 + dy^2);
            % Calculate the absolute distance between two bots
            if dist > 2*botRad
                d = dist - 2*botRad;
            else
                d = 0.2*dist;
            end

            % Calculate the force in two direction
            F_attr = (A + J*cos(phase(j) - phase(i))) / dist;
            F_rep  = B / (d^2);     % Make sure bots will never go too close
            F_total = F_attr - F_rep;
           
            Fx = Fx + F_total * dx / numBots;
            Fy = Fy + F_total * dy / numBots;
            F = sqrt(Fx^2 + Fy^2);

            % Calculate linear velocity
            dPos(i) = F; % Use velocity as scalar, not vector!
            dPosX(i) = Fx;
            dPosY(i) = Fy;

            % Calculate phase attraction
            dPhase(i) = dPhase(i) + K*sin(phase(j)-phase(i))/(dist * numBots);
        end
    end
end

% Add Control Barrier Function here

    % Calculate the collective center
    mean_center = mean(center, 1);
    mean_dPosX = mean(dPosX);
    mean_dPosY = mean(dPosY);
    
    radii = vecnorm(mean_center - center, 2, 2);
    radius = max(radii) + botRad;
    
    % Lyapunov function for target point
    V = (mean_center(1) - px)^2 + (mean_center(2) - py)^2;
    
    % Time derivative of V
    LfV = 2 * (mean_center(1) - px) * mean_dPosX + 2 * (mean_center(2) - py) * mean_dPosY;
    LgV_u1 = 2 * (mean_center(1) - px);
    LgV_u2 = 2 * (mean_center(2) - py);
    A_clf = [LgV_u1, LgV_u2, -1];   % -1 is for relaxation variable
    b_clf = -LfV - eps * V;
    
    % Control Barrier Function
    % Construct a constraint for all other obstacles
    A_cbf_obs = [];
    b_cbf_obs = [];

    for j = 1:numObstacles
        % b_j = (abs(mean_center(1) - ox(j))/(a(j) + botRad))^n(j) + ...
        %         (abs(mean_center(2) - oy(j))/(b(j) + botRad))^n(j) - 1;
        % db_dcenterX = n(j) * (abs(mean_center(1) - ox(j))/(a(j) + botRad))^(n(j)-1) ...
        %         * (1/(a(j) + botRad)) * sign(mean_center(1) - ox(j));
        % db_dcenterY = n(j) * (abs(mean_center(2) - oy(j))/(b(j) + botRad))^(n(j)-1) ...
        %         * (1/(b(j) + botRad)) * sign(mean_center(2) - oy(j));
        % Lfb_j = db_dcenterX * mean_dPosX + db_dcenterY * mean_dPosY;
        % Lgb_j_u1 = db_dcenterX;
        % Lgb_j_u2 = db_dcenterY;
        % A_temp = [-Lgb_j_u1, -Lgb_j_u2, 0];    
        % b_temp = Lfb_j + k*b_j;
        b_j  = (mean_center(1) - ox(j))^2 + (mean_center(2) - oy(j))^2 - (radius + (r(j) + 2*botRad))^2;
        distObs_j = sqrt( (mean_center(1) - ox(j))^2 + (mean_center(2) - oy(j))^2 ) - ( radius + r(j) + botRad );
        Lfb_j = 2*(mean_center(1) - ox(j)) * mean_dPosX + 2 * (mean_center(2) - oy(j)) * mean_dPosY;
        Lgb_u1_j = 2*(mean_center(1) - ox(j));
        Lgb_u2_j = 2*(mean_center(2) - oy(j));
        A_temp = [-Lgb_u1_j, -Lgb_u2_j, 0];
        b_temp = Lfb_j + k*b_j;
        
        % Add this constraint to all the constraints
        A_cbf_obs = [A_cbf_obs; A_temp];
        b_cbf_obs = [b_cbf_obs; b_temp];
    end
    
    % Quadratic programming to find control
    % For single barrier
    A_1 = [A_clf; A_cbf_obs; 1, 0, 0; -1, 0, 0; 0, 1, 0; 0, -1, 0]; % The name 'A' has been used
    b_1 = [b_clf; b_cbf_obs; v_max; v_max; v_max; v_max];           % The name 'b' has been used
    H = [2 0 0; 0 2 0; 0 0 0.02];
    F = [0; 0; 0];
    options = optimoptions('quadprog', 'Algorithm', 'interior-point-convex', 'Display', 'off');
    [ctrl, ~] = quadprog(H, F, A_1, b_1, [], [], [], [], [], options);
    
    % Update control velocities
    if ~isempty(ctrl)
        ctrlX = ctrl(1);
        ctrlY = ctrl(2);
        relax = ctrl(3);
    else
        ctrlX = 0;
        ctrlY = 0;
        fprintf("Unsolvable! \n");
    end
    % fprintf('ctrlX: %f\n', ctrlX);
    % fprintf('ctrlY: %f\n', ctrlY);
    % fprintf('relaxation: %f\n', relax);
    
    % Uncomment this to disable control
    % ctrlX = 0;
    % ctrlY = 0;

for i = 1:numBots
    % Calculate the final linear velocity and angular velocity
    Vx = dPosX(i) + ctrlX;
    Vy = dPosY(i) + ctrlY;
    V = sqrt(Vx^2 + Vy^2);
    desiredAngle = atan2(Vy, Vx);
    angleDiff = desiredAngle - orientation(i);
    angleDiff = atan2(sin(angleDiff), cos(angleDiff));
    if abs(angleDiff) > 0.5*pi
        V = 0;
    end
    dOrientation(i) = angleDiff;
    
    % Update and regulize orientation first
    orientation(i) = orientation(i) + dt * dOrientation(i);
    orientation(i) = atan2(sin(orientation(i)), cos(orientation(i)));

    % Update location
    center(i,1) = center(i,1) + dt * V*cos(orientation(i));
    center(i,2) = center(i,2) + dt * V*sin(orientation(i));
    
    % Update and regularize phase
    phase(i) = phase(i) + dt * (natFreq(i) + dPhase(i));
    phase(i) = mod(phase(i),2*pi);
end