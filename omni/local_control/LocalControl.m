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
                d = dist;
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

b_j_all = [];
distObs_j_all = [];
for i = 1:numBots
    % Add Control Barrier Function here

    % Calculate the collective center
    mean_center = mean(center, 1);
    mean_centerX = mean_center(1);
    mean_centerY = mean_center(2);
    centerX = center(i,1);
    centerY = center(i,2);
    dX = dPosX(i);
    dY = dPosY(i);
    
    % Lyapunov function for target point
    V = (mean_centerX - px)^2 + (mean_centerY - py)^2;
    
    % Time derivative of V
    LfV = 2 * (mean_centerX - px) * dX + 2 * (mean_centerY - py) * dY;
    LgV_u1 = 2 * (mean_centerX - px);
    LgV_u2 = 2 * (mean_centerY - py);
    A_clf = [LgV_u1, LgV_u2, -1];   % -1 is for relaxation variable
    b_clf = -LfV - eps * V;
    
    % Control Barrier Function
    A_cbf_obs = [];
    b_cbf_obs = [];

    for j = 1:numObstacles
        b_j  = (centerX - ox(j))^2 + (centerY - oy(j))^2 - (r(j) + botRad)^2;
        distObs_j = sqrt( (centerX - ox(j))^2 + (centerY - oy(j))^2 ) - ( r(j) + botRad );
        Lfb_j = 2*(centerX - ox(j)) * dX + 2 * (centerY - oy(j)) * dY;
        Lgb_u1_j = 2*(centerX - ox(j));
        Lgb_u2_j = 2*(centerY - oy(j));
        A_temp = [-Lgb_u1_j, -Lgb_u2_j, 0];
        b_temp = Lfb_j + k*b_j;
        
        % Add this constraint to all the constraints
        A_cbf_obs = [A_cbf_obs; A_temp];
        b_cbf_obs = [b_cbf_obs; b_temp];
        b_j_all = [b_j_all; b_j];
        distObs_j_all = [distObs_j_all; distObs_j];
    end
    

    % Construct a constraint for all other agents
    A_cbf_all = [];
    b_cbf_all = [];

    for j = 1:numBots
        if j ~= i
            obsX = center(j,1);
            obsY = center(j,2);
            b_ij = (centerX - obsX)^2 + (centerY - obsY)^2 - (2.05*botRad)^2;
            
            Lfb_ij = 2*(centerX - obsX)*dX + 2*(centerY - obsY)*dY;
            Lgb_ij_u1 = 2*(centerX - obsX);
            Lgb_ij_u2 = 2*(centerY - obsY);
            
            % Construct Constraint： -Lgb_ij * [u1; u2] + Lfb_ij + k*b_ij >= 0
            % Constraint： A_cbf * [u1; u2; relaxL; relaxB] <= b_cbf
            A_temp = [-Lgb_ij_u1, -Lgb_ij_u2, 0];
            b_temp = Lfb_ij + k * b_ij;
            
            % Add this constraint to all the constraints
            A_cbf_all = [A_cbf_all; A_temp];
            b_cbf_all = [b_cbf_all; b_temp];
        end
    end
    
    % Quadratic programming to find control
    v_max = 10;
    A_1 = [A_clf; A_cbf_obs; A_cbf_all; 1, 0, 0; -1, 0, 0; 0, 1, 0; 0, -1, 0]; % The name 'A' has been used
    b_1 = [b_clf; b_cbf_obs; b_cbf_all; v_max; v_max; v_max; v_max];
    H = [2 0 0; 0 2 0; 0 0 0.02];
    F = [0; 0; 0];
    options = optimoptions('quadprog', 'Algorithm', 'interior-point-convex', 'Display', 'off');
    [ctrl, ~] = quadprog(H, F, A_1, b_1, [], [], [], [], [], options);
    
    % Update control velocities
    if ~isempty(ctrl)
        ctrlX = ctrl(1);
        ctrlY = ctrl(2);
        relaxL = ctrl(3);
    else
        ctrlX = 0;
        ctrlY = 0;
        dPosX(i) = 0;
        dPosY(i) = 0;
        fprintf('Unsolvable!\n');
    end
    % fprintf('ctrlX: %f\n', ctrlX);
    % fprintf('ctrlY: %f\n', ctrlY);
    % fprintf('relaxationL: %f\n', relaxL);
    % fprintf('relaxationB: %f\n', relaxB);
    
    % Uncomment this to disable control
    % ctrlX = 0;
    % ctrlY = 0;

    % Calculate the final linear velocity and angular velocity
    Vx = dPosX(i) + ctrlX;
    Vy = dPosY(i) + ctrlY;

    % Update location
    center(i,1) = center(i,1) + dt * Vx;
    center(i,2) = center(i,2) + dt * Vy;
    % center(i,1) = center(i,1) + dt * Vx;
    % center(i,2) = center(i,2) + dt * Vy;
    
    % Update and regularize phase
    phase(i) = phase(i) + dt * (natFreq(i) + dPhase(i));
    phase(i) = mod(phase(i),2*pi);
end

b_j = min(b_j_all);
distObs_j = min(distObs_j_all);