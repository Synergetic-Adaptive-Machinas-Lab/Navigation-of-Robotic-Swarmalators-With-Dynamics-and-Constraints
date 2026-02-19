%%TotalMovementAndPhaseUpdateCalculation with CBF for Target Point

% Generate a taget trajectory
px = traj(timeStep, 1);
py = traj(timeStep, 2);

% Parameters for CBF
eps = 10;   % Control gain for Lyapunov function
k = 10;     % Control gain for CBF

dPos(1:numBots) = 0;
dPosX(1:numBots) = 0;
dPosY(1:numBots) = 0;
dOrientation(1:numBots) = 0;
dPhase(1:numBots) = 0;
for i = 1:numBots
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
            F_rep  = B / (d*dist);     % Make sure bots will never go too close
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

% Initialize the force implied in the object
F_pushx = 0;
F_pushy = 0;
pushCoeff = 0.1;

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

    radius = botRad;
    
    % Lyapunov function for target point
    V = (mean_centerX - px)^2 + (mean_centerY - py)^2;
    
    % Time derivative of V
    LfV = 2 * (mean_centerX - px) * dX + 2 * (mean_centerY - py) * dY;
    LgV_u1 = 2 * (mean_centerX - px);
    LgV_u2 = 2 * (mean_centerY - py);
    A_clf = [LgV_u1, LgV_u2, -1];   % -1 is for relaxation variable
    b_clf = -LfV - eps * V;
    
    % Control Barrier Function
    % Construct a constraint for all other agents
    A_cbf_obs = [];
    b_cbf_obs = [];

    for j = 1:numObstacles
        % Rotate 45 degrees
        c = cosd(theta_rot);
        s = sind(theta_rot);
        
        dx_temp = centerX - ox(j);
        dy_temp = centerY - oy(j);
        
        r1 = c * dx_temp - s * dy_temp;
        r2 = s * dx_temp + c * dy_temp;
        
        b_j = (abs(r1/(a(j) + botRad)))^n(j) + (abs(r2/(b(j) + botRad)))^n(j) - 1;
        db_dcenterX = n(j) * (abs(r1/(a(j)+botRad)))^(n(j)-1) * (1/(a(j)+botRad)) * sign(r1) * c + ...
              n(j) * (abs(r2/(b(j)+botRad)))^(n(j)-1) * (1/(b(j)+botRad)) * sign(r2) * s;
        db_dcenterY = n(j) * (abs(r1/(a(j)+botRad)))^(n(j)-1) * (1/(a(j)+botRad)) * sign(r1) * (-s) + ...
              n(j) * (abs(r2/(b(j)+botRad)))^(n(j)-1) * (1/(b(j)+botRad)) * sign(r2) * c;
        % b_j = (abs(centerX - ox(j))/ ( a(j) + botRad ) )^n(j) + (abs(centerY - oy(j))/( b(j) + botRad ) )^n(j) - 1;
        % db_dcenterX = n(j) * (abs(centerX - ox(j))/( a(j) + botRad ) )^(n(j)-1) * (1/( a(j) + botRad ) ) * sign(centerX - ox(j));
        % db_dcenterY = n(j) * (abs(centerY - oy(j))/( b(j) + botRad ) )^(n(j)-1) * (1/( b(j) + botRad ) ) * sign(centerY - oy(j));
        % Lfb = ∂b/∂centerX * dX + ∂b/∂centerY * dY
        Lfb_j = db_dcenterX * dX + db_dcenterY * dY;
        Lgb_j_u1 = db_dcenterX;
        Lgb_j_u2 = db_dcenterY;
        A_temp = [-Lgb_j_u1, -Lgb_j_u2, 0];    
        b_temp = Lfb_j + k*b_j;
        
        % Add this constraint to all the constraints
        A_cbf_obs = [A_cbf_obs; A_temp];
        b_cbf_obs = [b_cbf_obs; b_temp];
    end

    % Construct a constraint for all other agents
    A_cbf_bots = [];
    b_cbf_bots = [];

    for j = 1:numBots
        if j ~= i
            obsX = center(j,1);
            obsY = center(j,2);
            b_ij = (centerX - obsX)^2 + (centerY - obsY)^2 - (2.01*botRad)^2;
            
            Lfb_ij = 2*(centerX - obsX)*dX + 2*(centerY - obsY)*dY;
            Lgb_ij_u1 = 2*(centerX - obsX);
            Lgb_ij_u2 = 2*(centerY - obsY);
            
            % Construct Constraint： -Lgb_ij * [u1; u2] + Lfb_ij + k*b_ij >= 0
            % Constraint： A_cbf * [u1; u2; relaxL; relaxB] <= b_cbf
            A_temp = [-Lgb_ij_u1, -Lgb_ij_u2, 0];
            b_temp = Lfb_ij + k * b_ij;
            
            % Add this constraint to all the constraints
            A_cbf_bots = [A_cbf_bots; A_temp];
            b_cbf_bots = [b_cbf_bots; b_temp];
        end
    end
    
    % Quadratic programming to find control
    v_max = 10;
    A_1 = [A_clf; A_cbf_obs; A_cbf_bots; 
        1, 0, 0; -1, 0, 0; 0, 1, 0; 0, -1, 0]; % The name 'A' has been used
    b_1 = [b_clf; b_cbf_obs; b_cbf_bots; 
        v_max; v_max; v_max; v_max];        % The name 'b' has been used
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
    % center(i,1) = center(i,1) + dt * Vx;
    % center(i,2) = center(i,2) + dt * Vy;
    
    % Update and regularize phase
    phase(i) = phase(i) + dt * (natFreq(i) + dPhase(i));
    phase(i) = mod(phase(i),2*pi);
    
    % Calculate the velocity of the object
    % calculate the distance between object and angent
    dx_obj = objCenter(1) - center(i,1);
    dy_obj = objCenter(2) - center(i,2);
    dist_obj = sqrt(dx_obj^2 + dy_obj^2);
    
    % When agent and object have contact, add a force
    if dist_obj < (botRad + objRad)
        F_pushx = F_pushx + pushCoeff * V * cos(orientation(i));
        F_pushy = F_pushy + pushCoeff * V * sin(orientation(i));
        % The agent will stop 
        center(i,1) = center(i,1) - dt * V*cos(orientation(i));
        center(i,2) = center(i,2) - dt * V*sin(orientation(i));
    end
end

% Update the location of object
objVel_x = objVel_x * 0.9 + F_pushx;
objVel_y = objVel_y * 0.9 + F_pushy;
% fprintf('objVel_x: %f \n', objVel_y);
objCenter(1) = objCenter(1) + dt * objVel_x;
objCenter(2) = objCenter(2) + dt * objVel_y;
% fprintf('objcenter_x: %f \n', objCenter(1));