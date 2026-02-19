%%TotalMovementAndPhaseUpdateCalculation with CBF for Target Point

% Define the target point
% px = 6;
% py = 6;

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
    b1  = (mean_center(1) - ox2)^2 + (mean_center(2) - oy2)^2 - (radius + r2)^2;
    Lfb1 = 2*(mean_center(1) - ox2) * mean_dPosX + 2 * (mean_center(2) - oy2) * mean_dPosY;
    Lgb1_u1 = 2*(mean_center(1) - ox2);
    Lgb1_u2 = 2*(mean_center(2) - oy2);
    A_cbf1 = [-Lgb1_u1, -Lgb1_u2, 0];
    b_cbf1 = Lfb1 + k*b1;
    
    % Quadratic programming to find control
    % For single barrier
    A_1 = [A_clf; A_cbf1; 1, 0, 0; -1, 0, 0; 0, 1, 0; 0, -1, 0]; % The name 'A' has been used
    b = [b_clf; b_cbf1; v_max; v_max; v_max; v_max];
    H = [2 0 0; 0 2 0; 0 0 0.02];
    F = [0; 0; 0];
    options = optimoptions('quadprog', 'Algorithm', 'interior-point-convex', 'Display', 'off');
    [ctrl, ~] = quadprog(H, F, A_1, b, [], [], [], [], [], options);
    
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

% Initialize the force implied in the object
F_pushx = 0;
F_pushy = 0;
pushCoeff = 0.1;

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
% fprintf('objcenter_x: %f \n', objCenter(1))