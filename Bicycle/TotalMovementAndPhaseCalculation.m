%%TotalMovementAndPhaseUpdateCalculation

% For controller
angleDiff_old(1:numBots) = 0;

for i = 1:numBots
    dPos(i) = 0;
    dPosX(i) = 0;
    dPosY(i) = 0;
    dOrientation(i) = 0;
    dDelta(i) = 0;
    dPhase(i) = 0;

    Fx = 0;
    Fy = 0;
    F = 0;
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
            dPhase(i) = dPhase(i) + K*sin(phase(j)-phase(i))/(dist*numBots);
        end
    end
end

for i = 1:numBots

    Vx = dPosX(i);
    Vy = dPosY(i);
    
    % Add controller
    desiredAngle = atan2(Vy, Vx);
    angleDiff = desiredAngle - orientation(i);
    angleDiff = atan2(sin(angleDiff), cos(angleDiff));
    angleDiff_dot = (angleDiff - angleDiff_old(i))/dt;
    
    dDelta(i) = 1*angleDiff + 8*angleDiff_dot;
    
    angleDiff_old(i) = angleDiff;

    
    desiredAngle = atan2(Vy, Vx);
    angleDiff = desiredAngle - orientation(i);
    angleDiff = atan2(sin(angleDiff), cos(angleDiff));
    dDelta(i) = angleDiff;

    % Limit the velocity
    V = sqrt(Vx^2 + Vy^2);
    V = min(maxV, V);

    % Update rotateAngle first
    rotateAngle(i) = rotateAngle(i) + dt * dDelta(i);
    rotateAngle(i) = max(min(0.8, rotateAngle(i)), -0.8);

    % Update and regularize orientation
    orientation(i) = orientation(i) + dt * V * tan(rotateAngle(i)) / L;
    orientation(i) = atan2(sin(orientation(i)),  cos(orientation(i)));

    % Update location
    rear(i,1) = rear(i,1) + dt * V*cos(orientation(i));
    rear(i,2) = rear(i,2) + dt * V*sin(orientation(i));
    center(i,1) = rear(i,1) + (L/2)*cos(orientation(i));
    center(i,2) = rear(i,2) + (L/2)*sin(orientation(i));

    
    % Update and regularize phase
    phase(i) = phase(i) + dt * (natFreq(i) + dPhase(i));
    phase(i) = mod(phase(i),2*pi);
end