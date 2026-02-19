%%TotalMovementAndPhaseUpdateCalculation

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
            % Calculate the absolute distance between two bots
            dist = sqrt(dx^2 + dy^2);            
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
            dPhase(i) = dPhase(i) + K*sin(phase(j)-phase(i))/dist;
        end
    end
end

for i = 1:numBots
    % Calculate the linear velocity and angular velocity
    Vx = dPosX(i);
    Vy = dPosY(i);
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
    orientation(i) = atan2(sin(orientation(i)),  cos(orientation(i)));

    % Update location
    center(i,1) = center(i,1) + dt * V*cos(orientation(i));
    center(i,2) = center(i,2) + dt * V*sin(orientation(i));
    
    % Update and regularize phase
    phase(i) = phase(i) + dt * (natFreq(i)+(dPhase(i)/numBots));
    phase(i) = mod(phase(i),2*pi);
end