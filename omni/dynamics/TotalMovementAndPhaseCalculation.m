%%TotalMovementAndPhaseUpdateCalculation

for i = 1:numBots
    dPosX(i) = 0;
    dPosY(i) = 0;
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
            F_rep  = B / (d*dist);     % Make sure bots will never go too close
            F_total = F_attr - F_rep;

            % Calculate linear velocity
            dPosX(i) = dPosX(i) + F_total * dx;
            dPosY(i) = dPosY(i) + F_total * dy;

            % Calculate phase attraction
            dPhase(i) = dPhase(i) + K*sin(phase(j)-phase(i))/dist;
        end
    end
end

for i = 1:numBots
    % Update location
    center(i,1) = center(i,1) + dt * dPosX(i)/numBots;
    center(i,2) = center(i,2) + dt * dPosY(i)/numBots;
    
    % Update and regularize phase
    phase(i) = phase(i) + dt * (dPhase(i)/numBots);
    phase(i) = mod(phase(i),2*pi);
end