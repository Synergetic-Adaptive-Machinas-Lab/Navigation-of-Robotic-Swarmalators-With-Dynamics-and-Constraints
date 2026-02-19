%%InitialConfigurationRandom

for i = 1:numBots
    center(i,1) = -plotXLimit + rand*2*plotXLimit;
    center(i,2) = -plotYLimit + rand*2*plotYLimit;
    % Make sure generated bot don't collide
    while checkCollision(center, i, botRad) == true
        center(i,1) = -plotXLimit + rand*2*plotXLimit;
        center(i,2) = -plotYLimit + rand*2*plotYLimit;
    end
    orientation(i) = 2*pi*rand;
    % phase(i) = 2*pi*rand;
end
% Generate the phase uniformly
phase = linspace(2*pi/numBots,2*pi,numBots);

function valid = checkCollision(center, i, botRad)
    if i == 1
        valid = false;
        return
    end

    for j = 1:i-1
        dx = center(i,1) - center(j,1);
        dy = center(i,2) - center(j,2);
        dist = sqrt(dx^2 + dy^2);
        if dist < 3*botRad
            valid = true;
            return
        end
    end
    valid = false;
end