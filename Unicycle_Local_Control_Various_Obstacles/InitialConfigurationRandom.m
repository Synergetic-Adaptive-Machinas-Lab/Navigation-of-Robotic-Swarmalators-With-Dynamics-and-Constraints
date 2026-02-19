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

for i = 1:numObstacles
    minDist = 1.5;
    valid = false;
    while ~valid
        % Generate obstacles randomly
        ox_candidate = 3.5 + 6 * rand;     % [3.5,9.5]
        oy_candidate = 3.5 + 6 * rand;     % [3.5,9.5]
        a_candidate  = 0.2 + 0.3 * rand;   % [0.2,0.5]
        b_candidate  = a_candidate;        % [0.2,0.5]
        n_candidate  = 0.3 + 5 * rand;     % [0.5,5.5]

        % Check distance between this obs and others
        valid = true;
        for j = 1:(i-1)
            dist = sqrt((ox_candidate - ox(j))^2 + (oy_candidate - oy(j))^2);
            if dist < minDist
                valid = false;
                break;
            end
        end
    end
    % Save the obstacle data
    ox(i) = ox_candidate;
    oy(i) = oy_candidate;
    a(i)  = a_candidate;
    b(i)  = b_candidate;
    n(i)  = n_candidate;
end

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