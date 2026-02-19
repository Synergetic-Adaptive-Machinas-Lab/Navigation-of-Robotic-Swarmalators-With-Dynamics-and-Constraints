%%InitializationVariables
numBots = 100;
numObstacles = 15;
botRad = 0.1;
v_max = 5;
plotXLimit = 3;
plotYLimit = 3;
dt = 0.1;
A = 1;
B = 2;
P = 10;
J = JVariable;
K = KVariable;
threshDist = 100;
c = 1;

% Initialize each bot's location/orientation/phase
center(1:numBots, 1:2) = 0;
orientation(1:numBots) = 0;
phase(1:numBots) = 0;
currentBotRad(1:numBots) = 0;
natFreq(1:numBots) = 0;

% Initialize the trajectory
traj = zeros(finalTimeStep, 2);

for t = 0:finalTimeStep-1
    if t < 300
        % collect at (0,0)
        traj(t+1, :) = [0, 0];
    elseif t < 1700
        % from(0,0) to (12,12)
        ratio = (t - 300) / (1700 - 300);  % (t-300)/1400
        traj(t+1, :) = [12*ratio, 12*ratio];
    else
        % stay at (12,12)
        traj(t+1, :) = [12, 12];
    end
end

% Initialize location of barrier (superellipse)
ox(1:numObstacles) = 0;
oy(1:numObstacles) = 0;
a(1:numObstacles) = 0;
b(1:numObstacles) = 0;
n(1:numObstacles) = 0;