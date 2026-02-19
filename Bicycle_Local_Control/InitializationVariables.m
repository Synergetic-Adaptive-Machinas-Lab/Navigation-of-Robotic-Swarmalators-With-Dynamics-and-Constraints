%%InitializationVariables
numBots = 100;
botRad = 0.1;
L = 0.2;
v_max = 2;
maxV = 1;
plotXLimit = 7;
plotYLimit = 7;
dt = 0.1;
A = 1;
B = 5;
P = 10;
J = JVariable;
K = KVariable;
threshDist = 100;
c = 1;

% Initialize each bot's location/orientation/phase
rear(1:numBots, 1:2) = 0;
center(1:numBots, 1:2) = 0;
orientation(1:numBots) = 0;
rotateAngle(1:numBots) = 0;
phase(1:numBots) = 0;
currentBotRad(1:numBots) = 0;
natFreq(1:numBots) = 0;

% Initialize the trajectory
traj = zeros(finalTimeStep, 2);

for t = 0:finalTimeStep-1
    if t < 500
        % collect at (0,0)
        traj(t+1, :) = [0, 0];
    elseif t < 2500
        % from(0,0) to (20,20)
        ratio = (t - 500) / (2500 - 500);  % (t-300)/1400
        traj(t+1, :) = [20*ratio, 20*ratio];
    else
        % stay at (20,20)
        traj(t+1, :) = [20, 20];
    end
end

% Initialize obstacle's location
numObstacles = 1;
bx(1:finalTimeStep) = 0;
distObs(1:finalTimeStep) = 0;
ox(1:numObstacles) = 0;
oy(1:numObstacles) = 0;
a(1:numObstacles) = 0;
b(1:numObstacles) = 0;
n(1:numObstacles) = 0;
ox(1) = 16;
oy(1) = 12;
a(1) = 1;
b(1) = 1;
r(1) = 1;
n(1) = 1;

% ox(2) = 12;
% oy(2) = 16;
% a(2) = 1;
% b(2) = 1;
% r(2) = 1;
% n(2) = 1;