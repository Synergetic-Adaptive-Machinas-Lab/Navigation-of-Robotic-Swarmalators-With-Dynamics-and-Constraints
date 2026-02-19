%%InitializationVariables
numBots = 100;
botRad = 0.1;
plotXLimit = 2.5;
plotYLimit = 2.5;
dt = 0.1;
A = 1;
B = 3;
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