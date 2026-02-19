%%RecordResults

if(timeStep == 1)
    recordCenter(1:finalTimeStep,1:numBots,1:2) = 0;
    recordOrientation(1:finalTimeStep, 1:numBots) = 0;
    recordPhase(1:finalTimeStep,1:numBots) = 0;
end

for i = 1:numBots
    recordCenter(timeStep,i,1) = center(i,1);
    recordCenter(timeStep,i,2) = center(i,2);
    recordOrientation(timeStep, i) = orientation(i);
    recordPhase(timeStep,i) = phase(i);
end