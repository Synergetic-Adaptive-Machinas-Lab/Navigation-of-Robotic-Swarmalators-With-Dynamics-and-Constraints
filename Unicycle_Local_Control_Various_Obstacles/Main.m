%%Main

tic
clear all;
close all;

file = "obstacles";
data = load(strcat(file,'.mat'));
ox = data.ox;
% display(ox);
oy = data.oy;
a = data.a;
b = data.b;
n = data.n;

for KVariable = 1 % -1:0.1:2
    for JVariable = 0.1 % -1:0.1:1
        for trial = 1 % 1:10s
            clearvars -except KVariable JVariable trial botRad threshDist

            finalTimeStep = 2000;

            InitializationVariables;

            InitialConfigurationRandom;

            timeStep = 0;

            while(timeStep < finalTimeStep)
                timeStep = timeStep + 1;
                if(timeStep > 1)
                    TotalMovementAndPhaseCalculation;
                end

                if mod(timeStep,20) == 0
                    fprintf('timeStep: %d\n', timeStep);
                end
                
                RecordResults; % Save each particle's location and phase
            end

            SaveSimulationData; % Save all the data in Workspace
        end
    end
end
toc