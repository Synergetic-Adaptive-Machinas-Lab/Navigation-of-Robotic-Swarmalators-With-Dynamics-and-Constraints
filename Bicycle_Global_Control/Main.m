%%Main

tic
clear all;
close all;

for KVariable = 1 % [1,-1,0,-0.5,-0.05]
    for JVariable = 0.1 % 0.1:0.1:1
        for trial = 1:5
            clearvars -except KVariable JVariable trial botRad threshDist

            finalTimeStep = 3000;

            InitializationVariables;

            InitialConfigurationRandom;

            timeStep = 0;

            while(timeStep < finalTimeStep)
                timeStep = timeStep + 1;
                GlobalControl;
                
                % Record the b(x) in CBF
                bx(timeStep) = b_j;
                distObs(timeStep) = distObs_j;
                
                RecordResults; % Save each particle's location and phase
            end

%             SaveSimulationData; % Save all the data in Workspace
           save(strcat('Bi_global_bx_trial', num2str(trial), '.mat'), 'bx', 'distObs');
        end
    end
end
toc