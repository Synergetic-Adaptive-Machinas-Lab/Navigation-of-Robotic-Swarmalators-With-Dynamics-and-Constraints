%%Main

tic
clear all;
close all;

for KVariable = 1 % -1:0.1:2
    for JVariable = 0.1 % -1:0.1:1
        for trial = 1:5 % 1:10s
            clearvars -except KVariable JVariable trial botRad threshDist

            finalTimeStep = 3000;

            InitializationVariables;

            InitialConfigurationRandom;

            timeStep = 0;

            while(timeStep < finalTimeStep)
                timeStep = timeStep + 1;
                if mod(timeStep, 50) == 0
                    fprintf('Timestep: %d \n', timeStep);
                end
                % GlobalControl;
                LocalControl;
                
                % Record the b(x) in CBF
                bx(timeStep) = b_j;
                distObs(timeStep) = distObs_j;
%                 fprintf("%f \n",b_j);
                
                RecordResults; % Save each particle's location and phase
            end

            SaveSimulationData; % Save all the data in Workspace
            save(strcat('Omni_local_bx_trial', num2str(trial), '.mat'), 'bx', 'distObs');
        end
    end
end
toc