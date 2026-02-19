%%Main

tic
clear all;
close all;

for KVariable = 0 % -1:0.1:2
    for JVariable = 1 % -1:0.1:1
        for trial = 1 % 1:10s
            clearvars -except KVariable JVariable trial botRad threshDist

            finalTimeStep = 2000;

            InitializationVariables;

            InitialConfigurationRandom;

            timeStep = 0;
            % K = 2;
            % J = 0.05;


            while(timeStep < finalTimeStep)
                % if timeStep == 150
                %     K = -2;
                %     J = 1;
                % elseif timeStep == 300
                %     K = 0;
                %     J = 1;
                % end
                timeStep = timeStep + 1;
                if(timeStep > 1 && timeStep <=300+(finalTimeStep-500)/2)
                    LocalControl;
                elseif(timeStep > 300+(finalTimeStep-500)/2)
%                     GlobalControl;
                    LocalControl2;
                end
                
                if mod(timeStep, 20) == 0
                    fprintf('Timestep: %d\n', timeStep);
                end

                RecordResults; % Save each particle's location and phase
            end

            SaveSimulationData; % Save all the data in Workspace
        end
    end
end
toc