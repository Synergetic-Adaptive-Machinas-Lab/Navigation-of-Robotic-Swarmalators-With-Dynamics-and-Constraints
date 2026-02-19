%%Main

tic
clear all;
close all;

for KVariable = -0.05 %[1,-1,0,-0.5]
    for JVariable = 1 % 0.1:0.1:1
        for trial = 1
            clearvars -except KVariable JVariable trial botRad threshDist

            InitializationVariables;

            InitialConfigurationRandom;

            % For line plot
            % dt = 0.1
            % J = 1.0,  0.9,  0.8,  0.7,  0.6,  0.5,  0.4,  0.3,  0.2,  0.1
            % T = 20000,20000,20000,50000,50000,50000,50000,50000,50000,60000
            
            % For Emergent Behaviors
            % dt = 0.1
            % K = 1, -1, 0, -0.5, -0.05
            % T = 16000
            
            finalTimeStep = 16000;
            timeStep = 0;

            while(timeStep < finalTimeStep)
                timeStep = timeStep + 1;
                if(timeStep > 1)
                    TotalMovementAndPhaseCalculation;
                end
                
                RecordResults; % Save each particle's location and phase
            end

            SaveSimulationData; % Save all the data in Workspace
            
            % Only save the last timestep's info in Workspace
            % center = squeeze(recordCenter(finalTimeStep, :, :));
            % theta = squeeze(recordOrientation(finalTimeStep, :));
            % phase = squeeze(recordPhase(finalTimeStep, :));

            % save(strcat('Bicycle_J_data/', ...
            %     'K', num2str(KVariable), ...
            %     '_J', num2str(JVariable), ...
            %     '_Trial', num2str(trial), '.mat'), ...
            %     'botRad', 'numBots', 'L', 'center', 'theta', 'phase');
        end
    end
end
toc