% Load simulation data
clear all;
close all;
for KVariable = -0.5 % [1,-1,0,-0.5,-0.05]
     for JVariable = 1 % 0.1:0.1:1
        file = strcat('Omni_K_data/', ...
                'K', num2str(KVariable), ...
                '_J', num2str(JVariable), ...
                '_B3');
        data = load(strcat(file, '.mat'));

        botRad = data.botRad;
        dt = data.dt;
        plotXLimit = data.plotXLimit;
        plotYLimit = data.plotYLimit;
        numBots = data.numBots;
        recordCenter = data.recordCenter;
        recordPhase = data.recordPhase;
        recordOrientation = data.recordOrientation;
        finalTimeStep = data.finalTimeStep;
        natFreq = data.natFreq;

        % file = strcat('Omni_J_data/', ...
        %         'K', num2str(KVariable), ...
        %         '_J', num2str(JVariable), ...
        %         '_Trial1');
        % data = load(strcat(file,'.mat'));
        % 
        % botRad = data.botRad;
        % numBots = data.numBots;
        % recordCenter = data.center;
        % recordTheta = data.theta;
        % recordPhase = data.phase;

        % Randomly select few agents to plot the trajectory
        numToPlot = 5;
        % selectedAgents = randperm(numBots, numToPlot);
        selectedAgents = [1, 3, 6, 7, 20];
        % Plot only the final timestep
        timeStep = finalTimeStep;  % Use the final timestep
        
        figure;  % Create a new figure
        arrowLength = 1.2 * botRad;  % Define the length of the arrow relative to the bot radius

        % Compute the center-of-mass of the swarm
        comX = mean(recordCenter(timeStep,:,1));
        comY = mean(recordCenter(timeStep,:,2));
        
        for i = 1:numBots
            % Extract the state for agent i at the final timestep
            x = recordCenter(timeStep, i, 1) - comX;
            y = recordCenter(timeStep, i, 2) - comY;
            % x = recordCenter(i,1);
            % y = recordCenter(i,2);
            
            % Normalize the phase into [0, 1] for color (HSV conversion)
            normalizedPhase = recordPhase(timeStep, i) / (2 * pi);
            % normalizedPhase = recordPhase(i) / (2*pi);
            color = hsv2rgb([normalizedPhase, 1, 1]);
            
            % Draw the agent as a circle (using rectangle with full curvature)
            rectangle('Position', [x - botRad, y - botRad, 2*botRad, 2*botRad], ...
                      'Curvature', [1, 1], ...
                      'FaceColor', color, ...
                      'EdgeColor', 'k', ...
                      'LineWidth', 0.5);
            hold on;
        end

        % plot the trajectory for these agents
        if KVariable == -0.5
            for idx = 1:length(selectedAgents)
                agentID = selectedAgents(idx);
    
                trajX = squeeze(recordCenter(finalTimeStep-5000:finalTimeStep, agentID, 1));
                trajY = squeeze(recordCenter(finalTimeStep-5000:finalTimeStep, agentID, 2));
    
                plot(trajX, trajY, 'color', 'k', 'LineWidth', 2);
                hold on
            end
        end
        
        % Set axis properties for proper display
        axis equal;
        axis([-5, 5, -5, 5]);
        set(gca, 'Visible', 'off');  % Optionally hide the axis
        
        % Save the figure as an image (e.g., PNG)
        path = strcat('Omni_K/K_', num2str(KVariable), '.png');
        % path = strcat('Omni_J/J_', num2str(JVariable), '.png');
        % saveas(gcf, path);
        % exportgraphics(gcf, path, 'Resolution', 300);
        print(gcf, path, '-dpng', '-r300');
     end
end
