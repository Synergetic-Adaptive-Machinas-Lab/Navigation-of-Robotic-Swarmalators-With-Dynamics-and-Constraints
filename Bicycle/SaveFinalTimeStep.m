% PlotFinalTimestep using scatter for wheels
clear all;
close all;

% Simulation parameters (example)
for KVariable =  [1,-1,0,-0.5,-0.05]
    for JVariable = 1 % 0.1:0.1:1
        file = strcat('Bicycle_K_data/', ...
                'K', num2str(KVariable), ...
                '_J', num2str(JVariable), ...
                '_B10');
        data = load(strcat(file,'.mat'));

        botRad = data.botRad;
        L = data.L;
        dt = data.dt;
        plotXLimit = data.plotXLimit;
        plotYLimit = data.plotYLimit;
        numBots = data.numBots;
        recordCenter = data.recordCenter;
        recordPhase = data.recordPhase;
        recordOrientation = data.recordOrientation;
        recordRotateAngle = data.recordRotateAngle;
        finalTimeStep = data.finalTimeStep;
        natFreq = data.natFreq;

        % file = strcat('Bicycle_J_data/', ...
        %         'K', num2str(KVariable), ...
        %         '_J', num2str(JVariable), ...
        %         '_Trial1');
        % data = load(strcat(file,'.mat'));
        % 
        % botRad = data.botRad;
        % L = data.L;
        % numBots = data.numBots;
        % recordCenter = data.center;
        % recordTheta = data.theta;
        % recordPhase = data.phase;

        % Randomly select few agents to plot the trajectory
        numToPlot = 5;
        selectedAgents = randperm(numBots, numToPlot);
        
        % Use only the final timestep
        timeStep = finalTimeStep;
        figure;
%         set(gcf, 'Position', [300, 300, 1300, 1300]);
        
        for i = 1:numBots
            % Extract State Values /Position of rear wheel
            x = recordCenter(timeStep,i,1);
            y = recordCenter(timeStep,i,2);
            theta = recordOrientation(timeStep, i);
            delta = recordRotateAngle(timeStep, i);
            % x = recordCenter(i,1);
            % y = recordCenter(i,2);
            % theta = recordTheta(i);

            % Position of rear wheel
            xR = x - L/2 * cos(theta);
            yR = y - L/2 * sin(theta);

            % Position of front wheel
            xF = xR + L*cos(theta);
            yF = yR + L*sin(theta);

            
            % Normalize phase to [0,1] for color mapping (HSV to RGB)
            normalizedPhase = recordPhase(timeStep, i) / (2*pi);
            % normalizedPhase = recordPhase(i) / (2*pi);
            color = hsv2rgb([normalizedPhase, 1, 1]);

            % Add black frame to body
            l = 0.2;
            XR = xR - 0.65*l*cos(theta);
            YR = yR - 0.65*l*sin(theta);
            XF = xR + 1.5*l*cos(theta);
            YF = yR + 1.5*l*sin(theta);
            plot([XR, XF], [YR, YF], 'Color', 'k', 'LineWidth', 5);
            hold on;
            
            % Draw the body as a line between rear and front wheel
            plot([xR, xF], [yR, yF], 'Color', color, 'LineWidth', 3);
            hold on;
          
        end

        % plot the trajectory for these agents
        if KVariable == -0.5
            for idx = 1:length(selectedAgents)
                agentID = selectedAgents(idx);
    
                trajX = squeeze(recordCenter(finalTimeStep-1000:finalTimeStep, agentID, 1));
                trajY = squeeze(recordCenter(finalTimeStep-1000:finalTimeStep, agentID, 2));
    
                plot(trajX, trajY, 'color', 'k', 'LineWidth', 2);
                hold on
            end
        end
        
        axis equal;
        axis([-15 15 -15 15]);
        set(gca, 'Visible', 'off');
        path = strcat('Bicycle_K/K_', num2str(KVariable), '.png');
        % path = strcat('Bicycle_J/J_', num2str(JVariable), '.png');
        print(gcf, path, '-dpng', '-r300');
    end
end
