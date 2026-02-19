% PlotModules plots the modules in the simulation from the simulationdata

clear all;
close all;

KVariable = -0.5;
JVariable = 1;
file = strcat('SimData_K', num2str(KVariable), ...
            '_J', num2str(JVariable), ...
            '_R0.1_Sigma100_NumBots100_Trial1');
data = load(strcat(file,'.mat'));

botRad        = data.botRad;
dt            = data.dt;
plotXLimit    = data.plotXLimit;
plotYLimit    = data.plotYLimit;
numBots       = data.numBots;
recordCenter  = data.recordCenter;
recordPhase   = data.recordPhase;
recordOrientation = data.recordOrientation;
finalTimeStep = data.finalTimeStep;
natFreq       = data.natFreq;

% Save the video
E.vidfile = VideoWriter(strcat(file, 'Movie'),'Motion JPEG AVI');
E.vidfile.Quality = 95;
E.vidfile.FrameRate = 100;
open(E.vidfile);

% Turn on Figure
figure;
set(gcf, 'Position', [300, 300, 1300, 1300]);

for timeStep = 1:4:finalTimeStep

    for i = 1:numBots
        x = recordCenter(timeStep,i,1);
        y = recordCenter(timeStep,i,2);

        normalizedPhase = (recordPhase(timeStep, i)) / (2 * pi);
        color = hsv2rgb([normalizedPhase, 1, 1]);

        rectangle('Position', [x - botRad, y - botRad, 2*botRad, 2*botRad], ...
                  'Curvature', [1 1], ...
                  'FaceColor', color, ...
                  'EdgeColor', 'k', ...
                  'LineWidth', 0.5);
        hold on
    end
    
    %{
    % plot the trajectory for these agents
    for idx = 1:length(selectedAgents)
        agentID = selectedAgents(idx);
        
        trajX = squeeze(recordCenter(finalTimeStep-500:finalTimeStep, agentID, 1));
        trajY = squeeze(recordCenter(finalTimeStep-500:finalTimeStep, agentID, 2));
        
        plot(trajX, trajY, 'LineWidth', 2);
        hold on
    end
    %}

    axis equal
    axis([-5 5 -5 5])

    % pause(dt);

    % Write the frame into the video
    set(gca, 'Visible', 'off');  % hide the axis
    set(gcf, 'Color', 'w');
    F = getframe(gca);
    fixedFrame = imresize(F.cdata, [2000 2000]);
    writeVideo(E.vidfile, fixedFrame);
    clf;

end

close(E.vidfile)

toc