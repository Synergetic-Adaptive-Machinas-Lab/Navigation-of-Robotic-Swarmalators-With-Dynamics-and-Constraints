%%PlotModules plots the modules in the simulation from the simulationdata
%%file
tic

clear all;
close all;
KVariable = 1;
JVariable = 0.1;
numBots = 100;
% file = strcat('SimData_K', num2str(KVariable), ...
%                 '_J', num2str(JVariable), ...
%                 '_R0.1_Sigma100_NumBots',num2str(numBots), ...
%                 '_Trial1');
file = "K1_J0.1_R0.1_NumBots100_NumObs_2";
data = load(strcat(file,'.mat'));

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
traj = data.traj;
% Get obstacles' location
numObstacles = data.numObstacles;
ox = data.ox;
oy = data.oy;
r = data.r;

% Save the video
E.vidfile = VideoWriter(strcat(file, 'Movie'),'Motion JPEG AVI');
E.vidfile.Quality = 95;
E.vidfile.FrameRate = 100;
open(E.vidfile);

% Draw the figure at each timestep and then delete it
for timeStep = 1:1:finalTimeStep
    % Plot the target trajectory
    plot(traj(1:timeStep), traj(1:timeStep), 'b-', 'LineWidth', 2);
    hold on

    % Plot the reality trajectory
    comX(timeStep) = mean(recordCenter(timeStep,:,1));
    comY(timeStep) = mean(recordCenter(timeStep,:,2));
    plot(comX, comY, 'r-', 'LineWidth', 2);
    hold on

    % Plot the obstacle
    rectangle('Position', [ox(1) - r(1), oy(1) - r(1), 2*r(1), 2*r(1)], ...
              'Curvature', [1 1], ...
              'FaceColor', 'k', ...
              'EdgeColor', 'k', ...  % circle edge color
              'LineWidth', 0.1);

    rectangle('Position', [ox(2) - r(2), oy(2) - r(2), 2*r(2), 2*r(2)], ...
              'Curvature', [1 1], ...
              'FaceColor', 'k', ...
              'EdgeColor', 'k', ...  % circle edge color
              'LineWidth', 0.1);

    % Draw each agent with its orientation
    arrowLength = 1.2 * botRad;
    for i = 1:numBots
        x = recordCenter(timeStep,i,1);
        y = recordCenter(timeStep,i,2);
        theta = recordOrientation(timeStep, i);
        dx = arrowLength*cos(theta);
        dy = arrowLength*sin(theta);
        % Normalize the phase into [0, 1] (hsv)
        normalizedPhase = (recordPhase(timeStep, i)) / (2 * pi);
        % Transfer from hsv to rgb
        color = hsv2rgb([normalizedPhase, 1, 1]);

        rectangle('Position', [x - botRad, y - botRad, 2*botRad, 2*botRad], ...
              'Curvature', [1 1], ...
              'FaceColor', color, ...
              'EdgeColor', 'k', ...  % circle edge color
              'LineWidth', 0.5);
        hold on

        % The final '0' argument to quiver disables automatic scaling
        quiver(x, y, dx, dy, 0, ...
               'Color','k', ...
               'LineWidth',0.5, ...
               'MaxHeadSize',0.5);
        hold on
    end   
            
    axis equal
    axis([-4 15 -4 15])
    % arenaFactor = 2;
    % axis([-arenaFactor*plotXLimit arenaFactor*plotXLimit -arenaFactor*plotYLimit arenaFactor*plotYLimit])
    set(gca, 'Visible', 'off');
    % pause(dt);
    
    % Write the frame into the video
    F = getframe(gca);
    fixedFrame = imresize(F.cdata, [2000 2000]);
    writeVideo(E.vidfile, fixedFrame);

    if mod(timeStep,200) == 0 || timeStep == 1
        set(gcf, 'PaperPositionMode', 'auto');  
        filename = fullfile('media', sprintf('Frame_%04d.png', timeStep));
        print(gcf, filename, '-dpng', '-r300'); 
    end
    
    clf;
end

close(E.vidfile);

toc



