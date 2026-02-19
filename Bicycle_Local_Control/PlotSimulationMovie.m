%%PlotModules plots the modules in the simulation from the simulationdata
%%file
tic
clear all;
close all;
KVariable = 1;
JVariable = 0.1;
numBots = 100;
file = strcat('SimData_K', num2str(KVariable), ...
            '_J', num2str(JVariable), ...
            '_R0.1_Sigma100_NumBots',num2str(numBots), ...
            '_Trial1');
file = "Obs2";
data = load(strcat(file,'.mat'));

botRad = data.botRad;
L = data.L;
dt = data.dt;
plotXLimit = data.plotXLimit;
plotYLimit = data.plotYLimit;
numBots = data.numBots;
recordRear = data.recordRear;
recordCenter = data.recordCenter;
recordPhase = data.recordPhase;
recordOrientation = data.recordOrientation;
recordRotateAngle = data.recordRotateAngle;
finalTimeStep = data.finalTimeStep;
natFreq = data.natFreq;
traj = data.traj;
% Get obstacles' location
numObstacles = data.numObstacles;
ox = data.ox;
oy = data.oy;
r = data.r;
r(1) = r(1) - 0.1;
r(2) = r(2) - 0.1;

% Save the video
E.vidfile = VideoWriter(strcat(file, 'Movie'),'Motion JPEG AVI');
E.vidfile.Quality = 95;
E.vidfile.FrameRate = 100;
open(E.vidfile);

% Get the mean position (for saving figures only)
comX = squeeze(mean(recordCenter(:,:,1),2));
comY = squeeze(mean(recordCenter(:,:,2),2));

% Turn on Figure
figure;
set(gcf, 'Position', [300, 300, 1300, 1300]);

% Draw the figure at each timestep and then delete it
for timeStep = 1:1:finalTimeStep
        % Plot the target trajectory
        plot(traj(1:timeStep), traj(1:timeStep), 'b-', 'LineWidth', 2);
        hold on
    
        % Plot the reality trajectory
%         comX(timeStep) = mean(recordCenter(timeStep,:,1));
%         comY(timeStep) = mean(recordCenter(timeStep,:,2));
%         plot(comX, comY, 'r-', 'LineWidth', 2);
        plot(comX(1:timeStep), comY(1:timeStep), 'r-', 'LineWidth', 2);
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
            % Extract State Values /Position of rear wheel
            x = recordCenter(timeStep,i,1);
            y = recordCenter(timeStep,i,2);
            theta = recordOrientation(timeStep, i);
            delta = recordRotateAngle(timeStep, i);

            % Position of rear wheel
            xR = x - L/2 * cos(theta);
            yR = y - L/2 * sin(theta);

            % Position of front wheel
            xF = xR + L*cos(theta);
            yF = yR + L*sin(theta);
            
            % Add black frame to body
            l = 0.2;
            XR = xR - 0.65*l*cos(theta);
            YR = yR - 0.65*l*sin(theta);
            XF = xR + 1.5*l*cos(theta);
            YF = yR + 1.5*l*sin(theta);
            plot([XR, XF], [YR, YF], 'Color', 'k', 'LineWidth', 5);
            hold on;

            % Normalize the phase into [0, 1] (hsv)
            normalizedPhase = (recordPhase(timeStep, i)) / (2 * pi);
            % Transfer from hsv to rgb
            color = hsv2rgb([normalizedPhase, 1, 1]);

            % Draw the body
            plot([xR xF],[yR yF],'color',color,'LineWidth',2);
            hold on;

            % Draw the safety area of each agents
            % rectangle('Position', [x - botRad, y - botRad, 2*botRad, 2*botRad], ...
            %           'Curvature', [1, 1], ...
            %           'EdgeColor', color, ...
            %           'LineWidth', 1, ...
            %           'FaceColor', 'none');
            % hold on
         end   
            
    axis equal
    a = 26;
    axis([-7 a -7 a])
    set(gca, 'Visible', 'off');
    set(gcf,'Color','w');
    % pause(dt);


    % Write the frame into the video
    F = getframe(gca);
    fixedFrame = imresize(F.cdata, [2000 2000]);
    writeVideo(E.vidfile, fixedFrame);

%     if mod(timeStep-1,200) == 0 || timeStep == 1
%         set(gcf, 'PaperPositionMode', 'auto');  
%         filename = fullfile('media', sprintf('Frame_%04d.png', timeStep-1));
%         print(gcf, filename, '-dpng', '-r300'); 
%     end

    clf;
end
toc



