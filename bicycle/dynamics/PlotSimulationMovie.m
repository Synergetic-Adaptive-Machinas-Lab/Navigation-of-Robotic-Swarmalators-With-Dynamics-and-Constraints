%%PlotModules plots the modules in the simulation from the simulationdata
%%file
tic
clear all;
close all;

for KVariable = -0.05 % [1,-1,0,-0.5,-0.05]
    clearvars -except KVariable

    JVariable = 1;
    numBots = 100;
    file = strcat('SimData_K', num2str(KVariable), ...
                '_J', num2str(JVariable), ...
                '_R0.1_Sigma100_NumBots',num2str(numBots), ...
                '_Trial1');
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

    % Save the video
    E.vidfile = VideoWriter(strcat(file, 'Movie'),'Motion JPEG AVI');
    E.vidfile.Quality = 95;
    E.vidfile.FrameRate = 100;
    open(E.vidfile);

    % Turn on Figure
    figure;
    set(gcf, 'Position', [300, 300, 1300, 1300]);


    % Draw the figure at each timestep and then delete it
    for timeStep = 1:8:finalTimeStep
            % comX = mean(recordCenter(timeStep,:,1));
            % comY = mean(recordCenter(timeStep,:,2));

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

                % Normalize the phase into [0, 1] (hsv)
                normalizedPhase = (recordPhase(timeStep, i)) / (2 * pi);
                % Transfer from hsv to rgb
                color = hsv2rgb([normalizedPhase, 1, 1]);
    %             colormap(hsv);
    %             caxis([0, 2*pi]);
    % 
    %             cb = colorbar;
    %             cb.Ticks = [0, pi, 2*pi];
    %             cb.TickLabels = {'0', '\pi', '2\pi'};
    %             cb.FontSize = 14;

                % Add black frame to body
                l = 0.2;
                XR = xR - 0.3*l*cos(theta);
                YR = yR - 0.3*l*sin(theta);
                XF = xR + 1.3*l*cos(theta);
                YF = yR + 1.3*l*sin(theta);
                plot([XR, XF], [YR, YF], 'Color', 'k', 'LineWidth', 5);
                hold on;

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

                % Draw wheelw
                % wheelRadius = 0.15*L;

                % % Draw the rear wheel as a circle (using rectangle with curvature [1,1])
                % rectangle('Position', [xR - wheelRadius, yR - wheelRadius, ...
                %             2*wheelRadius, 2*wheelRadius], ...
                %           'Curvature', [1, 1], ...
                %           'FaceColor', color, ...
                %           'EdgeColor', 'k', ...
                %           'LineWidth', 0.5);
                % 
                % % Draw the front wheel as a circle
                % rectangle('Position', [xF - wheelRadius, yF - wheelRadius, ...
                %             2*wheelRadius, 2*wheelRadius], ...
                %           'Curvature', [1, 1], ...
                %           'FaceColor', color, ...
                %           'EdgeColor', 'k', ...
                %           'LineWidth', 0.5);
             end   

        axis equal
        a = 20;
        axis([-a a -a a])
        % arenaFactor = 1.5;
        % axis([-arenaFactor*plotXLimit arenaFactor*plotXLimit -arenaFactor*plotYLimit arenaFactor*plotYLimit])
        set(gca, 'Visible', 'off');
        set(gcf,'color','w');
        % pause(dt);


        % Write the frame into the video
        F = getframe(gca);
        fixedFrame = imresize(F.cdata, [2000 2000]);
        writeVideo(E.vidfile, fixedFrame);

        clf;
    end
end
toc



