% Load simulation data
clear all;
close all;

% Initialize arrays
J = 0.1:0.1:1;
innerRad = zeros(size(J));
outerRad = zeros(size(J));
idx = 1;

for KVariable = 0 %[1,-1,0,-0.5,-0.05]
    for JVariable = 0.1:0.1:1
        for trial = 1:10
            file = strcat('Omni_J_data/', ...
                    'K', num2str(KVariable), ...
                    '_J', num2str(JVariable), ...
                    '_Trial', num2str(trial));
            data = load(strcat(file, '.mat'));
            
            botRad = data.botRad;
            numBots = data.numBots;
            recordCenter = data.center;
            recordTheta = data.theta;
            recordPhase = data.phase;
           
            
            % Get the mean position of all agents
            mean_center = mean(recordCenter, 1);  % 1×2 vector, mean position
            
            % Compute distances of each agent from the mean center.
            radii = vecnorm(recordCenter - mean_center, 2, 2);
            inner = min(radii) - botRad;
            outer = max(radii) + botRad;
            
            % Save to array
            innerRad(idx) = innerRad(idx) + inner/10;
            outerRad(idx) = outerRad(idx) + outer/10;
            
            fprintf('J = %f, inner = %f, outer = %f \n', JVariable, inner, outer);
        end
        idx = idx + 1;
    end
end
save('JvsRadii_omni.mat', 'J', 'innerRad', 'outerRad');

% Interprete J
J_fine = linspace(min(J), max(J), 100);
innerRad_smooth = interp1(J, innerRad, J_fine, 'spline');
outerRad_smooth = interp1(J, outerRad, J_fine, 'spline');

% Plot smooth lines
figure;
plot(J_fine, innerRad_smooth, 'LineWidth', 2, 'Color', 'b'); 
hold on;
plot(J_fine, outerRad_smooth, 'LineWidth', 2, 'Color', 'r');
% Mark original data
scatter(J, innerRad, 50, 'b', 'filled');
scatter(J, outerRad, 50, 'r', 'filled');

xlabel('J', 'FontSize', 12);
ylabel('Radius', 'FontSize', 12);
title('Radii vs. J', 'FontSize', 14);
legend('Inner Radius', 'Outer Radius', 'Location', 'best');
% grid on;
set(gca, 'FontSize', 12);
