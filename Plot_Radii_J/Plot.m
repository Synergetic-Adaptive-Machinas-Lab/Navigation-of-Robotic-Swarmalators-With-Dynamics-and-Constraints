clear; clc;

% Load simulation data
omni = load("JvsRadii_omni.mat");
J = omni.J;
omni_innerRad = omni.innerRad;
omni_outerRad = omni.outerRad;

uni = load("JvsRadii_uni.mat");
uni_innerRad = uni.innerRad;
uni_outerRad = uni.outerRad;

bi = load("JvsRadii_bi.mat");
bi_innerRad = bi.innerRad;
bi_outerRad = bi.outerRad;

% Interprete
J_fine = linspace(min(J), max(J), 100);
omni_innerRad_smooth = interp1(J, omni_innerRad, J_fine, 'spline');
omni_outerRad_smooth = interp1(J, omni_outerRad, J_fine, 'spline');
uni_innerRad_smooth = interp1(J, uni_innerRad, J_fine, 'spline');
uni_outerRad_smooth = interp1(J, uni_outerRad, J_fine, 'spline');
bi_innerRad_smooth = interp1(J, bi_innerRad, J_fine, 'spline');
bi_outerRad_smooth = interp1(J, bi_outerRad, J_fine, 'spline');

% Plot smooth lines
figure;
plot(J_fine, omni_innerRad_smooth, 'LineWidth', 2, 'Color', 'b'); 
hold on;
plot(J_fine, omni_outerRad_smooth, 'LineWidth', 2, 'Color', 'r');
hold on;
plot(J_fine, uni_innerRad_smooth, '--', 'LineWidth', 2, 'Color', 'b'); 
hold on;
plot(J_fine, uni_outerRad_smooth, '--', 'LineWidth', 2, 'Color', 'r');
hold on;
plot(J_fine, bi_innerRad_smooth, ':', 'LineWidth', 2, 'Color', 'b'); 
hold on;
plot(J_fine, bi_outerRad_smooth, ':', 'LineWidth', 2, 'Color', 'r');

% Mark original data
% scatter(J, omni_innerRad, 50, 'b', 'filled');
% scatter(J, omni_outerRad, 50, 'r', 'filled');
% scatter(J, uni_innerRad, 50, 'b', 'MarkerFaceColor', 'none');
% scatter(J, uni_outerRad, 50, 'r', 'MarkerFaceColor', 'none');
% scatter(J, bi_innerRad, 50, 'b', 'filled');
% scatter(J, bi_outerRad, 50, 'r', 'filled');

xlabel('J', 'FontSize', 12);
ylabel('Radius', 'FontSize', 12);
title('Radii vs. J', 'FontSize', 14);
legend('Omni Inner Radius', 'Omni Outer Radius', ...
        'Uni Inner Radius', 'Uni Outer Radius', ...
        'Bi Inner Radius', 'Bi Outer Radius', ...
        'Location', 'best');
% grid on;
set(gca, 'FontSize', 12);
