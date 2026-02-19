%%InitializationVariables
numBots = 100;
botRad = 0.1;
v_max = 1;
plotXLimit = 5;
plotYLimit = 5;
dt = 0.1;
A = 1;
B = 1.5;
P = 10;
J = JVariable;
K = KVariable;
threshDist = 100;
c = 1;

% Initialize each bot's location/orientation/phase
center(1:numBots, 1:2) = 0;
orientation(1:numBots) = 0;
phase(1:numBots) = 0;
currentBotRad(1:numBots) = 0;
natFreq(1:numBots) = 0;

% Initialize the trajectory
T1 = finalTimeStep/2 - 200; % the time reach the target for local control
T2 = finalTimeStep/2;       % the time start global control
traj = zeros(finalTimeStep, 2);
t1 = 6;
t2 = 6;

for t = 0:finalTimeStep
    if t < 300
        % first 500 timesteps
        traj(t+1, :) = [0, 0];
    else
        t_adj = t - 300;
        finalTimeStep_adj = finalTimeStep - 300;
        
        % Adjust the new time interval
        T1_new = finalTimeStep_adj/2 - 200;  % the time reach the target for local control
        T2_new = finalTimeStep_adj/2;        % the time start global control
        
        if t_adj <= T1_new
            % Stage 1: from (0,0) to (t1,t1)
            ratio = t_adj / T1_new;
            traj(t+1, :) = [t1*ratio, t1*ratio];
        elseif t_adj <= T1_new + 200
            % Stage 2: keep in (t1,t1) stay 200 timesteps
            traj(t+1, :) = [t1, t1];
        elseif t_adj <= finalTimeStep_adj - 200
            % Stage 3: from (t1,t1) to (t1+t2, t1+t2)
            ratio = (t_adj - T1_new - 200) / (T2_new - 200);
            traj(t+1, :) = [t1 + t2*ratio, t1 + t2*ratio];
        else
            % Stage 4: finally stay in (t1+t2, t1+t2)
            traj(t+1, :) = [t1+t2, t1+t2];
        end
    end
end

%{
% Plot trajectory
figure;
for w = 2:10:finalTimeStep
    plot(traj(1:w), traj(1:w), 'b-', 'LineWidth', 2);
    axis([0 13 0 13])
    hold on
    pause(0.1)
end
%}

% Initialize obstacle's location
ox1 = 6;
oy1 = 6;
r1 = 0.5;

ox2 = 10;
oy2 = 8;
r2 = 0.5;

% Initialize location of barrier (superellipse)
numObstacles = 2;
theta_rot = 45;
ox(1) = 11;
oy(1) = 7;
a(1) = 0.8;
b(1) = 1.8;
n(1) = 5;

ox(2) = 7;
oy(2) = 11;
a(2) = 0.8;
b(2) = 1.8;
n(2) = 5;

% Initialize the object for moving
objCenter(1:2) = 6;
objRad = 0.5;
objVel_x = 0;
objVel_y = 0;

%{
% plot obstacle for test
t = linspace(0, 2*pi, 200);

% Rotate
R = [cosd(-theta_rot), -sind(-theta_rot); 
     sind(-theta_rot),  cosd(-theta_rot)];

figure;
hold on;
for i = 1:numObstacles
    x_offset = a(i) * sign(cos(t)) .* (abs(cos(t))).^(2/n(i));
    y_offset = b(i) * sign(sin(t)) .* (abs(sin(t))).^(2/n(i));
    
    rotated = R * [x_offset; y_offset];
    
    x_rot = ox(i) + rotated(1,:);
    y_rot = oy(i) + rotated(2,:);
    
    fill(x_rot, y_rot, 'k', 'EdgeColor', 'k', 'FaceAlpha', 0.5);
end

axis equal;
grid on;
xlabel('x');
ylabel('y');

%}


