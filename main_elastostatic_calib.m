%% Elastostatic Calibration
clear all;
close all;
clc;

%% Initial state
link_lengths = [675, 350, 1150, 1200, -41, 240]*1e-3;

% Initialize actuators
q = [0, 0, 0, 0, 0, 0, 0];

% Stiffness contsant coefficients
k_const = [1, 2, 0.5, 3, 2.5, 1.5, 0.75]*1e06;

% Stiffness matrix
K_theta = diag(k_const);

% Number of experiments
N = 30;

% Initialize the components of the parameter estimation equation
A_p1 = zeros(7,7); % first component 
A_p2 = zeros(7,1); % second component 

%% Getting the True stiffness matrix
for n = 1:N
    % Random Wrench vector
    w = randn(6,1)*1000/4;

    % Random angles
    q_revolute = randn(6,1);
    q_prismatic = rand(1,1);
    
    q = [q_prismatic, q_revolute(1), q_revolute(2), q_revolute(3), q_revolute(4), q_revolute(5), q_revolute(6)];

    % Jacobian Computation
    [J_theta, Jt1, Jt2, Jt3, Jt4, Jt5, Jt6, Jt7] = Jacobian(q, link_lengths);
    
    % Random noise
    eps = randn(6,1)*1e-05;
   
    % Calculate the deflection vector
    dt = (J_theta / K_theta * J_theta')*w + eps;
    
    % Calculate the A matrices
    A1 = Jt1 * Jt1' * w;
    A2 = Jt2 * Jt2' * w;
    A3 = Jt3 * Jt3' * w;
    A4 = Jt4 * Jt4' * w;
    A5 = Jt5 * Jt5' * w;
    A6 = Jt6 * Jt6' * w;
    A7 = Jt7 * Jt7' * w;
    
    A = [A1, A2, A3, A4, A5, A6, A7];
    
    A_p1 = A_p1 + A'*A;
    A_p2 = A_p2 + A'*dt;

end

%% Calculate the real compliance vector
K_compliance = A_p1 \ A_p2;
K_stiffness = 1./K_compliance;

%% Forces
F = [-500, -200, -2000, 0, 0, 0]';

%% Trajectory
% Draw half circle
t = linspace(0,2*pi);

x = sin(t); %cos(t)
y = t; %sin(t)
z = (0*t)+2.5;

% Get the stiffness matrix
K_theta = diag(K_stiffness);

%% Uncalibrated Path Calculation
q_start = [0,0,0,0,0,0,0];
qs = zeros(7,length(t));
uncalib_poses = zeros(6,length(t));
for i = 1:length(t)
    pose = [x(i) y(i) z(i) 0 0 0]';
    qs(:,i) = IK_calib(q_start, link_lengths, pose);
   
    % Calculating jacobians 
    [J_theta, Jt1, Jt2, Jt3, Jt4, Jt5, Jt6, Jt7] = Jacobian(q, link_lengths);
    eps = randn(6,1)*1e-05;
    dt = (J_theta / K_theta * J_theta')*F + eps;
    
    % Calculate the uncalibrated poses
    uncalib_pose = pose + dt;
    uncalib_poses(:,i) = uncalib_pose;
  
end

% Compensation 
difference = [x;y;z;zeros(1,length(t));zeros(1,length(t));zeros(1,length(t))] - uncalib_poses;
calibrated = uncalib_poses + difference;

% Plotting
figure('name','deflection')
plot3(x,y,z,'--r','LineWidth',2)
grid on
hold on
scatter3(uncalib_poses(1,:),uncalib_poses(2,:),uncalib_poses(3,:), 20, [0 0 1])
scatter3(calibrated(1,:),calibrated(2,:),calibrated(3,:), 20, [0 0.75  0.5])
xlabel('x')
ylabel('y')
zlabel('z')
legend('desired path','uncalibrated path','calibrated path')