clc;
clear;
close all;
% Ensure only the desired model is in the path
% (Optional: Add the directory of your desired model to the path)
%addpath('C:\path\to\your\model\directory'); % Change this to your actual model directory
% Model parameters and initial conditions
R = 0.25;
x_obs = [1; 1];
x_init = [0; 0];
gamma = 20;  % Ensure gamma is defined
beta = 1;    % Ensure beta is defined
Tf = 5; % Duration
Ts = 0.001; % Sample time
% Ensure parameters are in the workspace
assignin('base', 'R', R);
assignin('base', 'x_obs', x_obs);
assignin('base', 'x_init', x_init);
assignin('base', 'gamma', gamma);
assignin('base', 'beta', beta);
assignin('base', 'Tf', Tf);
assignin('base', 'Ts', Ts);
% Simulink model
mdl = 'CBF_for_2Dof';
open_system(mdl);
% Run the simulation
out = sim(mdl);
% Extract logged data
logData = out.logsout;
size_of_time = size(logData{2}.Values.Data, 1);
x1 = zeros(size_of_time, 1);
x2 = zeros(size_of_time, 1);
control_input = zeros(size_of_time, 2);
des_input = zeros(size_of_time, 2);
hx = zeros(size_of_time, 1);
for ct = 1:size_of_time
    x1(ct) = logData{1}.Values.Data(ct, 1);
    x2(ct) = logData{1}.Values.Data(ct, 2);
    control_input(ct, :) = logData{2}.Values.Data(ct, :);
    des_input(ct, :) = logData{3}.Values.Data(:,:,ct)';
    hx(ct) = logData{4}.Values.Data(ct);
end
% Plot tracking with constraint
figure('Name', 'Tracking with Constraint');
viscircles(x_obs', R);
hold on;
plot(x1, x2, 'b'), grid;
xlabel('x1');
ylabel('x2');
plot(x1(1), x2(1), 'g*');
plot(x1(end), x2(end), 'go');
quiver(x1(1), x2(1), x1(20)-x1(1), x2(20)-x2(1), 10, 'LineWidth', 1.5, 'MaxHeadSize', 2);
plot(x_obs(1), x_obs(2), 'Color', 'b', 'Marker', '+');
xlim([-0.5 2.5]);
ylim([0 2.5]);
% Plot the values of the CBF
figure('Name', 'The values of the CBF');
plot(out.tout(1:size_of_time), hx, 'LineWidth', 2), grid;
xlim([0 5]);
ylim([-0.5 3.5]);
% Plot control inputs
figure('Name', 'Input');
plot(out.tout(1:size_of_time), control_input, 'LineWidth', 2);
hold on;
plot(out.tout(1:size_of_time), des_input, 'LineWidth', 2);
grid;
legend('u1_{Act}', 'u2_{Act}', 'u1_{Des}', 'u2_{Des}');