% Code intented to be upscale to 3D spatial case
% Please generalize it if you see it is poorly written for that case

clear;clc;close all

%% Load matlab data
fileName = "4_1.mat";
variableToLoad = {'x_true', 'x_true_second', 'u_second_value'}; % Load state pos first, then vel, then controls

state_filename = 'states_traj.csv';
ctrl_filename = 'ctrl_traj.csv';

[~, col] = size(variableToLoad);

S = load(fileName, variableToLoad{:});

%% Defining some hardcode parameters
numOfStateVars = 2; % The number of matrix corresponding to states (e.g. x_true & x_true_second -> 2)
n = 4; % Dimension of state (e.g. x & x_dot = 2)
numDimPerVar = n/numOfStateVars; % Number of dim w/in each var
dimControl = 2; % Control input dim for force (2 for planar)

%% Reconfigure matlab data

% Reconfigure state trajectories
trajLength = length(S.(variableToLoad{1})); % Length of state or control trajectory (should be the same)
states_traj = zeros(trajLength,n); % Hardcoded 4 rows for 2d planner
ctrl_traj = zeros(trajLength, dimControl);


%{
Currently reconfigure to:
[x_1 x_2, ..., x_n, xdot_1, xdot_2, ..., xdot_n].T

where x is position, and xdot is velocity
%}

% Pushing states into states_trajectory
for i=1:numOfStateVars
    % Column index example for numStatesPerVar = 2 | i=1 -> 1:2, 3:4, 5:6
    states_traj(1:trajLength, numDimPerVar*(i-1)+1:numDimPerVar*i) = S.(variableToLoad{i}).';
end

% Pushing control into control_trajectory
for i=1:(col-numOfStateVars)
    ctrl_traj(1:trajLength, numDimPerVar*(i-1)+1:numDimPerVar*i) = S.(variableToLoad{numOfStateVars+i}).';
end

%% Export to CSV file
writematrix(states_traj, state_filename);
writematrix(ctrl_traj, ctrl_filename);
