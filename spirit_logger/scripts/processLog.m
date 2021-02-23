function processLog(varargin)
% Process a bagfile and save the resulting images
close all;clc;

% If a trial name is provided, use that to save everything
if nargin>0
    % Leave these as is so the default is to animate and save
    trialName = varargin{1};
    bAnimate = true;
    bSave = true;
else
    % Specify the trial name and settings
    trialName = 'spirit_log_current';
    bAnimate = false;
    bSave = false;
end

% Check that this is the right current directory otherwise paths won't work
if ~endsWith(pwd, 'spirit-software/spirit_logger/scripts')
    error('This script must be run from spirit-software/spirit_logger/scripts/');
end

% Import URDF
figure
spirit40 = importrobot('../../spirit_simulator/spirit_description/urdf/spirit.urdf');
% homeConfig = homeConfiguration(spirit40);
% show(spirit40,homeConfig);

% Load the data
[data, trialName] = parseSpiritBag(trialName);
state_estimate = data.state_estimate;
state_ground_truth = data.state_ground_truth;
state_trajectory = data.state_trajectory;

% Plot the state
[COMTrajFig, linearStateFig, angularStateFig, jointPositionFig, jointVelocityFig, jointEffortFig] = plotState(state_trajectory);
% plotState(stateEstimate);

% Compute and plot the toe forces - in progress
% toeForces = getToeForces(spirit40, stateEstimate);
% plotToeForces(toeForces);

% Save the data if desired
logDir = [];
if bSave
    logDir = saveLog(trialName, COMTrajFig, linearStateFig, angularStateFig, jointPositionFig, jointVelocityFig, jointEffortFig);
end

% Animate and save if desired
if bAnimate
    videosDir = fullfile(logDir,'videos/');
    animateData(spirit40,state_trajectory, fullfile(videosDir, trialName), bSave);
end