% Trajectory Planning of Robot Manipulators with MATLAB and Simulink
% Startup script
%
% Copyright 2019 The Mathworks, Inc.

clear, clc, close all;
disp('Starting Robot Manipulator Trajectory Example...')

% Set up the MATLAB search path
rootDir = fileparts(which(mfilename));
addpath(genpath('matlab'),genpath('simulink'),genpath('utilities'));

% Set the code generation and cache folders to a work folder
% If the folder does not exist, create it
if ~isfolder('work')
    mkdir('work');
end
Simulink.fileGenControl('set','CacheFolder','work','CodeGenFolder','work');