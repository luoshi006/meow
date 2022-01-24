%% project root
basedir = fileparts(which('meow_startup.m'));

%% add lib
addpath(fullfile(basedir, 'utils'));
addpath(fullfile(basedir, 'utils/kinematic'));
addpath(fullfile(basedir, 'third/RoboticsToolbox'));
addpath(fullfile(basedir, 'visualization'));

