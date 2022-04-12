%% project root
basedir = fileparts(which('meow_startup.m'));

%% add lib
addpath(fullfile(basedir, 'utils'));
addpath(fullfile(basedir, 'utils/common/curve/bezier/'));
addpath(fullfile(basedir, 'utils/kinematic'));
addpath(fullfile(basedir, 'utils/path'));
addpath(fullfile(basedir, 'utils/speed'));

addpath(fullfile(basedir, 'third/RoboticsToolbox'));
% linux 64 version, ref: https://osqp.org/docs/get_started/matlab.html
addpath(fullfile(basedir, 'third/osqp'));

addpath(fullfile(basedir, 'visualization'));

