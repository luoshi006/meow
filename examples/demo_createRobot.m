clc; clear all; close all;

addpath('..');
meow_startup;

rbt1 = createRobot('diff', 'cuboid', 'y');
rbt1.name = 'rbt1';

wp = [0 2 6;...
      0 0 6];
rd1 = createRoad(wp);
% drawRoad(rd1);

view(3);
for i=1:100
    rbt1.wheels(1).rotAngle = deg2rad(i);
    rbt1.wheels(2).rotAngle = deg2rad(i);
    hdl = drawRobot(rbt1);
    makeGif(getframe(gcf), 'robot_diff');
    pause(0.01);
    delete(hdl.group);
end

drawRobot(rbt1);