% from vehicle vel to wheel vel
clc; clear all; close all;

addpath('..');
meow_startup;

rbt2 = createRobot('diff', 'cuboid', 'y');
rbt2.name = 'rbt2';

figure; hold on; view(3); axis equal; grid on
set(gcf,'color','w');
plot3(-1,-1,0); plot3(3,2,2);

%% stright line

dt = 0.02;
v = [1,0,0];      % m/s
w = [0,0,0];      % rad/s

for i=1:1/dt
    rbt2 = updateWheelInvKinetic(rbt2, v, w, dt);
    figR = drawRobot(rbt2);
%     makeGif(getframe(gcf), 'demo_inv_kinetic');
    pause(dt);
    delete(figR.group);
end

% pure rotate
v = [0 0 0];
w = [0 0 1];
for i=1:(pi/2)/dt
    rbt2 = updateWheelInvKinetic(rbt2, v, w, dt);
    figR = drawRobot(rbt2);
%     makeGif(getframe(gcf), 'demo_inv_kinetic');
    pause(dt);
    delete(figR.group);
end

% circle 
v = [0 0 1];
w = [0 0 1];
for i=1:round(pi/2,1)/dt
    rbt2 = updateWheelInvKinetic(rbt2, v, w, dt);
    figR = drawRobot(rbt2);
%     makeGif(getframe(gcf), 'demo_inv_kinetic');
    pause(dt);
    delete(figR.group);
end

figR = drawRobot(rbt2);