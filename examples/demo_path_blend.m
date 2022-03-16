% from vehicle vel to wheel vel
clc; clear all; close all;

addpath('..');
meow_startup;

defaultWps = [0 2.5 2.5 3.5 2.5;...
              0 0   2.5 5.0 6.0];

%% param
radius_tol = 0.5;

%% process
path = waypointsFittingBezier(defaultWps, radius_tol);

% sample traj
traj = path2traj(path);

%% animation
rbt3 = createRobot('diff', 'cuboid', 'y');
rbt3.name = 'rbt3';
figure; hold on; view(3); axis equal; grid on
set(gcf,'color','w');

plot(traj.trajPts(1,:), traj.trajPts(2,:));
plot(min(defaultWps(1,:))-1, min(defaultWps(2,:))-1);
plot(max(defaultWps(1,:))+1, max(defaultWps(2,:))+1);

for i=1:length(traj.vel)-1
    v = [traj.vel(i),0,0];
    w = [0,0,traj.vel(i) * traj.curvature(i)];
    dt= traj.dt(i);
    rbt3 = updateWheelKinetic(rbt3, v, w, dt);
    if 0 == mod(i,20)
        figR = drawRobot(rbt3);
        plot3(rbt3.pose.position(1),rbt3.pose.position(2),rbt3.pose.position(3),'.b');
%         makeGif(getframe(gcf), 'demo_path_blend');
        pause(0.001);
        delete(figR.group);
    end
end
drawRobot(rbt3);

%% plot
figure; hold on; axis equal
title('quintic Bezier Blend');
plot(defaultWps(1,:), defaultWps(2,:), 'sr');
plot(defaultWps(1,:), defaultWps(2,:), '--');

% draw tolrance circle
    % rectangle('Position',[x-r,y-r,2r,2r],'Curvature',[1,1]);
for i=2:length(defaultWps)-1
    x = defaultWps(1,i);
    y = defaultWps(2,i);
    r = radius_tol;
    rectangle('Position',[x-r,y-r,2*r,2*r],'Curvature',[1,1],'LineStyle',':');
end

% draw path seg
for i=1:path.size
    seg = path.seg(i);
    if length(seg.type) == 0
        continue;
    elseif (strcmp(seg.type, 'line'))
        plot(seg.ctrl_pts(1,:), seg.ctrl_pts(2,:), 'g');
    elseif (strcmp(seg.type, 'quinticBezier'))
        t = linspace(0,1, 100);
        B = bernsteinMatrix(5,t);
        P = seg.ctrl_pts';
        bezier = B*P;
        plot(bezier(:,1), bezier(:,2), 'c');
        scatter(P(:,1), P(:,2), 'filled');
    end
end

figure; grid on; hold on; view(3)
plot3(traj.trajPts(1,:), traj.trajPts(2,:), traj.curvature);
scatter3(traj.trajPts(1,:), traj.trajPts(2,:), traj.vel, 1, traj.acc);
colormap(cool);
plot3(-1,-1,0);
legend('curvature', 'velocity','Location','NorthEast');
title('traj curvature & velocity');

figure; grid on;
plot(traj.tangent(1,:), traj.tangent(2,:));
axis equal;
title('traj tangent vector');

