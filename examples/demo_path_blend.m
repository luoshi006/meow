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
traj.trajPts   = [];
traj.curvature = [];
traj.tangent   = [];

sample_step = 0.001;    % 1mm
for i=1:path.size
    seg = path.seg(i);
    if length(seg.type) == 0
        continue;
    elseif (strcmp(seg.type, 'line'))
        pts = [];
        num = ceil(norm(seg.ctrl_pts(:,1) - seg.ctrl_pts(:,2))/sample_step);
        pts(1,:) = linspace(seg.ctrl_pts(1,1), seg.ctrl_pts(1,2), num);
        pts(2,:) = linspace(seg.ctrl_pts(2,1), seg.ctrl_pts(2,2), num);
        traj.trajPts   = [traj.trajPts'; pts']';
        traj.curvature = [traj.curvature, zeros(1,num)];
        theta = atan2(seg.ctrl_pts(2,2) - seg.ctrl_pts(2,1),seg.ctrl_pts(1,2) - seg.ctrl_pts(1,1));
        dx = ones(1,num)*cos(theta)*sample_step;
        dy = ones(1,num)*sin(theta)*sample_step;
        traj.tangent   = [traj.tangent'; [dx; dy]']';
        % https://personal.math.ubc.ca/~CLP/CLP3/clp_3_mc/sec_curves.html
    elseif (strcmp(seg.type, 'quinticBezier'))
        num = ceil(lengthBezier(seg.ctrl_pts)/sample_step);
        t   = linspace(0,1,num);

        B = bernsteinMatrix(5,t);
        P = seg.ctrl_pts';
        pts = B*P;

        dB = bernsteinMatrix(4,t);
        dP = transpose(derivateBezier(seg.ctrl_pts));
        dpts = dB*dP;

        ddB = bernsteinMatrix(3,t);
        ddP = transpose(derivateBezier(dP'));
        ddpts = ddB*ddP;

        tangent = zeros(2,num);
        kappa   = zeros(1,num);
        for i=1:num
            tangent(:,i) = sample_step*dpts(i,:)';
            % calc curvature
            % k(u) = | f'(u) × f''(u) | / | f'(u) |^3
            kappa(i) = norm(cross([dpts(i,:),0], [ddpts(i,:),0]))/(norm(dpts(i,:))^3);
        end

        traj.trajPts   = [traj.trajPts'; pts]';
        traj.curvature = [traj.curvature, kappa];
        traj.tangent   = [traj.tangent'; tangent']';
    end
end


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
plot3(-1,-1,0);
title('traj curvature');

figure; grid on;
plot(traj.tangent(1,:), traj.tangent(2,:));
axis equal;
title('traj tangent vector');

