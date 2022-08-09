%% Time optimal velocity planning using Linear Programming in the form of a second order B-Spline

clc; clear all; close all;

addpath('..');
meow_startup;

defaultWps = [0 2.5 2.5 3.5 2.5;...
              0 0   2.5 5.0 6.0];

%% param
radius_tol = 0.5;

%% process
path = waypointsFittingBezier(defaultWps, radius_tol);

limit.fast_speed = 1;
limit.slow_speed = 0.5;
limit.fast_acc   = 1;
limit.slow_acc   = 0.3;
limit.jerk       = 5;

global max_jerk;
max_jerk = limit.jerk;

% sample traj and calc the trap vel profile;
traj = path2traj(path, limit);

%% speed planning prepare
% Time-Optimal Feedrate Planning for Freeform Toolpaths for Manufacturing Applications

% https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-basis.html
% https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-derv.html#:~:text=Therefore%2C%20the%20derivative%20of%20a,.%2C%20Qn%2D1.

global sample_N;
% sample point on s-s'
ds = max(min(...
            min((limit.fast_speed^2)/(2*limit.fast_acc), (limit.slow_speed^2)/(2*limit.slow_acc))/2 ...
            , 1)...
        , 0.1);

traj_dist = traj.s(end);
ctrl_N = ceil(traj_dist/ds);
sample_N = ctrl_N*2;
k = 3;      % order
d = 2;      % degree
n = ctrl_N - 1;      % ctrl_points_size = n + 1
m = n + k;  % knots_size = m + 1

knots = augknt(linspace(0,1,n-k+1+2), k);   % clamped uniform
tau = linspace(0,1,sample_N);
ctrl_sp = linspace(0,1,ctrl_N);

c_idx = round(ctrl_sp*length(traj.s), 0);
c_idx(1) = 1;   % fix 0 - 1

ctrl_pts = (traj.vel(c_idx)/traj_dist).^2;

profile    = spmak(knots, ctrl_pts);
profile_d  = fnder(profile);
profile_dd = fnder(profile_d);

global basis_vel basis_jrk L;
basis_vel    = spcol(knots, k, tau);

% https://github.com/lewisli/PFA-CFCA/blob/master/thirdparty/fda_matlab/notneeded/bspline.m
nderiv = 1;
nderivp1 = nderiv + 1;
onevec  = ones(nderivp1,1);
xmat    = reshape(tau,[1,length(tau)]);
tau_tmp = reshape(onevec * xmat,[1,length(tau)*nderivp1]);
basismat = spcol(knots,k,tau_tmp);
index_acc = nderivp1 : nderivp1 : length(tau)*nderivp1;

basis_acc  = basismat(index_acc,:);

nderiv = 2;
nderivp1 = nderiv + 1;
onevec  = ones(nderivp1,1);
xmat    = reshape(tau,[1,length(tau)]);
tau_tmp = reshape(onevec * xmat,[1,length(tau)*nderivp1]);
basismat = spcol(knots,k,tau_tmp);
index_jerk = nderivp1 : nderivp1 : length(tau)*nderivp1;

basis_jrk = basismat(index_jerk,:);

%  linear programming problems
breakpts = tau;
tau_s    = tau * traj_dist;
% assume the traj ds = 1mm;
tau_idx  = round(tau_s * 1000, 0);
tau_idx(1) = 1; tau_idx(end) = length(traj.s);

%% first step to calc upper bound on q
f = -1*ones(1, sample_N) * basis_vel;

L = traj_dist;
A = [basis_vel', basis_acc', -basis_acc']';    % [2*sampleN] x [sampleN]
    vimaxL = traj.vel_limit(tau_idx)/L;
    aimaxL = traj.acc_limit(tau_idx)/L;
b = [vimaxL.^2, 2*aimaxL, 2*aimaxL];
Aeq = zeros(2*sample_N, ctrl_N);
    Aeq(1,:)        = basis_vel(1,:);           % vel(0) = 0;
    Aeq(sample_N,:) = basis_vel(sample_N,:);    % vel(end) = 0;
    Aeq(sample_N+1,:)        = basis_acc(1,:);        % acc(0) = 0;
    Aeq(sample_N+sample_N,:) = basis_acc(sample_N,:); % acc(end) = 0;
beq = zeros(1, 2*sample_N);
lb  = zeros(1, ctrl_N);
ub  = ones(1, ctrl_N)*(limit.fast_speed/L)^2;

% LP
options = optimoptions(@linprog,'Display','iter');
tic
[x fval exit flag] = linprog(f,A,b,Aeq,beq,lb,ub,options);
toc
% ctrl_pts = x';
disp('===== first calc q* for pseudo-jerk =====')

%% second step using Pseudo-Jerk
f2 = f;

A2 = [basis_vel', basis_vel', basis_acc', -basis_acc', basis_jrk', -basis_jrk']';
    sqtq   = (basis_vel*x)'.^0.5;
    sqtq(sqtq<0.01) = 0.01;
    jimaxLq = limit.jerk/L * ones(1,sample_N) ./ sqtq;
b2 = [vimaxL.^2, (basis_vel*x)', 2*aimaxL, 2*aimaxL, 2*jimaxLq, 2*jimaxLq];
Aeq2 = Aeq;
beq2 = beq;
lb2 = lb;
ub2 = ub;
tic
[x fval exit flag] = linprog(f2,A2,b2,Aeq2,beq2,lb2,ub2,options);
toc
ctrl_pts = x';
disp('===== second calc q (LP) =====')

%% 3rd step using SQP for nonlinear obj

eps_q = 0.001;
x0 = x+eps_q;
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
tic
[x,fval,exitflag,output] = fmincon(@objectiveFun,x0,A,b,Aeq,beq,lb,ub,@jerk_constraint);
toc
% x(x<0) = 0;
disp(output);
disp('===== third calc q (SQP) =====')

%% plot
figure; hold on; axis equal
title('quintic Bezier Blend');
plot(defaultWps(1,:), defaultWps(2,:), 'sr');
plot(defaultWps(1,:), defaultWps(2,:), '--');

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
%         scatter(P(:,1), P(:,2), 'filled');
    end
end

figure;
subplot(2,2,[1 3]);
hold on; grid on;
ylabel('vel'); xlabel('s');
title('$\dot{s}-s$','interpreter','latex', 'FontSize', 18);
plot(traj.s, traj.vel);
tau_disp   = linspace(0,1,length(traj.s));
basis_disp = spcol(knots, k, tau_disp);
% plot(basis_disp)
plot(tau_disp*traj_dist, L*(basis_disp*ctrl_pts').^0.5);
plot(ctrl_sp*traj_dist, L*ctrl_pts'.^0.5, 's');

plot(tau_disp*traj_dist, L*(basis_disp*x).^0.5);
plot(ctrl_sp*traj_dist, L*x.^0.5, 's');
legend('trap','LP', 'LP ctrl pts', 'SQP', 'SQP ctrl pts');


nderiv = 1;
nderivp1 = nderiv + 1;
onevec  = ones(nderivp1,1);
xmat    = reshape(tau_disp,[1,length(tau_disp)]);
tau_tmp = reshape(onevec * xmat,[1,length(tau_disp)*nderivp1]);
basismat = spcol(knots,k,tau_tmp);
index_acc = nderivp1 : nderivp1 : length(tau_disp)*nderivp1;
basis_acc_disp = basismat(index_acc,:);

subplot(2,2,2);
plot(tau_disp*traj_dist, L*0.5*basis_acc_disp*x);  hold on; grid on;
plot(tau_disp*traj_dist, L*0.5*basis_acc_disp*ctrl_pts');
plot(traj.s, traj.acc_limit, 'r--');
plot(traj.s, -traj.acc_limit, 'r--');
legend('SQP', 'LP');
ylabel('acc'); xlabel('s');
title('acc & jerk')

nderiv = 2;
nderivp1 = nderiv + 1;
onevec  = ones(nderivp1,1);
xmat    = reshape(tau_disp,[1,length(tau_disp)]);
tau_tmp = reshape(onevec * xmat,[1,length(tau_disp)*nderivp1]);
basismat = spcol(knots,k,tau_tmp);
index_jerk = nderivp1 : nderivp1 : length(tau_disp)*nderivp1;
basis_jerk_disp = basismat(index_jerk,:);

subplot(2,2,4);
plot(tau_disp*traj_dist, L*0.5*(basis_jerk_disp*x).*((basis_disp*x).^0.5));
hold on;
plot(tau_disp*traj_dist, L*0.5*(basis_jerk_disp*ctrl_pts').*((basis_disp*ctrl_pts').^0.5));

plot(traj.s, ones(1,length(traj.s))*limit.jerk, 'r--');
plot(traj.s, -ones(1,length(traj.s))*limit.jerk, 'r--');
legend('SQP', 'LP');
ylabel('jerk'); xlabel('s'); grid on;



%% func for SQP
function obj = objectiveFun(x)
    global basis_vel basis_jrk sample_N L;
    cost_time = -1*ones(1, sample_N) * basis_vel * x;
    cost_jerk = ones(1, sample_N) * ((0.5*L*basis_jrk*x).^2 .* (basis_vel*x));
    w_j = 0.0001;
    w_t = 1 - w_j;
    obj =  w_t*cost_time + w_j*cost_jerk;
end

function [c,ceq] = jerk_constraint(x)
    global basis_jrk basis_vel L sample_N max_jerk;
    ceq = [];
    c = [];
    % abs() is so nonlinear for constraint;
    c1 = (L*0.5*basis_jrk*x) .* ((basis_vel*x) .^ 0.5) - max_jerk * ones(1,sample_N)';
    c2 = -(L*0.5*basis_jrk*x) .* ((basis_vel*x) .^ 0.5) - max_jerk * ones(1,sample_N)';
    c = [c1; c2];
end
