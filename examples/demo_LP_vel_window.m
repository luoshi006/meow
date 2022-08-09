%% A Forward Projection method for segments trajectory velocity planning
% refs: ./doc/velocity_planning.md

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


max_jerk = limit.jerk;

% sample traj and calc the trap vel profile;
traj = path2traj(path, limit);
[jp_idx_down, jp_vel] = findJumpDownPoints(traj);
jp_idx_up = findJumpUpPoints(traj, jp_idx_down);
jp_dist_down = traj.s(jp_idx_down);
jp_dist_up   = traj.s(jp_idx_up);

%% speed planning prepare
% Time-Optimal Feedrate Planning for Freeform Toolpaths for Manufacturing Applications

% https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-basis.html
% https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-derv.html#:~:text=Therefore%2C%20the%20derivative%20of%20a,.%2C%20Qn%2D1.

% sample point on s-s'
sample_step = max(min(...
                        min((limit.fast_speed^2)/(2*limit.fast_acc), (limit.slow_speed^2)/(2*limit.slow_acc))/2 ...
                        , 1)...
                    , 0.1);

traj_dist = traj.s(end);
ctrl_pts_num = ceil(traj_dist/sample_step);
check_pts_num_pre = ctrl_pts_num*2;     % check points in optimal problem,

k = 3;                              % order
d = 2;                              % degree
n = ctrl_pts_num - 1;               % ctrl_points_size = n + 1
m = n + k;                          % knots_size = m + 1

knots = augknt(linspace(0,1,n-k+1+2), k);   % clamped uniform
tau_ck_pts_pre = linspace(0,1,check_pts_num_pre);
[tau_ck_pts, check_pts_num, tau_jp_idx_down] = tau_insert(tau_ck_pts_pre, jp_dist_down/traj.s(end));
[tau_ck_pts, check_pts_num, tau_jp_idx_up]   = tau_insert(tau_ck_pts    , jp_dist_up  /traj.s(end));

tau_ctrl_pts_samp = linspace(0,1,ctrl_pts_num);

% sample the trap vel profile as control points initial guess
traj_idx_ctrl_pts = round(tau_ctrl_pts_samp*length(traj.s), 0);
traj_idx_ctrl_pts(1) = 1;   % fix 0 - 1

ctrl_pts = (traj.vel(traj_idx_ctrl_pts)/traj_dist).^2;

spl_vel = spmak(knots, ctrl_pts);
spl_acc = fnder(spl_vel);
spl_jrk = fnder(spl_acc);

% calc basis matrix for spline
spl_basis_vel = spcol(knots, k, tau_ck_pts);
spl_basis_acc = basis_der(1, k, tau_ck_pts, knots);
spl_basis_jrk = basis_der(2, k, tau_ck_pts, knots);

%  linear programming problems
dist_ck_pts = tau_ck_pts * traj_dist;

% assume in path2traj, sample step is 1mm;
traj_idx_ck_pts  = round(dist_ck_pts * 1000, 0);
traj_idx_ck_pts(1) = 1; traj_idx_ck_pts(end) = length(traj.s);
% make sure the jump point vel limit less than the lower one;
traj_idx_ck_pts(tau_jp_idx_down) = jp_idx_down;
traj_idx_ck_pts(tau_jp_idx_up)   = jp_idx_up;

%% first step to calc upper bound on q
%  min f * x
%      s.t.
%           A x  <= b
%           Aeq x = beq
%           lb <= x <= ub
f = -1*ones(1, check_pts_num) * spl_basis_vel;

L = traj_dist;
A = [spl_basis_vel',                    ... % vel always >0
     spl_basis_acc', -spl_basis_acc',   ... % |a| > |0.5 r' q'|
    ]';    % [2*sampleN] x [sampleN]        % r''' == 0, so do not consider jerk
    vimaxL = traj.vel_limit(traj_idx_ck_pts)/L; % (v/l)^2 >= q
    aimaxL = traj.acc_limit(traj_idx_ck_pts)/L; % 2a/l > q'
b = [vimaxL.^2, 2*aimaxL, 2*aimaxL];
Aeq = zeros(2*check_pts_num, ctrl_pts_num);
    % vel equal constraint
    Aeq(1,:) = spl_basis_vel(1,:);                              % vel(0) = 0;
    Aeq(check_pts_num,:) = spl_basis_vel(check_pts_num,:);      % vel(end) = 0;
    % acc equal constraint
    Aeq(check_pts_num+1,:) = spl_basis_acc(1,:);                        % acc(0) = 0;
    Aeq(check_pts_num*2,:)          = spl_basis_acc(check_pts_num,:);   % acc(end) = 0;
beq = zeros(1, 2*check_pts_num);
% Note: jump point constraint make the LP problem can not solve.
%     Aeq(tau_jp_idx_down,:)    = spl_basis_vel(tau_jp_idx_down,:);         % vel(jump) = vel_limit;
%     beq(tau_jp_idx_down) = jp_vel;
%     Aeq(check_pts_num+tau_jp_idx_down,:) = spl_basis_vel(tau_jp_idx_down,:);      % acc(jump) = 0;

lb  = zeros(1, ctrl_pts_num);
ub  = ones(1, ctrl_pts_num)*limit.fast_speed^2;

% LP solver
options = optimoptions(@linprog,'Display','iter');
tic
[x fval exit flag] = linprog(f,A,b,Aeq,beq,lb,ub,options);
toc
ctrl_pts_pseudo = x';
disp('===== first calc q* for pseudo-jerk =====')

%% windows
win_size = length(jp_idx_down) + 1;

win_idx_start = [1, jp_idx_up];
win_vel_start = [0, jp_vel];
win_idx_goal  = [jp_idx_down, length(traj.s)];
win_vel_goal  = [jp_vel, 0];

merge_dis = [];
merge_vel = [];
merge_acc = [];
merge_jrk = [];
for i = 1:win_size
    idx0 = win_idx_start(i);
    idx1 = win_idx_goal(i);

    win_traj_dist = traj.s(idx1) - traj.s(idx0);
    win_ctrl_pts_idx0 = round(traj.s(idx0)/sample_step, 0) + 1;
    win_ctrl_pts_idx1 = round(traj.s(idx1)/sample_step, 0);
    % win_ctrl_pts_num = ceil(win_traj_dist/sample_step);
    win_ctrl_pts_num = win_ctrl_pts_idx1 - win_ctrl_pts_idx0 + 1;

    win_chk_pts_num = win_ctrl_pts_num * 2;

    win_k = k;  % order
    win_d = d;  % degree
    win_n = win_ctrl_pts_num - 1;   % ctrl pts size = n + 1
    win_m = win_n + win_k;

    win_knots = augknt(linspace(0, 1, win_n-win_k+3), win_k);
    win_tau_chk_pts = linspace(0,1,win_chk_pts_num);
    % win_ctrl_pts = ctrl_pts_pseudo(win_ctrl_pts_idx0 : win_ctrl_pts_idx1);
    % win_ctrl_pts_psu = ctrl_pts_pseudo(win_ctrl_pts_idx0 : win_ctrl_pts_idx1);
    % win_spl_vel = spmak(win_knots, win_ctrl_pts);

    win_spl_basis_vel = spcol(win_knots, win_k, win_tau_chk_pts);
    win_spl_basis_acc = basis_der(1, win_k, win_tau_chk_pts, win_knots);
    win_spl_basis_jrk = basis_der(2, win_k, win_tau_chk_pts, win_knots);

    win_dist_chk_pts = win_tau_chk_pts * win_traj_dist*1000 + idx0;
    win_traj_idx_chk_pts = round(win_dist_chk_pts, 0);
    win_traj_idx_chk_pts(1) = idx0; win_traj_idx_chk_pts(end) = idx1;

    %% pseudo jerk for window
    win_fp = -1*ones(1, win_chk_pts_num)*win_spl_basis_vel;
    win_L = win_traj_dist;
    win_Ap = [win_spl_basis_vel',                        ...
              win_spl_basis_acc', -win_spl_basis_acc'    ...
            ]';
            w_v_l = traj.vel_limit(win_traj_idx_chk_pts)/win_L;
            w_a_l = traj.acc_limit(win_traj_idx_chk_pts)/win_L;
    win_bp = [w_v_l.^2, 2*w_a_l, 2*w_a_l];
    win_Aeqp = zeros(2*win_chk_pts_num, win_ctrl_pts_num);
            win_Aeqp(1,:) = win_spl_basis_vel(1,:);
            win_Aeqp(win_chk_pts_num,:)   = win_spl_basis_vel(win_chk_pts_num,:);
            win_Aeqp(win_chk_pts_num+1,:) = win_spl_basis_acc(1,:);
            win_Aeqp(win_chk_pts_num*2,:) = win_spl_basis_acc(win_chk_pts_num,:);
    win_beqp = zeros(1, 2*win_chk_pts_num);
            win_beqp(1) = (win_vel_start(i)/win_L)^2;
            win_beqp(win_chk_pts_num) = (win_vel_goal(i)/win_L)^2;
    win_lbp = zeros(1, win_ctrl_pts_num);
    win_ubp = ones(1, win_ctrl_pts_num)*(limit.fast_speed/win_L)^2;
    options = optimoptions(@linprog,'Display','iter');

    tic
    [x fval exit flag] = linprog(win_fp,win_Ap,win_bp,win_Aeqp,win_beqp,win_lbp,win_ubp,options);
    toc
    win_ctrl_pts_psu = x;
    disp(['seg ', num2str(i), ' pseudo-jerk solved.']);

    %% solver
    win_f = -1*ones(1, win_chk_pts_num)*win_spl_basis_vel;
    win_A = [win_spl_basis_vel',                        ...
             win_spl_basis_acc', -win_spl_basis_acc',   ...
             win_spl_basis_jrk', -win_spl_basis_jrk',   ...
             eye(win_ctrl_pts_num)
            ]';
            w_sqt_q = (win_spl_basis_vel*win_ctrl_pts_psu)'.^0.5;
            w_sqt_q(w_sqt_q<0.01) = 0.01;
            w_j_l = limit.jerk/win_L * ones(1,win_chk_pts_num) ./ w_sqt_q;
    win_b = [w_v_l.^2, 2*w_a_l, 2*w_a_l, 2*w_j_l, 2*w_j_l, win_ctrl_pts_psu'];
    win_Aeq = zeros(2*win_chk_pts_num, win_ctrl_pts_num);
            win_Aeq(1,:) = win_spl_basis_vel(1,:);
            win_Aeq(win_chk_pts_num,:)   = win_spl_basis_vel(win_chk_pts_num,:);
            win_Aeq(win_chk_pts_num+1,:) = win_spl_basis_acc(1,:);
            win_Aeq(win_chk_pts_num*2,:) = win_spl_basis_acc(win_chk_pts_num,:);
    win_beq = zeros(1, 2*win_chk_pts_num);
            win_beq(1) = (win_vel_start(i)/win_L)^2;
            win_beq(win_chk_pts_num) = (win_vel_goal(i)/win_L)^2;
    win_lb = zeros(1, win_ctrl_pts_num);
    win_ub = ones(1, win_ctrl_pts_num)*(limit.fast_speed/win_L)^2;

    win_lp_opt = optimoptions(@linprog, 'Display', 'iter');
    tic
    [win_x] = linprog(win_f, win_A, win_b, win_Aeq, win_beq, win_lb, win_ub, win_lp_opt);
    toc
    disp(['seg ', num2str(i), ' profile solved.']);

    % segments visualization
    win_tau_disp = linspace(0,1,idx1-idx0+1);
    win_basis_disp_vel = spcol(win_knots, win_k, win_tau_disp);
    win_basis_disp_acc = basis_der(1,win_k,win_tau_disp,win_knots);
    win_basis_disp_jrk = basis_der(2,win_k,win_tau_disp,win_knots);

    win_disp_spl_dist = win_tau_disp*win_traj_dist;
    win_disp_spl_vel  = win_L*(win_basis_disp_vel*win_x).^0.5;
    win_disp_spl_acc  = win_L*0.5*win_basis_disp_acc*win_x;
    win_disp_spl_q = (win_basis_disp_vel*win_x);
    win_disp_spl_q(win_disp_spl_q< 0.001) = 0;
    win_disp_spl_jrk  = win_L*0.5*(win_basis_disp_jrk*win_x) .* win_disp_spl_q .^0.5;

    win_disp_ctrl_pts_dist = tau_ctrl_pts_samp(win_ctrl_pts_idx0:win_ctrl_pts_idx1)*traj_dist;
    win_disp_ctrl_pts_2_vel = win_L*win_x.^0.5;

    % merge
    merge_dis = [merge_dis, win_disp_spl_dist + traj.s(idx0)];
    merge_vel = [merge_vel, win_disp_spl_vel'];
    merge_acc = [merge_acc, win_disp_spl_acc'];
    merge_jrk = [merge_jrk, win_disp_spl_jrk'];

    if i ~= win_size
        idx2 = win_idx_start(i+1);
        dist_tau = linspace(0,1, idx2-idx1+1);
        dist = dist_tau * (traj.s(idx2)-traj.s(idx1));
        merge_dis = [merge_dis, dist + traj.s(idx1)];
        merge_vel = [merge_vel, win_vel_goal(i)*ones(1,length(dist_tau))];
        merge_acc = [merge_acc, zeros(1,length(dist_tau))];
        merge_jrk = [merge_jrk, zeros(1,length(dist_tau))];
    end
%     figure;
%     plot(win_disp_spl_dist, win_disp_spl_vel); grid on;
end

%% visualization
% data prepare
tau_disp   = linspace(0,1,length(traj.s));
basis_disp_vel = spcol(knots, k, tau_disp);
basis_disp_acc = basis_der(1, k, tau_disp, knots);
basis_disp_jrk = basis_der(2, k, tau_disp, knots);

% plot(basis_disp_vel)
% velocity from pseudo solver
disp_spl_dist    = tau_disp*traj_dist;
disp_spl_vel_ub  = L*(basis_disp_vel*ctrl_pts_pseudo').^0.5;  % vel = r' \dot{s}
% acc profile
disp_spl_acc = L*0.5*basis_disp_acc*ctrl_pts_pseudo';
% jerk profile
disp_spl_jrk_psd = L*0.5*(basis_disp_jrk*ctrl_pts_pseudo') .* ((basis_disp_vel*ctrl_pts_pseudo').^0.5);

% control point -> vel
disp_ctrl_pts_dist  = tau_ctrl_pts_samp*traj_dist;
disp_ctrl_pts_2_vel = L*ctrl_pts_pseudo'.^0.5;

% jump points
jp_pts_dist_down = traj.s(traj_idx_ck_pts(tau_jp_idx_down));
jp_pts_basis_vel_down = spcol(knots, k, tau_ck_pts(tau_jp_idx_down));
jp_pts_vel_down = L*(jp_pts_basis_vel_down*ctrl_pts_pseudo').^0.5;

jp_pts_dist_up = traj.s(traj_idx_ck_pts(tau_jp_idx_up));
jp_pts_basis_vel_up = spcol(knots, k, tau_ck_pts(tau_jp_idx_up));
jp_pts_vel_up = L*(jp_pts_basis_vel_up*ctrl_pts_pseudo').^0.5;

% plot
figure;
% subplot(2,2,[1 3]);
hold on; grid on;
ylabel('vel'); xlabel('s');
title('$\dot{s}-s$','interpreter','latex', 'FontSize', 18);
plot(traj.s, traj.vel);
plot(disp_spl_dist, disp_spl_vel_ub);
plot(merge_dis, merge_vel);
plot(jp_pts_dist_down, jp_pts_vel_down, 'sk');
plot(jp_pts_dist_up  , jp_pts_vel_up  , 'sk');

% plot(disp_ctrl_pts_dist, disp_ctrl_pts_2_vel, 's');

plot(traj.s, traj.vel_limit, 'r--');

legend('trap','pseudo','LPW','jump point down','jump point up', 'limit');

figure; hold on;
plot(disp_spl_dist, disp_spl_acc);
plot(merge_dis, merge_acc);
plot(traj.s, traj.acc_limit, 'r--');
plot(traj.s, -traj.acc_limit, 'r--');
grid on;
legend('pseudo', 'LPW')
title('Acc');

figure; hold on;
plot(disp_spl_dist, disp_spl_jrk_psd);
plot(merge_dis, merge_jrk);
plot(traj.s, ones(1,length(traj.s))*limit.jerk, 'r--');
plot(traj.s, -ones(1,length(traj.s))*limit.jerk, 'r--');
grid on;
legend('pseudo-jerk', 'LPW');
title('Jerk');


function sp_basis_der = basis_der(nderiv, k, tau, knots)
    % https://github.com/lewisli/PFA-CFCA/blob/master/thirdparty/fda_matlab/notneeded/bspline.m
    nderivp1 = nderiv + 1;
    onevec  = ones(nderivp1,1);
    xmat    = reshape(tau,[1,length(tau)]);
    tau_tmp = reshape(onevec * xmat,[1,length(tau)*nderivp1]);
    basismat = spcol(knots,k,tau_tmp);
    index_der = nderivp1 : nderivp1 : length(tau)*nderivp1;
    sp_basis_der  = basismat(index_der,:);
end

function [idx, vel] = findJumpDownPoints(traj)
    idx = [];
    vel = [];
    for i=2:length(traj.vel_limit)
        if traj.vel_limit(i) < traj.vel_limit(i-1)
            idx = [idx, i];
            vel = [vel, traj.vel_limit(i)];
        end
    end
end

function [idxu] = findJumpUpPoints(traj, idxd)
    idxu = [];
    for i=1:length(idxd)
        for j = idxd(i)+1 : length(traj.vel_limit)
            if traj.vel_limit(j) > traj.vel_limit(idxd(i))
                idxu = [idxu, j];
                break;
            end
        end
    end
end

function [tau3, num, tau3_2_idx] = tau_insert(tau1, tau2)
% tau3 = tau1 + tau2 then sort.
    len3 = length(tau1) + length(tau2);
    eps  = 1E-7;
    tau3 = []; tau3_2_idx = [];
    i1 = 1; % idx for tau1;
    i2 = 1; % idx for tau2;
    i3 = 1; % idx for tau3;
    while i3 <= len3
        if tau1(i1) < tau2(i2) - eps
            tau3(i3) = tau1(i1);
            i1 = i1 + 1;
        elseif tau1(i1) > tau2(i2) + eps
            tau3(i3) = tau2(i2);
            i2 = i2 + 1;
            tau3_2_idx = [tau3_2_idx, i3];
        else    % equal
            tau3(i3) = tau1(i2);
            len3 = len3 - 1;
            i1 = i1 + 1;
            i2 = i2 + 1;
            tau3_2_idx = [tau3_2_idx, i3];
        end
        i3 = i3 + 1;
        if i2 > length(tau2)
            tau3 = [tau3, tau1(i1:end)];
            break;
        end
    end
    num = len3;
end

function v_g = heuristicVel(traj, idx)
% TODO: forward project search goal vel
    v_g = traj.vel_limit(idx);
end
