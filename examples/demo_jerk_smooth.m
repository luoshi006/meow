%% ref: Toward a More Complete, Flexible, and Safer Speed Planning for Autonomous Driving via Convex Optimization

% autoware_repo::src/universe/autoware/planning/motion_velocity_smoother/src/smoother/jerk_filtered_smoother.cpp
%               -- without time efficiency objective
%               -- fitting the semi s-shape velocity profile, using QP

close all; clear all; clc;

addpath('..');
meow_startup;

%% init
v0 = 0.;
a0 = 0;
sf = 1;
vs = 0;
as = 0;

vmax  = 1;
amax  = 2;
dmax  = 1;
jmax  = 7;

%% jerk smooth
sample_n = ceil(sf/0.001);
profile.s = linspace(0, sf, sample_n);
profile.v = zeros(1, sample_n);
profile.a = zeros(1, sample_n);
profile.v(1) = v0;
profile.a(1) = a0;

% forward jerk
profilef = profile;
cur_vel = v0;
cur_acc = a0;
for i=2:sample_n
    ds = profilef.s(i) - profilef.s(i-1);
    max_dt = (6.0*ds/jmax)^(1/3);
    dt = min(ds/max(cur_vel, 1E-6), max_dt);

    if (cur_acc + jmax*dt >= amax)
        tmp_jerk = min((amax - cur_acc)/dt, jmax);
        cur_vel  = cur_vel + cur_acc*dt + 0.5*tmp_jerk*dt*dt;
        cur_acc  = amax;
    else
        cur_vel  = cur_vel + cur_acc*dt + 0.5*jmax*dt*dt;
        cur_acc  = cur_acc + jmax*dt;
    end

    if cur_vel > vmax
        cur_vel = vmax;
        cur_acc = 0;
    end

    profilef.v(i) = cur_vel;
    profilef.a(i) = cur_acc;
end

% backward jerk
profileb = profile;
profileb.v(sample_n) = vs;
profileb.a(sample_n) = as;

cur_vel = vs;
cur_acc = as;
for i=sample_n-1:-1:1
    ds = profileb.s(i+1) - profileb.s(i);
    max_dt = (6.0*ds/jmax)^(1/3);
    dt = min(ds/max(cur_vel, 1E-6), max_dt);

    if (cur_acc + jmax*dt >= dmax)
        tmp_jerk = min((dmax - cur_acc)/dt, jmax);
        cur_vel  = cur_vel + cur_acc*dt + 0.5*tmp_jerk*dt*dt;
        cur_acc  = dmax;
    else
        cur_vel  = cur_vel + cur_acc*dt + 0.5*jmax*dt*dt;
        cur_acc  = cur_acc + jmax*dt;
    end

    if cur_vel > vmax
        cur_vel = vmax;
        cur_acc = 0;
    end

    profileb.v(i) = cur_vel;
    profileb.a(i) = cur_acc;

end

% merge jerk smooth
profile = profilef;
if (profileb.v(1) < v0)
    cur_vel = v0;
    cur_acc = a0;
    i = 1;
    while (profileb(i) < cur_vel) && (i < sample_n)
        profile.v(i) = cur_vel;
        profile.a(i) = cur_acc;

        ds = profile.s(i+1) - profile.s(i);
        max_dt = (6.0*ds/jmax)^(1./3.);
        dt = min(ds/max(cur_vel, 1E-6), max_dt);

        if cur_acc - jmax*dt < - dmax
            tmp_jerk = max((-dmax - cur_acc)/dt, -jmax);
            cur_vel = cur_vel + cur_acc*dt + 0.5*tmp_jerk*dt*dt;
            cur_acc = max(cur_acc+tmp_jerk*dt, -dmax);
        else
            cur_vel = cur_vel + cur_acc*dt + 0.5*(-jmax)*dt*dt;
            cur_acc = cur_acc + (-jmax)*dt;
        end

        if cur_vel > profile.v(i)
            cur_vel = profile.v(i);
        end
        i = i+1;
    end
end
disp(['while ending at ', num2str(i)]);
for j=i:sample_n
    if profilef.v(j) < profileb.v(j)
        profile.v(j) = profilef.v(j);
    else
        profile.v(j) = profileb.v(j);
    end
end

%% objective function
l_variables = 5*sample_n;
l_constraints = 4*sample_n + 1;
IDX_B0 = 0;
IDX_A0 = sample_n;
IDX_DELTA0 = 2 * sample_n;
IDX_SIGMA0 = 3 * sample_n;
IDX_GAMMA0 = 4 * sample_n;

A = zeros(l_constraints, l_variables);
P = zeros(l_variables,   l_variables);
lower_bound = zeros(1,   l_constraints);
upper_bound = zeros(1,   l_constraints);
q = zeros(1, l_variables);
w_s = 0.1;
w_v = 10000;
w_a = 5000;
w_j = 200;

% jerk smooth
for i=1:sample_n-1
    ref_vel = profile.v(i);
    interval_dist = max(profile.s(i+1)-profile.s(i), 0.00001);
    w_x_ds_inv = (1./interval_dist) * ref_vel;

    P(IDX_A0+i, IDX_A0+i) = P(IDX_A0+i, IDX_A0+i) + w_s * w_x_ds_inv * w_x_ds_inv * interval_dist;
%     disp(['1 - ',num2str(min(eig(P)))]);
    P(IDX_A0+i, IDX_A0+i+1) = P(IDX_A0+i, IDX_A0+i+1) - w_s * w_x_ds_inv * w_x_ds_inv * interval_dist;
%     disp(['2 - ',num2str(min(eig(P)))]);
    P(IDX_A0+i+1, IDX_A0+i) = P(IDX_A0+i+1, IDX_A0+i) - w_s * w_x_ds_inv * w_x_ds_inv * interval_dist;
%     disp(['3 - ',num2str(min(eig(P)))]);
    P(IDX_A0+i+1, IDX_A0+i+1) = P(IDX_A0+i+1, IDX_A0+i+1) + w_s * w_x_ds_inv * w_x_ds_inv * interval_dist;
%     disp(['4 - ',num2str(min(eig(P)))]);
end

% disp(min(eig(P)));
w_iod = 10000;
for i=1:sample_n
%     q(IDX_B0+i) = -1.;
    v_max = max(profile.v(i), 0.02);
    q(IDX_B0+i) = -1. * w_iod / (v_max*v_max);
    if (i<sample_n)
        q(IDX_B0+i) = q(IDX_B0+i) * max(profile.s(i+1)-profile.s(i), 0.00001);
    else
        q(IDX_B0+i) = q(IDX_B0+i) * max(profile.s(i)-profile.s(i-1), 0.00001);
    end

    P(IDX_DELTA0+i, IDX_DELTA0+i) = P(IDX_DELTA0+i, IDX_DELTA0+i) + w_v;
    P(IDX_SIGMA0+i, IDX_SIGMA0+i) = P(IDX_SIGMA0+i, IDX_SIGMA0+i) + w_a;
    P(IDX_GAMMA0+i, IDX_GAMMA0+i) = P(IDX_GAMMA0+i, IDX_GAMMA0+i) + w_j;
end

% disp(min(eig(P)));


% constraint matrix
constr_idx = 1;
% soft constraint velocity limit: 0 < b-delta < v_max^2
for i=1:sample_n
    A(constr_idx, IDX_B0+i) = 1.;
    A(constr_idx, IDX_DELTA0+i) = -1;
    upper_bound(constr_idx) = profile.v(i)^2;
    lower_bound(constr_idx) = 0.;

    constr_idx = constr_idx + 1;
end

% soft constraint acceleration limit: a_min < a-sigma < a_max
for i=1:sample_n
    A(constr_idx, IDX_A0+i) = 1.;       % a_i
    A(constr_idx, IDX_SIGMA0+i) = -1.;  % -sigma_i

    stop_vel = 0.001;
    stop_decel = 0;
    if profile.v(i) < stop_vel
        upper_bound(constr_idx) = stop_decel;
        lower_bound(constr_idx) = stop_decel;
    else
        upper_bound(constr_idx) = amax;
        lower_bound(constr_idx) = -dmax;
    end

    constr_idx = constr_idx + 1;
end

% soft constraint jerk limit : jerk_min < pseudo_jerk[i]*ref_vel[i] - gamma[i] < jerk_max
%   -->> jerk_min * ds < (a[i+1]-a[i])*ref_vel[i] - gamma[i]*ds < jerk_max*ds
zero_vel_thr = 0.3;
for i=1:sample_n-1
    ref_vel = max(profile.v(i), zero_vel_thr);
    ds = max(profile.s(i+1)-profile.s(i), 0.00001);
    A(constr_idx, IDX_A0+i) = -ref_vel;     % -a[i]*ref_vel
    A(constr_idx, IDX_A0+i+1) = ref_vel;    %  a[i+1]*ref_vel
    A(constr_idx, IDX_GAMMA0+i) = -ds;      % -gamma[i]*ds
    upper_bound(constr_idx) =  jmax * ds;
    lower_bound(constr_idx) = -jmax * ds;

    constr_idx = constr_idx + 1;
end

% b' = 2a ... (b(i+1)-b(i)) / ds = 2a(i)
for i=1:sample_n-1
    A(constr_idx, IDX_B0+i) = -1.;
    A(constr_idx, IDX_B0+i+1) = 1.;
    A(constr_idx, IDX_A0+i) = -2. * max(profile.s(i+1)-profile.s(i), 0.00001);
    upper_bound(constr_idx) = 0.;
    lower_bound(constr_idx) = 0.;

    constr_idx = constr_idx + 1;
end

% initial condition
A(constr_idx, IDX_B0+1) = 1.;
upper_bound(constr_idx) = v0*v0;
lower_bound(constr_idx) = v0*v0;
constr_idx = constr_idx + 1;

A(constr_idx, IDX_A0+1) = 1.;
upper_bound(constr_idx) = a0;
lower_bound(constr_idx) = a0;
constr_idx = constr_idx + 1;

A(constr_idx, IDX_B0+sample_n) = 1.;
upper_bound(constr_idx) = vs*vs;
lower_bound(constr_idx) = vs*vs;
constr_idx = constr_idx + 1;

A(constr_idx, IDX_A0+sample_n) = 1.;
upper_bound(constr_idx) = as;
lower_bound(constr_idx) = as;
constr_idx = constr_idx + 1;

%% QP solver
% osqp is more fast than quadprog
use_osqp = true;
if ~use_osqp
    H = P + eye(length(P))*1e-6;
    f = q;
    A2 = [A; -A];
    b  = [upper_bound, -lower_bound];

    [x, fval, exitflag, output] = quadprog(H,f,A2,b);

    mqp.beta = x(1:sample_n);
    mqp.beta(mqp.beta < 0) = 0;       % force positive
    mqp.vel  = mqp.beta.^0.5;
    mqp.acc  = x(sample_n+1:sample_n*2);

    % calc jerk
    for i=1:sample_n-1
        vr = profile.v(i);
        vi = mqp.vel(i);
        ai = mqp.acc(i);
        aip= mqp.acc(i+1);
        ds = max(profile.s(i+1)-profile.s(i), 0.00001);

        mqp.jerk_p(i)   = (aip - ai)/ds;
        mqp.jerk_cvt(i) = mqp.jerk_p(i) * vr;
        mqp.jerk_idl(i) = mqp.jerk_p(i) * vi;
    end
        mqp.jerk_p(sample_n)   = mqp.jerk_p(sample_n-1);
        mqp.jerk_cvt(sample_n) = mqp.jerk_cvt(sample_n-1);
        mqp.jerk_idl(sample_n) = mqp.jerk_idl(sample_n-1);    
else
    m = osqp;

    settings = m.default_settings();
    settings.max_iter = 20000;
    settings.eps_abs  = 1e-04;
    settings.eps_rel  = 1e-04;

    m.setup(P, q, A, lower_bound, upper_bound, settings);
    results = m.solve();

    osqp.beta = results.x(1:sample_n);
    osqp.beta(osqp.beta < 0) = 0;       % force positive
    osqp.vel  = osqp.beta.^0.5;
    osqp.acc  = results.x(sample_n+1:sample_n*2);

    % calc jerk
    for i=1:sample_n-1
        vr = profile.v(i);
        vi = osqp.vel(i);
        ai = osqp.acc(i);
        aip= osqp.acc(i+1);
        ds = max(profile.s(i+1)-profile.s(i), 0.00001);

        osqp.jerk_p(i)   = (aip - ai)/ds;
        osqp.jerk_cvt(i) = osqp.jerk_p(i) * vr;
        osqp.jerk_idl(i) = osqp.jerk_p(i) * vi;
    end
        osqp.jerk_p(sample_n)   = osqp.jerk_p(sample_n-1);
        osqp.jerk_cvt(sample_n) = osqp.jerk_cvt(sample_n-1);
        osqp.jerk_idl(sample_n) = osqp.jerk_idl(sample_n-1);
end

%% opt result
if use_osqp
    opt.vel      = osqp.vel;
    opt.acc      = osqp.acc;
    opt.jerk_cvt = osqp.jerk_cvt;
    opt.jerk_idl = osqp.jerk_idl;
else
    opt.vel      = mqp.vel;
    opt.acc      = mqp.acc;
    opt.jerk_cvt = mqp.jerk_cvt;
    opt.jerk_idl = mqp.jerk_idl;
end

%% debug
% figure
% ppp = (P-min(P(:)))/(max(P(:))-min(P(:)));
% imshow(1-ppp);
%
% figure
% aa = (A-min(A(:)))/(max(A(:))-min(A(:)));
% imshow(1-aa);

% figure;
% plot(profilef.s, profilef.v); hold on;
% plot(profileb.s, profileb.v);
% plot(profile.s, profile.v);
% legend('forward', 'backward', 'merged');
% title('jerk smooth motion');

%% plot
subplot(3,1,1);
hold on; grid on;
plot(profile.s, profile.v);
plot(profile.s, opt.vel);
legend('upper\_bound', 'vel');
title('vel')

subplot(3,1,2);
 hold on; grid on
plot(profile.s, amax*ones(1,sample_n), '--');
plot(profile.s, opt.acc)
plot(profile.s, -dmax*ones(1,sample_n), '--');
legend('upper\_bound', 'acc', 'lower\_bound');
title('acc')

subplot(3,1,3);
hold on; grid on;
plot(profile.s, jmax*ones(1,sample_n), '--');
plot(profile.s, opt.jerk_idl);
plot(profile.s, opt.jerk_cvt);
plot(profile.s, -jmax*ones(1,sample_n), '--');
legend('upper\_bound', 'jerk ideal', 'jerk cvt', 'lower\_bound');
title('jerk');





