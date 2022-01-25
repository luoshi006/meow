function obj = updateWheelKinetic(obj, v, w, dt)
% from vehicle v/w to wheels v/w
% v = [vx, vy, vz]
% w = [wx, wy, wz]

%[1] Kelly, A. . (2010). A vector algebra formulation of kinematics of wheeled mobile robots.

%% update wheel state
% wv - wheel velocity
for i=1:length(obj.wheels)
    r = obj.wheels(i).pose.position;
    r(3) = 0;
    wv(:,i) = v + cross(w, r);

    % calc wheel RPM
    spd = wv(1,i);
    omg = spd/obj.wheels(i).radius;
    obj.wheels(i).rotAngle = wrapToPi(obj.wheels(i).rotAngle + omg*dt);
    obj.wheels(i).steerAngle = atan2(wv(2,i), wv(1,i));
end

%% update pose
bds = v*dt;      % body frame;
bdtheta = w*dt;  % body frame;

% wds = [0,0,0]';      % world frame;
% wdtheta = [0,0,0]';
eps = 0.0001;

w_n = w/norm(w);
dtheta = norm(bdtheta);
q_body_icr = [cos(dtheta/2), w_n*sin(dtheta/2)];
q_icr_w = angle2quat(obj.pose.rpy(1),obj.pose.rpy(2),obj.pose.rpy(3), 'xyz');

icr_r_v = cross(v,w);
% update position by screw motion theroy
if norm(icr_r_v) < eps
    dcm = angle2dcm(obj.pose.rpy(1),obj.pose.rpy(2),obj.pose.rpy(3), 'xyz');
    wds = dcm * bds';
else
    icr_rn = norm(v)/norm(w);    %scalar
    icr_r = icr_r_v/norm(icr_r_v) * icr_rn;
    dcm_icr_body = quat2dcm(quatinv(q_body_icr));
    dcm_w_icr = quat2dcm(quatinv(q_icr_w));

    wds = dcm_w_icr*(dcm_icr_body*icr_r' - icr_r');
end

% update quaternion by 1-order Taylor
halfwt = w*dt/2;
q1 = [         1  -halfwt(1) -halfwt(2) -halfwt(3);...
        halfwt(1)         1   halfwt(3) -halfwt(2);...
        halfwt(2) -halfwt(3)         1   halfwt(1);...
        halfwt(3)  halfwt(2) -halfwt(1)         1 ;] * q_icr_w';
[r,p,y] = quat2angle(q1', 'xyz');
q0 = obj.pose.rpy;
wdtheta = [r-q0(1), p-q0(2), y-q0(3)]';

obj.pose.rpy = obj.pose.rpy + wdtheta';
obj.pose.position = obj.pose.position + wds';

end