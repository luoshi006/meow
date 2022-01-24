function obj = updateWheelInvKinetic(obj, v, w, dt)
% from vehicle v/w to wheels v/w
% v = [vx, vy, vz]
% w = [wx, wy, wz]

%% update wheel state
% wv - wheel velocity
for i=1:length(obj.wheels)
    r = obj.wheels(i).pose.position;
    r(3) = 0;
    wv(:,i) = v + cross(w, r);

    % calc wheel RPM
    spd = norm(wv(:,i));
    omg = spd/obj.wheels(i).radius;
    obj.wheels(i).rotAngle = wrapToPi(obj.wheels(i).rotAngle + omg*dt);
    obj.wheels(i).steerAngle = atan2(wv(2,i), wv(1,i));
end

%% update pose
% XXX: only SE2 support
bds = v*dt;      % body frame;
bdtheta = w*dt;  % body frame;

wds = [0,0,0];      % world frame;
wdtheta = 0;
if norm(w) < 0.0001
    % w=0
    dcm = angle2dcm(obj.pose.rpy(1),obj.pose.rpy(2),obj.pose.rpy(3), 'xyz');
    wds = dcm * bds';
    wdtheta = 0;
else
    theta = obj.pose.rpy(3);
    dtheta = norm(bdtheta);
    icr_r = norm(v)/norm(w);    %scalar
    dx = -icr_r*sin(theta) + icr_r*sin(theta+dtheta);
    dy =  icr_r*cos(theta) - icr_r*cos(theta+dtheta);
    wds = [dx, dy, 0]';
    wdtheta = dtheta;
end

obj.pose.rpy(3) = obj.pose.rpy(3) + wdtheta;
obj.pose.position = obj.pose.position + wds';

end