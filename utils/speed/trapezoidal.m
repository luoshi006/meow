function traj_out = trapezoidal(traj_in)

    % param
    min_vel = 0.002;

    % init
    traj_out = traj_in;
    num = length(traj_in.vel_limit);
    traj_out.vel = ones(1,num)*min_vel;
    traj_out.acc = zeros(1,num);
    vel_f = ones(1,num)*min_vel;
    vel_b = ones(1,num)*min_vel;

    % forward
    for i=2:num
        a  = traj_in.acc_limit(i);
        s  = norm(traj_in.trajPts(:,i)-traj_in.trajPts(:,i-1));
        v0 = vel_f(i-1);
        vc = sqrt(2*a*s + v0^2);
        vel_f(i) = min(vc, traj_in.vel_limit(i));
    end

    % backward
    for i=num-1:-1:1
        a = traj_in.acc_limit(i);
        s = norm(traj_in.trajPts(:,i)-traj_in.trajPts(:,i+1));
        v0 = vel_b(i+1);
        vc = sqrt(2*a*s + v0^2);
        vel_b(i) = min(vc, traj_in.vel_limit(i));
    end

    % find min
    eps = 0.001;
    for i=1:num-1
        s = norm(traj_in.trajPts(:,i)-traj_in.trajPts(:,i+1));
        if (abs(vel_f(i)-vel_b(i)) < eps)
            % equal
            traj_out.vel(i) = vel_f(i);
            traj_out.acc(i) = 0;
            traj_out.dt(i)  = s/vel_f(i);
        elseif (vel_f(i) < vel_b(i))
            % acc
            traj_out.vel(i) = vel_f(i);
            traj_out.acc(i) = traj_in.acc_limit(i);
            traj_out.dt(i)  = 2*s/(vel_f(i)+vel_f(i+1));
        else
            % dcc
            traj_out.vel(i) = vel_b(i);
            traj_out.acc(i) = -traj_in.acc_limit(i);
            traj_out.dt(i)  = 2*s/(vel_b(i) + vel_b(i+1));
        end
    end
    traj_out.vel(num) = vel_b(num);
    traj_out.acc(num) = -traj_in.acc_limit(num);
    traj_out.dt(num)  = 0;
end
