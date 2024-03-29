function traj = path2traj(path, limit)

    limit_fast_speed = limit.fast_speed;
    limit_slow_speed = limit.slow_speed;
    limit_fast_acc   = limit.fast_acc;
    limit_slow_acc   = limit.slow_acc;

    traj.trajPts   = [];
    traj.curvature = [];
    traj.tangent   = [];
    traj.vel_limit = [];
    traj.acc_limit = [];

    %% sample trajectory points
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
            traj.vel_limit = [traj.vel_limit, ones(1,num)*limit_fast_speed];
            traj.acc_limit = [traj.acc_limit, ones(1,num)*limit_fast_acc];
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
            % sign    = zeros(1,num);
            for j=1:num
                tangent(:,j) = sample_step*dpts(j,:)';
                % calc curvature
                % k(u) = | f'(u) × f''(u) | / | f'(u) |^3
                % https://pomax.github.io/bezierinfo/#curvature
                % kappa(j) = norm(cross([dpts(j,:),0], [ddpts(j,:),0]))/(norm(dpts(j,:))^3);
                kappa(j) = (dpts(j,1)*ddpts(j,2)-ddpts(j,1)*dpts(j,2))/(dpts(j,1)^2+dpts(j,2)^2)^1.5;
                % sign(j)  = (dpts(j,1)*ddpts(j,2)-ddpts(j,1)*dpts(j,2));
            end

            traj.trajPts   = [traj.trajPts'; pts]';
            traj.curvature = [traj.curvature, kappa];
            traj.tangent   = [traj.tangent'; tangent']';
            traj.vel_limit = [traj.vel_limit, ones(1,num)*limit_slow_speed];
            traj.acc_limit = [traj.acc_limit, ones(1,num)*limit_slow_acc];

        end
    end

    traj = trapezoidal(traj);

end