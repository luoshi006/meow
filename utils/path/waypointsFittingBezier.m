function path = waypointsFittingBezier(wps, tolerance)
% path
%    |-- seg
%    |      |-- type: line / bezier / bspline
%    |      |-- ctrl pts:
%    |      |-- knots:
%    |-- size

% ref: [1] Biagiotti, L. , and  C. Melchiorri . Trajectory Planning for Automatic Machines and Robots.  2008.
%          8.11 Linear Interpolation with Polynomial Blends

% prepare
len = length(wps);
path.size = len-1 + len-2;    % line + bezier segments

% add line
idx = 1;
for i=1:len-1
    start_pt = wps(:,i);
    goal_pt  = wps(:,i+1);
    line_length = norm(goal_pt - start_pt);
    line_length_fix = line_length - tolerance;
    if line_length_fix < 0
        line_length_fix = line_length*2/3;
    end

    line_scale = line_length_fix / line_length;
    if i ~= len-1
        goal_pt_fix = start_pt + (goal_pt - start_pt)*line_scale;
    else
        goal_pt_fix = goal_pt;
    end
    if i ~= 1
        start_pt_fix = goal_pt + (start_pt - goal_pt)*line_scale;
    else
        start_pt_fix = start_pt;
    end

    path.seg(idx).type = 'line';
    path.seg(idx).ctrl_pts = [start_pt_fix' ; goal_pt_fix']';
    idx = idx + 2;
end

% add quintic bezier
idx = 2;
for i=2:len-1
    p0k = path.seg(idx-1).ctrl_pts(:,2);
    p5k  = path.seg(idx+1).ctrl_pts(:,1);
    qk = wps(:,i);

    t0k = (qk - p0k)/norm(qk - p0k);
    t5k = (p5k - qk)/norm(p5k - qk);

    % calc alpha
    a = 256 - 49 * norm(t0k + t5k)^2;
    b = 420 * (p5k-p0k)'*(t0k+t5k);
    c = -900 * norm(p5k-p0k)^2;

    delta = sqrt(b^2 - 4*a*c);
    alphak = (-b+delta)/2/a;
%     alpha2 = (-b-delta)/2/a;
    if alphak < 0
        alphak = 0.01;  % the min speed;
        disp('[warnning]: transform point speed alpha should be positive!');
    end

    p1k = p0k   + alphak/5*t0k;
    p2k = 2*p1k - p0k;
    p4k = p5k   - alphak/5*t5k;
    p3k = 2*p4k - p5k;

    path.seg(idx).type = 'quinticBezier';
    path.seg(idx).ctrl_pts = [p0k'; p1k'; p2k'; p3k'; p4k'; p5k']';
    idx = idx + 2;
end


end