function len = lengthBezier(ctrl_pts)

    n = length(ctrl_pts) - 1;

    % calc sample number
    ployLen = 0;
    for i=1:n
        ployLen = ployLen + norm(ctrl_pts(:,i) - ctrl_pts(:,i+1));
    end
    sample_num = ployLen*100;  % 1cm each point
    sample_num = min(sample_num, 1E5);  % max = 1km

    % sample
    t = linspace(0,1, sample_num);
    B = bernsteinMatrix(n,t);
    P = ctrl_pts';
    sample_pts = B*P;

    % calc length
    len = 0;
    for i=1:sample_num-1
        len = len + norm(sample_pts(i,:)-sample_pts(i+1,:));
    end

end