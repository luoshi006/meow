function dCtrlPts = derivateBezier(ctrlPts)
% [x0, x1, ... , xn;
%  y0, y1, ... , yn]
% ref: https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/Bezier/bezier-der.html

n = length(ctrlPts) - 1;
dCtrlPts = zeros(2, n);

for i=1:n
    dCtrlPts(:,i) = n * (ctrlPts(:,i+1) - ctrlPts(:,i));
end

end