clc; clear; close all;

k = 4;      % order
d = 3;      % degree
n = 6;      % ctrl_points_size = n + 1
m = n + k;  % knots_size = m + 1

knots = augknt(linspace(0,1,n-k+1+2), k);   % clamped uniform
tau = linspace(0,1,100);

% draw basis function
basis = spcol(knots, k, tau);
plot(tau, basis);
hold on;
title('4 Order Clamped B-spline Basis Function');
xlabel('press any key to continue!')

kx = repmat(knots, 2, 1);
ky  = repmat(.1*[1;-1],1, m+1);
plot(kx, ky, 'k');

pause;
for i=1:m-k+1
    tmp_f = plot(tau, basis(:,i),'LineWidth',3);
    title(['4 Order B-spline Basis Function ', num2str(i)]);
    pause;
    delete(tmp_f);
end

% The Local Partition of Unity
for i=1:n-k+2
    ind = find(tau>=knots(k+i-1)&tau<=knots(k+i));
    tmp_f = plot(tau(ind)', basis(ind,i:k+i-1)', 'LineWidth',3);
    tmp_f2= plot(tau(ind)', ones(size(tau(ind)))', 'k', 'LineWidth',3);
    tmp_f3= plot([knots(k+i-1) knots(k+i-1) knots(k+i) knots(k+i)],...
                 [0 1 1 0],'--k');
    title(['4 Order B-spline Sum of Basis Function ', num2str(i), '-', num2str(k+i-1)]);
    pause;
    delete(tmp_f); delete(tmp_f2); delete(tmp_f3);
end
hold off;

%% draw sin curve
ctrl_points = [0 0.82 1.40 0 -1.40 -0.82 0];
sp_sin = spmak(knots, ctrl_points);
val_sin = basis*ctrl_points';
plot(tau, val_sin);
hold on;
xlabel('press any key to continue!')

node_ts = aveknt(knots,4);
plot(node_ts,ctrl_points,':ok');
plot(node_ts,zeros(size(node_ts)),'*')
plot(kx, ky, 'k');

% The Convex Hull Property and the Control Polygon
for i=1:n-k+2
    ind = find(tau>=knots(k+i-1)&tau<=knots(k+i));
    tmp_f = fill(node_ts(i:k+i-1),ctrl_points(i:k+i-1),'y','EdgeColor','r', 'FaceAlpha',0.7, 'EdgeAlpha',0.2);
    tmp_f2= plot(tau(ind), val_sin(ind),'r', 'LineWidth', 3);
    tmp_f3= plot(node_ts(i:k+i-1),ctrl_points(i:k+i-1),'sg', 'MarkerFaceColor','g');
    pause;
    delete(tmp_f); delete(tmp_f2); delete(tmp_f3);
end
