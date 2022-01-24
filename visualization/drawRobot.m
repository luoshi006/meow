function hd = drawRobot(obj)

drawFrame = true;
frm.framecolor = 'b';
frm.framelength = 0.3;
frm.framethick = 1;
frm.framestyle = '-';

%% update pose
% baselink (on the ground) to world
b2w = SE3(obj.pose.position)*SE3.rpy(obj.pose.rpy);
% config body (center of robot) to baselink
c2b = SE3(obj.body.pose.position)*SE3.rpy(obj.body.pose.rpy);
% config body to world
c2w = b2w * c2b;

%% update vertice
% calc base vertice
mesh.vert = h2e(c2w.T*e2h(obj.body.mesh.vert'))';
% calc wheels vertic
wheel_sz = length(obj.wheels);
for i=1:wheel_sz
    p = obj.wheels(i).pose.position;
    q = obj.wheels(i).pose.rpy;
    r = obj.wheels(i).rotAngle;
    whl2c(i) = SE3(p)*SE3.rpy(q)*SE3.rpy(0,0,r);
    whl2w(i) = c2w * whl2c(i);
    mesh.wheel(i).vert = h2e(whl2w(i).T * e2h(obj.wheels(i).mesh.vert'))';
end

%% draw
group = hggroup('Tag', obj.name);
hd.group = group;

if ~ishold
    hold on; axis equal; view(3);
end

opt_bdy = obj.body.mesh.color;
% plot body
hd.base = patch('vertices', mesh.vert, 'faces', obj.body.mesh.face, 'facecolor',...
                    opt_bdy.facecolor, 'facealpha', opt_bdy.facealpha, 'edgecolor',...
                    opt_bdy.edgecolor, 'edgealpha', opt_bdy.edgealpha, 'parent', group);
set(hd.base, 'Tag', [obj.name '-base']);

if (drawFrame)
    hd.baseframe = c2w.plot('color', frm.framecolor,'length',...
    0.4*min(obj.body.size(1:2)), 'thick', frm.framethick, 'style', frm.framestyle);
    set(hd.baseframe,'parent',group);
    set(hd.baseframe,'Tag', [obj.name '-frame']);

end

% plot wheels
opt_whl = obj.wheels(i).mesh.color;
for i=1:wheel_sz
    whl_vert = mesh.wheel(i).vert;
    % plot cylinder
    hd.whl(i).cyl = patch('vertices', whl_vert, 'faces', obj.wheels(i).mesh.face, 'facecolor',...
                    opt_whl.facecolor, 'facealpha', opt_whl.facealpha, 'edgecolor',...
                    opt_whl.edgecolor, 'edgealpha', opt_whl.edgealpha, 'parent', group);
    set(hd.whl(i).cyl, 'Tag', [obj.name '-whl' num2str(i) '-cyl']);
    % plot lower surface
    vert = reshape(whl_vert',6,21)';
    hd.whl(i).surl = patch('vertices',vert(:,1:3), 'faces', 1:21, 'facecolor',...
        opt_whl.facecolor, 'facealpha', opt_whl.facealpha, 'edgecolor',...
        opt_whl.edgecolor, 'edgealpha', opt_whl.edgealpha, 'parent', group);
    set(hd.whl(i).surl,'Tag', [obj.name '-whl' num2str(i) '-surl']);

    % plot upper surface
    hd.whl(i).suru = patch('vertices',vert(:,4:6), 'faces', 1:21, 'facecolor',...
        opt_whl.facecolor, 'facealpha', opt_whl.facealpha, 'edgecolor',...
        opt_whl.edgecolor, 'edgealpha', opt_whl.edgealpha, 'parent', group);
    set(hd.whl(i).suru,'Tag', [obj.name '-whl' num2str(i) '-suru']);

    if (drawFrame)
        hd.whl(i).frame = whl2w(i).plot('color', frm.framecolor,'length',...
                                        obj.wheels(i).size(1)/2, 'thick', frm.framethick, 'style', frm.framestyle);
        set(hd.whl(i).frame,'parent',group);
        set(hd.whl(i).frame,'Tag', [obj.name '-whl' num2str(i) '-frame']);
    end
end

end