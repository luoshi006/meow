function opt = createColorScheme(style)
    % blue
    % green
    % red
    % cyan
    % magenta
    % yellow
    % black

switch style
case {'blue', 'b'}
    opt.facecolor = [0, 0, 1];
    opt.facealpha = 0.4;
    opt.edgecolor = [0., 0., 0.4];
    opt.edgealpha = 0.8;
case {'green', 'g'}
    opt.facecolor = [0, 1, 0];
    opt.facealpha = 0.4;
    opt.edgecolor = [0., 0.4, 0];
    opt.edgealpha = 0.8;
case {'red', 'r'}
    opt.facecolor = [1, 0, 0];
    opt.facealpha = 0.4;
    opt.edgecolor = [0.4, 0, 0];
    opt.edgealpha = 0.8;
case {'cyan', 'c'}
    opt.facecolor = [0, 1, 1];
    opt.facealpha = 0.4;
    opt.edgecolor = [0., 0.4, 0.4];
    opt.edgealpha = 0.8;
case {'magenta', 'm'}
    opt.facecolor = [1, 0, 1];
    opt.facealpha = 0.4;
    opt.edgecolor = [0.4, 0., 0.4];
    opt.edgealpha = 0.8;
case {'yellow', 'y'}
    opt.facecolor = [1, 1, .0];
    opt.facealpha = 0.4;
    opt.edgecolor = [0.4, 0.4, 0.];
    opt.edgealpha = 0.8;
case {'black', 'k'}
    opt.facecolor = [0, 0, 0];
    opt.facealpha = 0.4;
    opt.edgecolor = [0.4, 0.4, 0.4];
    opt.edgealpha = 0.8;
otherwise
    % gray
    opt.facecolor = [.8, .8, .8];
    opt.facealpha = 0.4;
    opt.edgecolor = [0.4, 0.4, 0.4];
    opt.edgealpha = 0.8;

end

end