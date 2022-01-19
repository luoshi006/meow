function hd = drawRoad(path)


%% draw
group = hggroup('Tag', 'road');
hd.group = group;

if ~ishold
    hold on; axis equal; view(2);
end
RoadTileFaceColor = [.8 .8 .8];
RoadTileEdgeColor = [.7 .7 .7];
RoadBorderColor = [.7 .5 .3];
RoadCenterlineColor = [1 1 1];

% hd.lfts = plot(lfts(:,1), lfts(:,2), '.','Color', RoadBorderColor, 'parent', group);
% hd.rgts = plot(rgts(:,1), rgts(:,2), '.','Color', RoadBorderColor, 'parent', group);
% set(hd.lfts, 'Tag', 'road-left');
% set(hd.rgts, 'Tag', 'road-right');

% draw tile
for i=1:length(path.tiles)
    rt = path.tiles(i);
    hd.tiles(i) = patch(rt.Vertices(:,1), rt.Vertices(:,2),RoadTileFaceColor,'EdgeColor',RoadTileEdgeColor, 'parent', group);
    set(hd.tiles(i), 'Tag', ['road-tile-' num2str(i)]);
end

hd.wps = plot(path.wayPoints(:,1), path.wayPoints(:,2), 'h', 'parent', group);
set(hd.wps, 'Tag', 'road-wps');
hd.spls = plot(path.splinewaypoints(:,1),path.splinewaypoints(:,2),'Color',RoadCenterlineColor, 'parent', group);
set(hd.spls, 'Tag', 'road-spline');
axis equal

end
