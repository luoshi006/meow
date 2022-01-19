function path = createRoad(varargin)
% wps: waypoints,    e.g. [x0, x1...xn;
%                         y0, y1...yn]

% option
defaultRoadWidth = 1;
defaultRoadTileLen = .3;
defaultWps = [0 2.5 2.5 2.5 2.5;...
              0 0   2.5 5.0 6.0];

inp = inputParser;
validNumChk  = @(x) isnumeric(x) && isscalar(x) && (x > 0);
validWpsChk = @(x) isnumeric(x) && ismatrix(x) && (size(x,1) == 2);
addOptional(inp, 'wps'          , defaultWps            , validWpsChk);
addOptional(inp, 'roadWidth'    , defaultRoadWidth      , validNumChk);
addOptional(inp, 'roadTileLen'  , defaultRoadTileLen    , validNumChk);
parse(inp, varargin{:});

wpx = inp.Results.wps(1,:);
wpy = inp.Results.wps(2,:);

path.roadWidth = inp.Results.roadWidth;
path.tileLength = inp.Results.roadTileLen;

[px,py] = driving.scenario.clothoid(wpx,wpy);
xyDist = cumsum(sqrt(sum(diff([px(1) py(1); px' py']).^2,2)));

path.wayPoints = [wpx',wpy'];

% return total distance traveled along the way
path.splinewaypoints = [[px(1) py(1)]; [px' py']];
% cumdist = cumsum(sqrt(sum(diff(splinewaypoints,1,1).^2,2)));
cumdist = xyDist;

% divide total length into tiles of approx length tileLength
path.ntiles = ceil(cumdist(end)/path.tileLength);
path.splinedist = linspace(cumdist(1),cumdist(end),1+path.ntiles);

path.centerPoints = interp1(cumdist, path.splinewaypoints(2:end,:), path.splinedist, 'spline');
centerPointsEps = interp1(cumdist, path.splinewaypoints(2:end,:), path.splinedist+1e-3, 'spline');

cpNormal = [path.centerPoints(:,2)-centerPointsEps(:,2), centerPointsEps(:,1)-path.centerPoints(:,1)];
cpNormal = cpNormal./sqrt(sum(cpNormal.^2,2));

lfts = path.centerPoints+cpNormal.*path.roadWidth;
rgts = path.centerPoints-cpNormal.*path.roadWidth;

for i=1:size(lfts,1)-1
    path.tiles(i).Vertices(:,:) = [lfts(i,:); rgts(i,:); rgts(i+1,:); lfts(i+1,:)];
end

end