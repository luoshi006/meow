function robot = createRobot(varargin)
% rbt1 = createRobot('diff', 'cuboid', 'red');

%- Robot object
%   - Body
%       - size :     length/width/height
%       - pose:     position/rpy
%       - type:     cuboid/cylinder
%       - mesh:
%           - vertex/face
%           - color:    facecolor/facealpha/edgecolor/edgealpha
%           - drawFrame: true/false
%   - Wheels
%       - size :     width/radius
%       - pose:     position/rpy
%       - type:     diff/omni/etc.
%       - mesh:
%           - vertex/face
%           - color: facecolor/facealpha/edgecolor/edgealpha
%           - drawFrame: true/false
%   - Sensors
%       - to be done...

% option
defaultType  = 'diff';
defaultShape = 'cuboid';
defaultColor = 'gray';
defaultSize  = [1, 0.5, 0.4];

inp = inputParser;
validNumChk  = @(x) isnumeric(x) && isscalar(x) && (x > 0);
validSizeChk = @(x) isnumeric(x) && isvector(x) && (length(x) == 3);
validCharChk = @(x) isa(x, 'char');
addOptional(inp, 'type' , defaultType , validCharChk);
addOptional(inp, 'shape', defaultShape, validCharChk);
addOptional(inp, 'color', defaultColor, validCharChk);
addOptional(inp, 'size' , defaultSize , validSizeChk);
parse(inp, varargin{:});

% default wheel param
wheelWidth  = 0.02;
wheelRadius = 0.1;
wheelVisEps = 0.02;
switch inp.Results.color
case {'yellow', 'y'}
    wheelColor = 'red';
otherwise
    wheelColor = 'yellow';
end

%% create robot body
hgt = inp.Results.size(3)/2 + wheelVisEps;
robot.body.pose.position = [0,0,hgt];
robot.body.pose.rpy      = [0,0,0];
robot.body.size          = inp.Results.size;
switch inp.Results.shape
case 'cuboid'
    robot.body.mesh = createCuboid(inp.Results.size, inp.Results.color);
% case 'cylinder'
%     robot.body.mesh = createCylinder(inp.Results.size, inp.Results.color);
otherwise
    disp('unsupport robot body type');
end

%% create wheel
switch inp.Results.type
case 'diff'
    w_base_half = (robot.body.size(2) - wheelWidth)/2;
    wheelSize = [2*(wheelRadius+wheelVisEps), 2*(wheelRadius+wheelVisEps), wheelWidth+wheelVisEps];
    for i=1:2
        robot.wheels(i).mesh = createCylinder(wheelSize, wheelColor);
        robot.wheels(i).mesh.color.facealpha = 0.2;
        whl_hgt = wheelRadius + wheelVisEps - hgt;
        robot.wheels(i).pose.position = [0, sign(i-1.5)*w_base_half, whl_hgt];
        robot.wheels(i).pose.rpy      = [-pi/2, 0, 0];
        robot.wheels(i).size = wheelSize;
        robot.wheels(i).radius = wheelRadius;
        robot.wheels(i).rotAngle = 0;
        robot.wheels(i).steerAngle = 0;
        robot.wheels(i).type = 'Standard';  % Steering
    end
otherwise
    disp('unsupport robot type');
end

%% kinetic
robot.kinematic.vel_max = 4;
robot.kinematic.acc_max = 4;
robot.kinematic.omega_max = 4;

%% default pose
robot.pose.position = [0,0,0];
robot.pose.rpy      = [0,0,0];

%% default param
robot.name = 'robot';

end