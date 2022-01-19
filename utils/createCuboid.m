function obj = createCuboid(siz, color)
%createCuboid - Description
%
% Syntax: obj = createCuboid(siz, color)
%
% Long description

obj.color = createColorScheme(color);

templateVerts = [0,0,0;
                 0,1,0;
                 1,1,0;
                 1,0,0;
                 0,0,1;
                 0,1,1;
                 1,1,1;
                 1,0,1];
templateFaces = [1,2,3,4;
                 5,6,7,8;
                 1,2,6,5;
                 3,4,8,7;
                 1,4,8,5;
                 2,3,7,6];

edge = siz';
obj.face = templateFaces;
obj.vert = [templateVerts(:,1)*edge(1)-edge(1)/2,...
             templateVerts(:,2)*edge(2)-edge(2)/2,...
             templateVerts(:,3)*edge(3)-edge(3)/2];

end