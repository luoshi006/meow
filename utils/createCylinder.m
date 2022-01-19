function obj = createCylinder(siz, color)
%createCylinder - Description
%
% Syntax: obj = createCylinder(siz, color)
%
% Long description

obj.color = createColorScheme(color);

l = siz(1)/2;
w = siz(2)/2;
h = siz(3);
[X,Y,Z] = cylinder(1,20);
[TRI,V]= surf2patch(l*X,w*Y,Z);

obj.vert = [V(:,1:2),V(:,3)*h-h/2];
obj.face  = TRI;

end
