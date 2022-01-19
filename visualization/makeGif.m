function nr = makeGif(fig, filename)
% Writes a matlab figure to an animated GIF.

global last_file
global pic_num
global fullname

if ~strcmp(last_file, filename)
    pic_num = 1;
    basedir = fileparts(which('meow_startup.m'));
%     fullname = [basedir, '/doc/', filename, '.gif'];
    fullname = fullfile(basedir, 'doc', [filename '.gif']);
    last_file = filename;
end
% if ~exist('fullname')
%     fullname = [basedir, '/doc/', filename, '.gif'];
% end

% F = getframe(gcf);
F = fig;
I=frame2im(F);
[I,map] = rgb2ind(I, 256);

if pic_num == 1
    imwrite(I,map,fullname, 'gif', 'Loopcount', 3, 'DelayTime', 0.03);
else
    imwrite(I,map,fullname, 'gif', 'WriteMode', 'append', 'DelayTime', 0.03);
end
pic_num = pic_num +1;

end

