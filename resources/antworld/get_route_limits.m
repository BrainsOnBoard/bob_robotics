function [mincoords, maxcoords]=get_route_limits(fname)
if ~nargin
    fnames = {dir('ant*.bin').name};
    mincoords=[inf inf inf];
    maxcoords=[-inf -inf -inf];
    for i = 1:numel(fnames)
        [cmin,cmax]=get_route_limits(fnames{i});
        mincoords=min(cmin,mincoords);
        maxcoords=max(cmax,maxcoords);
    end
    disp('Overall minimum:')
    disp(mincoords)
    disp('Overall maximum:')
    disp(maxcoords)
    return
end

fprintf('Loading %s...\n',fname)
f = fopen(fname,'r');
coords = fread(f,inf,'double');
coords = reshape(coords,[numel(coords)/3 3]) / 100; % vals in cm
disp('Minimum:')
mincoords=min(coords);
disp(mincoords);
disp('Maximum:')
maxcoords=max(coords);
disp(maxcoords);
disp('.')
