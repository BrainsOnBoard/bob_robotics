function [head,minval,ridf]=infomax_gethead(im,imsz,weights,angleunit,nth,fov)
if nargin < 6
    fov = 360;
end
if nargin < 5 || isempty(nth)
    nth = 360;
end
if nargin < 4 || isempty(angleunit)
    angleunit = 2*pi;
end

if isempty(imsz)
    imsz = size(im);
end

im = im2double(im);

imvars = nth/(imsz(2)*360/fov);
ridf = NaN(nth,1); % sort of not really an ridf
for i = 0:imvars-1
    cim = imresize(circshift(im,[0 i]),imsz,'bilinear');
    
    for j = 1:imsz(2)
        crim = cshiftcut(cim,imsz(2),j-1);
        ridf(imvars*(j-1)+1+i) = sum(abs(weights*crim(:)));
    end
end

[minval,minI] = min(ridf);

head = angleunit*(minI-1)/nth;