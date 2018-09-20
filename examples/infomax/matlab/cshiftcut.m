function crim = cshiftcut(im,hsz,shft)
    cim = circshift(im,[0 shft]);
    xoff = (size(im,2)-hsz)/2;
    crim = cim(:,xoff+(1:hsz));
end