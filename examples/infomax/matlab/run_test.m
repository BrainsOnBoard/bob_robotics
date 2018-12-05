function [head,minval,ridf]=run_test

imsize = [25 90];

disp('Training network')
tic
W = infomax_init2(prod(imsize), prod(imsize));
for i = 0:10:810
    im = loadim(i,imsize);
    W = infomax_train(prod(imsize),im(:),W);
end
toc

testim = loadim(10,imsize);
[head,minval,ridf] = infomax_gethead(testim,imsize,W,2*pi,imsize(2));
head * 180/pi
minval

figure(1);clf
plot(ridf)

function im=loadim(num,imsize)
impath = sprintf('../../../tools/ant_world_db_creator/ant1_route1/image_%05d.png',num);
im = imresize(rgb2gray(imread(impath)),imsize);