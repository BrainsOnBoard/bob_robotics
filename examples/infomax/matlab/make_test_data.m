function make_test_data
if ~exist('../test_data','dir')
    mkdir('../test_data')
end

rng(42); % fixed seed

writefiles(ones(2),rand(2,1));
writefiles(ones(4),rand(4,1));
writefiles(infomax_init2(4,4),rand(4,1));

function writefiles(weights_init,im)
persistent num;
if isempty(num)
    num = 1;
else
    num = num+1;
end
fprintf('Making test set %d\n',num)

pref = sprintf('test%d_',num);

write([pref 'weights_init'],weights_init);
write([pref 'train_image'],im2uint8(im));
[weights_out,learning_rate,u,y] = infomax_train(numel(im),im,weights_init);
write([pref 'u'],u);
write([pref 'y'],y);
fprintf('Final learning rate: %g\n\n',learning_rate)
write([pref 'weights_out'],weights_out);

function write(fn,data)
fid = fopen(fullfile('../test_data',[fn '.bin']),'w');
fwrite(fid,size(data),'int32');
fwrite(fid,data,class(data));
fclose(fid);