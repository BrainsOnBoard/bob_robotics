function make_test_data
if ~exist('../test_data','dir')
    mkdir('../test_data')
end

rng(42); % fixed seed

writefiles(ones(2),rand(2,1));

function writefiles(weights_init,im)
persistent num;
if isempty(num)
    num = 1;
else
    num = num+1;
end
fprintf('Making test set %d\n',num)

write(sprintf('weights_init%d.bin',num),weights_init);
write(sprintf('image%d.bin',num),im2uint8(im));
[weights_out,learning_rate] = infomax_train(numel(im),im,weights_init);
fprintf('Final learning rate: %g\n\n',learning_rate)
write(sprintf('weights_out%d.bin',num),weights_out);

function write(fn,data)
fid = fopen(fullfile('../test_data',fn),'w');
fwrite(fid,size(data),'int32');
fwrite(fid,data,class(data));
fclose(fid);