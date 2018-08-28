function make_test_data
if ~exist('../test_data','dir')
    mkdir('../test_data')
end

rng(42); % fixed seed
weights_init = ones(2);
write('weights_init.bin',weights_init);

im = uint8(round(rand(2,1) * 255));
imwrite(im,'../test_data/image.png');

[weights_out,learning_rate] = infomax_train(numel(im),im,weights_init);
fprintf('Final learning rate: %g\n',learning_rate)
write('weights_out.bin',weights_out);

function write(fn,data)
fid = fopen(fullfile('../test_data',fn),'w');
fwrite(fid,size(data),'int32');
fwrite(fid,data,'double');
fclose(fid);