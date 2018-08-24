function [W,mu]=infomax_train(nhid,D,W)
% function [W]=infomax_train(nhid,D,W)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% inputs
% nhid - size of the hidden layer of the network noramlly set to number of
%        pixels in input image (i.e. 17x90 = 1530) but can be less
% sig - SD of noise to add to chosen direction of movement (try 0.1) for
%       starters. sig = 0 gives noise free performance
% MIKESDATA - normally contains all the data specifying the 3D model of the
%       environment needed by get View
% runName0 - contains the training data
% filename - name of file used to save figure
% W - optional input containing pretrained weights of the Infomax
% network,server to re-run a same memory
%
% outputs
% W -   trained network weights
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% load the training data
% load(runName0); % x y th D runName
% mat file containing the following data
% x - [1 x number of training points] vector
% y - [1 x number of training points] vector
% th - [1 x number of training points] vector
% D - [num pixels in image x number of training points]

% if only 2 inputs initialise a new network
if nargin<3 || isempty(W)
    ninput = size(D,1); % number of pixels determines the size of the input
    W = infomax_init2(ninput,nhid);
else
    W=W';
end

D = im2double(D); % make sure we have the correct input type - uint8 images won't work

% learning rate
mu=0.001;
% this while loop just reduces the learning rate if the weights blow up
while true
    try
        W = infomax_learn2(W',D,[],mu);
        break;
    catch ex
        if strcmp(ex.identifier,'LearningRule:WeightBlowUpError')
            mu = mu*0.99;
            warning('weights blew up; reducing learning rate to %g',mu)
        else
            rethrow(ex);
        end
    end
end
clear D


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INFOMAX FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
w = w ./ repmat (std(w,1,2), 1, H);

function weights = infomax_learn2(weights,patts,vars,lrate)
% Infomax "extended algorithm"
if nargin < 4
    str = 'A learning rate is required';
    id = 'LearningRule:noLrateError';
    fd_error(str, id);
end
[N, P] = size(patts);
[H,V] = size(weights);
% fd_disp('Presenting familiar patterns ...','filename');
for i=1:P
    u=weights*patts(:,i);
    y=tanh(u);
    weights = weights + lrate/N * (eye(H)-(y+u)*u') * weights;
    if any(any(isnan(weights)))
        str='Weights blew up';
        id='LearningRule:WeightBlowUpError';
%         fd_error(str, id);
        error(id, str);
    end
end