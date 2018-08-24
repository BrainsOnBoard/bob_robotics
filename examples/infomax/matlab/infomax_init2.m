function w = infomax_init2(V,H)
% function w = infomax_init(N)
%
% Initialises the weights of a familiarity discrimination network.
%
% Input:
%  N - size of the network
% Output:
%  w - the matrix of weights: each row corresponds to the weights of a single novelty neuron

w = randn(V,H);

% weight normalization to ensure mean = 0 and std = 1
w = w - repmat (mean(w,2), 1, H);
w = w ./ repmat (std(w,1,2), 1, H);
