function L = ImageProcessing(image, threshold)
% splits up an image into good and bad region
% INPUTS:
%   image - an RGB image, made by doing imread()
%   threshold - threshold to find edge vs paper (default is set to 0.3)
% OUTPUTS:
%   L - sectioned image that is 1 for bad area, and 0 for good area

if nargin == 1
    threshold = 0.3;
end
G = rgb2gray(image); % converted to gray scale
BW = imbinarize(G,threshold); % converted to binary
[~,L] = bwboundaries(BW,'noholes'); % converted to sectioned image
end