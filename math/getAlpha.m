function [alpha, RBFBasis] = getAlpha(params)
X = params.road.X;
Y = params.road.Y;
Z = params.road.Z;
xxb = params.road.xxb;
yyb = params.road.yyb;
decayRate = params.road.decayRate;
%% solve by least squares
% RBFBasis = rbfBasisMat(decayRate, X(:), Y(:), xb, yb);
RBFBasis = RBFBasisLongLat(decayRate, X(:), Y(:), xxb(:)', yyb(:)'); % get the basis matrix
alpha = RBFBasis\Z(:);                                       % compute weigths of basis by least squares
end





