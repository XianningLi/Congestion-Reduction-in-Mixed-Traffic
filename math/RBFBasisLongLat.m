function basisFun = RBFBasisLongLat(decayRate, x, y, xb, yb)
    xbb = repmat(xb,numel(x),1); % x locations of basis functions
    ybb = repmat(yb,numel(y),1); % y locations of basis functions
    basisFun = exp(-decayRate(1)*(x-xbb).^2-decayRate(2)*(y-ybb).^2 ); % basis function matrix
end