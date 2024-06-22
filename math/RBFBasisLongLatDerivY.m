function basisFun = RBFBasisLongLatDerivY(decayRate, x, y, xb, yb)
    xbb = repmat(xb,numel(x),1); % x locations of basis functions
    ybb = repmat(yb,numel(y),1); % y locations of basis functions
    basisFun = (y-ybb).*exp(-decayRate(1)*(x-xbb).^2-decayRate(2)*(y-ybb).^2 ); % derivative w.r.t y
end