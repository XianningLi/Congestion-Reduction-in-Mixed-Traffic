function RBFBasis = rbfBasisMat(decayRate, x, y, xb, yb)

RBFBasis = [];
for i = 1:length(yb)
    for j = 1:length(xb)
        RBFBasis = [RBFBasis RBFBasisLongLat(decayRate, x, y, xb(j), yb(i))];
    end
end

end