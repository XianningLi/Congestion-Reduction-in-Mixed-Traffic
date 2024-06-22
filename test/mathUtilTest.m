function tests = mathUtilTest()
tests = functiontests(localfunctions);
end

function testRBFBasisLongLat(testCase)
    % scalar input. When the inputs x, y, xp, yp are scalars, the output
    % should be a scalar.
    decayRate = [1 1];
    actSolution = RBFBasisLongLat(decayRate, 0, 0, 0, 0);
    expSolution = 1;
    verifyEqual(testCase,actSolution,expSolution, 'Wrong output');

    % When the x and y are m-dimensional column vectors, and the xp and yp
    % are n dimensional row vectors, the output should have the size of [m, n]
    decayRate = [1 1];
    actSolution = RBFBasisLongLat(decayRate, [0;1], [1;2], [2 3], [4 5]);
    expectedDimension = [2,2];
    verifyEqual(testCase,size(actSolution), expectedDimension, 'Wrong dimension');
end