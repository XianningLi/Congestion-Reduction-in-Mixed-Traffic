function plotScenario(filename)
jData = readScenarioFromJson(filename);
mesh(jData.X,jData.Y,jData.Z)
daspect([1 1 1]) % adjust the display ratio of each axis
end