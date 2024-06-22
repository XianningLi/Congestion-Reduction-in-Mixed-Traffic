function data = readScenarioFromJson(filename)
    fid = fopen(filename,'r');
    jData = fscanf(fid, '%s');
    fclose(fid);
    data = jsondecode(jData);
end