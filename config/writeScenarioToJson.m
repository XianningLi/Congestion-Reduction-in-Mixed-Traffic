function writeScenarioToJson(s, filename)
    jData = jsonencode(s, 'PrettyPrint', true);
    fid = fopen(filename,'w');
    fprintf(fid,'%s',jData);
    fclose(fid);
end