function accelerations = getAccelerations(v0, v, s)

n = length(v);
indx = round(length(s)/n);
accelerations = zeros(1, length(s));
count1 = 1;
count2 = indx;

for i = 1:n-1

    accelerations = (v^2-v0^2)./(2*s);

end

v = (sqrt(v0^2 + 2*a(i+1)*s(count1:end)));
accelerations(count1:end) = v;


end