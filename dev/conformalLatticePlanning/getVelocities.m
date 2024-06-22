function velocities = getVelocities(v0, a, s)

n = length(a);
indx = round(length(s)/n);
velocities = zeros(1, length(s));
count1 = 1;
count2 = indx;
for i = 1:n-1

    v = sqrt(v0^2 + 2*a(i)*s(count1:count2));
    if isreal(v)
        velocities(count1:count2) = v;
        v0 = velocities(count2);
        count1 = count2 + 1;
        count2 = count2 + indx;
    else
        velocities = nan;
        warning("the acceleration leads to undefined velocity. Either decrease the acceleration or start with a higher velocity.")
        break;
    end

end

v = (sqrt(v0^2 + 2*a(i+1)*s(count1:end)));
if isreal(v)
    velocities(count1:end) = v;
else
    velocities = nan;
    warning("the acceleration leads to undefined velocity. Either decrease the acceleration or start with a higher velocity.")
end

end