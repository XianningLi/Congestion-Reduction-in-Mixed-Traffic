function timeTaken = getTimeTaken(v0, t0, v, a ,s)

timeTaken = zeros(1, length(s));

for i = 1:length(s)

    if a(i) == 0
        timeTaken(i) = s(i)/v0;
        t0 = timeTaken(i);
        v0 = v(i);
    else
        timeTaken(i) = (v(i)-v0)/a(i);
        t0 = timeTaken(i);
        v0 = v(i);
    end

end

end