function polyParams = getPolyParams(p, params)
% this function converts the parameterization parameters (pi's) to the polynomial parameters [a, b, c, d], it is assumed that the
% curvature at the initial and final pose are zero. Thus, p1 = 0, p4 = 0.

if params.flag == "XY"
    p = [zeros(1,numel(p(1,:)))
        p(1,:)
        p(2,:)
        zeros(1,numel(p(1,:)))
        p(3,:)];
    a = p(1,:);
    b = -( 11*p(1,:)-18*p(2,:)+9*p(3,:)-2*p(4,:) )./(2*p(5,:));
    c = ( 9*(2*p(1,:)-5*p(2,:)+4*p(3,:)-p(4,:)) )./(2*p(5,:).^2);
    d = -( 9*(p(1,:)-3*p(2,:)+3*p(3,:)-p(4,:)) )./(2*p(5,:).^3);
    polyParams = [a; b; c; d];

else

    p = [params.xyPlan.s0*ones(1,numel(p(1,:)))
        p(1,:)
        p(2,:)
        params.xyPlan.s1*ones(1,numel(p(1,:)))
        p(3,:)];
    a = p(1,:);
    b = -( 11*p(1,:)-18*p(2,:)+9*p(3,:)-2*p(4,:) )./(2*p(5,:));
    c = ( 9*(2*p(1,:)-5*p(2,:)+4*p(3,:)-p(4,:)) )./(2*p(5,:).^2);
    d = -( 9*(p(1,:)-3*p(2,:)+3*p(3,:)-p(4,:)) )./(2*p(5,:).^3);
    polyParams = [a; b; c; d];

end

end