function [velSpiral, accelSpiral] = getVelAccelSpriral(paramsVelAccel)

polyParams = getVelAccelPolyParams(paramsVelAccel);

s = paramsVelAccel.s;

velSpiral = polyParams(1,:) + polyParams(2,:).*s + polyParams(3,:).*s.^2 + polyParams(4,:).*s.^3;

accelSpiral = polyParams(2,:) + 2*polyParams(3,:).*s + 3*polyParams(4,:).*s.^2;

end

function VelAccelPolyParams = getVelAccelPolyParams(paramsVelAccel)

s0 = paramsVelAccel.s0;
s1 = paramsVelAccel.s1;
v0 = paramsVelAccel.v0;
v1 = paramsVelAccel.v1;
a0 = paramsVelAccel.a0;
a1 = paramsVelAccel.a1;

p0 = -(a1*s0^3*s1 + a0*s0^2*s1^2 - a1*s0^2*s1^2 - a0*s0*s1^3 - 3*s0*s1^2*v0 + s1^3*v0 - s0^3*v1 + 3*s0^2*s1*v1) / (s0 - s1)^3;

p1 = -(-a1*s0^3 - 2*a0*s0^2*s1 - a1*s0^2*s1 + a0*s0*s1^2 + 2*a1*s0*s1^2 + a0*s1^3 + 6*s0*s1*v0 - 6*s0*s1*v1) / (s0 - s1)^3;

p2 = -(a0*s0^2 + 2*a1*s0^2 + a0*s0*s1 - a1*s0*s1 - 2*a0*s1^2 - a1*s1^2 - 3*s0*v0 - 3*s1*v0 + 3*s0*v1 + 3*s1*v1) / (s0 - s1)^3;

p3 = -(-a0*s0 - a1*s0 + a0*s1 + a1*s1 + 2*v0 - 2*v1) / (s0 - s1)^3;

VelAccelPolyParams = [p0;p1;p2;p3];

end