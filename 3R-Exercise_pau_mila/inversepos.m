function [a1,a2,a3] = inversepos(P, O)

L1 = 0.62;
L2 = 0.57;

g1 = 0.1;
g2 = 0.2;
g3 = 0.3;

x = P(1) - g3;
y = P(2) + g2;

d = 2*L1*L2;

f = (x - g1*cos(O))^2 + (y - g1*sin(O))^2 - L1^2 - L2^2;

a2 = acos(f/d);

A = L1 + L2*cos(a2);

B = L2 *sin(a2);

E = x - g1*cos(O);

F = y - g1*sin(O);

% A*c1 - B*s1 = E
% B*c1 + A*s1 = F

X = [A, -B; B, A];

Y = [E;F];

res = X\Y;

a1 = atan(res(2)/res(1));

a3 = O - a1 -a2;

end




