syms q1 q2 q3 l1 l2
ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];
h1 = ez; h2=ey; h3=ey;
R01 = rot(h1,q1); R02 = R01*rot(h2,q2); R03 = R02*rot(h3,q3);
p23 = [l1;0;0]; p3T = [l2;0;0];
p0T = R02*p23 + R03*p3T;

J = [diff(p0T,q1) diff(p0T,q2) diff(p0T,q3)];
simple(det(J))