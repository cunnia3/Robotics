%% ii
% symbolic joint angles, by denoted by S at the end
syms l1 l2 p0Tx p0Ty p0Tz q1S q2S q3S;
p0T = [p0Tx;p0Ty;p0Tz];

% forward kinematics
h1 = [0;0;1];
h2 = [0;1;0];
h3 = [sqrt(2)/2; 0; sqrt(2)/2];

p23 = [l1;0;0];
p3T = [l2;0;0];

R01 = angvec2r(q1S,h1);
R12 = angvec2r(q2S,h2);
R23 = angvec2r(q3S,h3);

% subproblem 3
q3 = simple(subproblem3sym(h3,p3T,p23,norm(p0T)));
pretty(q3);

% subproblem 2
alpha = p23 + R23*p3T;
[q1,q2] = subproblem2sym(h1,h2,alpha,p0T);
