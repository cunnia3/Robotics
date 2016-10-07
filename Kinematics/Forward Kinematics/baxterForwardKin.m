% Forward kinematics for right arm of the baxter

% principle axes
x = [1; 0; 0];
y = [0; 1; 0];
z = [0; 0; 1];

% frame displacements
PB0 = [.06375; -.25888; .119217];
P01 = rotz(-pi/4) * [.069; 0; .270];
P12 = [0; 0; 0];
P23 = rotz(-pi/4) * [.36435; 0; -.069];
P34 = [0; 0; 0];
P45 = rotz(-pi/4) * [.37429; 0; -.01];
P6T = rotz(-pi/4) * [.229; 0; 0];

% axes of rotation
H0 = rotz(-pi/4) * z;
H1 = rotz(-pi/4) * y;
H2 = rotz(-pi/4) * x;
H3 = rotz(-pi/4) * y;
H4 = rotz(-pi/4) * x;
H5 = rotz(-pi/4) * y;
H6 = rotz(-pi/4) * x;

q = zeros(1,7);

% rotations
RB0 = angvec2r(q(1),H0);
R01 = angvec2r(q(2),H1);
R12 = angvec2r(q(3),H2);
R23 = angvec2r(q(4),H3);
R34 = angvec2r(q(5),H4);
R45 = angvec2r(q(6),H5);
R56 = angvec2r(q(7),H6);
R6T = eye(3);
RBT = RB0*R01*R12*R23*R34*R45*R56;

% position kinematics
PBT = PB0 + RB0 * P01 + RB0*R01*R12*P23 + RB0*R01*R12*R23*R34*P45 + RBT*P6T
RBT