%
% kincontrol3linkfunc.m
% ode function for kincontrol3link.m
%
function xdot=kincontrol3linkfunc(t,x,dummy,l1,l2,l3,c,r,xd,Kp,gamma)
  
  s1=sin(x(1));c1=cos(x(1));
  s12=sin(x(1)+x(2));c12=cos(x(1)+x(2));
  s123=sin(x(1)+x(2)+x(3));c123=cos(x(1)+x(2)+x(3));
  
% JT

  J=[1 1 1; -l1*s1-l2*s12-l3*s123 -l2*s12-l3*s123 -l3*s123;
    l1*c1+l2*c12+l3*c123 l2*c12+l3*c123 +l3*c123];
  
% task coordinate
  th=x(1)+x(2)+x(3);
  p=[l1*c1+l2*c12+l3*c123;l1*s1+l2*s12+l3*s123];
  xT=[th;p];

% penalty function  
  g1=r^2-norm(p-c)^2;
  f1_p=(g1>0).*g1.^2;
  pf1pxT=gamma*[ 0 ;(g1>0)*2*g1*(-2*(p-c))];

% joint 2 joint limits
g2max=pi/2;
%  g2max=pi;
  g2=x(2)-g2max;
  f2_p=(g2>0).*g2.^2;
  pf2pq=50*(g2>0)*[0;2*(x(2)-g2max);0];

  g2min=-pi/2;
%  g2min=-pi;
  g3=g2min-x(2);
  f3_p=(g3>0).*g3.^2;
  pf3pq=50*(g3>0)*[0;-2*(g2min-x(2));0];
  
%% damped least square kinematic controller with penalty function
  epsilon=0.5;
  u=-inv(J'*Kp*J+epsilon*eye(size(J'*J)))*(J'*(Kp*(xT-xd)+ pf1pxT)+pf2pq+pf3pq);

% for the jacobian inverse and transpose controllers, we can just lump the
% h as a single constant in a matrix (it is not a function of q!)
%% jacobian inverse
%  epsilon=0.5;
%  u=-inv(epsilon*eye(size(J)))*( (J\(Kp*(xT-xd))) + pinv(J)*pf1pxT+ pf2pq + pf3pq);
%% jacobian transpose
%  epsilon = .7;
%  u=-inv(epsilon*eye(size(J)))*(J'*(Kp*(xT-xd)+ pf1pxT)+pf2pq+pf3pq);
  
 % rescale commanded values one exceeds the desired max
 u_max = 1.5;
 if max(abs(u))>u_max
    xdot=u/max(abs(u))*u_max;
 else
    xdot = u;
 end
end

  