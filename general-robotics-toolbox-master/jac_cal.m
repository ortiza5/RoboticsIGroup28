%
% recursive computation of Jacobian (in inertial frame)
% 
% Usage:
% 
% for symbolic: q=[q1;q2;q3;q4;q5;q6];JR=sym(zeros(3,6));Jp=sym(zeros(3,6));
% for numeric: JR=zeros(3,6);Jp=zeros(3,6);
% [R,p,JR,Jp]=jac_cal(robot,q,1,eye(3,3),JR,Jp);
% J=[JR;Jp];
% J is (J_T)_0
%

function [R0n,p,JR,Jp]=jac_cal(robot,q,i,R,JR,Jp)
  h_i=robot.H(1:3,i);
  R0=R;
  if robot.joint_type(i)==0
    p_i1_i=robot.P(1:3,i);
    R=R*expm(hat(h_i)*q(i));
  else
    p_i1_i=robot.P(1:3,i)+q(i)*h_i;
    R=R;
  end
  if i==numel(robot.joint_type)
    p=R*robot.P(1:3,i+1);
    R0n=R;
  else
    [R0n,p,JR,Jp]=jac_cal(robot,q,i+1,R,JR,Jp);
  end
  
  if robot.joint_type(i) ==0
    JR(1:3,i)=R*h_i;
    Jp(1:3,i)=hat(R*h_i)*p;
  else
    JR(1:3,i)=zeros(3,1);
    Jp(1:3,i)=R*h_i;
  end

  p=p+R0*p_i1_i;

