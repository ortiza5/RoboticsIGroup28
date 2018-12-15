%Jacobian example from previous project
function [J]=jacobian(q1,q2,q3)
l2=0.135;
l1 = l2;
a=0.035;
l4=l1+a;
l3=0.025;
J11=-(l1*cos(q1)*cos(q2)+l2*sin(q3)*cos(q1));
J12=l1*sin(q1)*sin(q2);
J13=-l1*cos(q2)*sin(q1);
J21=0;
J22=l1*cos(q2);
J23=l2*sin(q3);
J31=-l1*cos(q2)*sin(q1);
J32=-l2*sin(q2)*cos(q1);
J33=l2*cos(q3)*cos(q1);
J=[J11 J12 J13;J31 J32 J33;J21 J22 J23];
end