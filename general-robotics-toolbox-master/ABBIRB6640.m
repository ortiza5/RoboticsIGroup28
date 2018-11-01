%
% Inverse Kinematics Example for PUMA 560
%

clear all;close all;clc;

ex=[1;0;0]; ey=[0;1;0]; ez=[0;0;1]; zv=[0;0;0];
h1=ez; h2=ey; h3=ey; h4=ex; h5=ey; h6=ex;

p01 = [0;0;0];
p12 = [320;0;780];
p23 = [0;0;1075];
p34 = [1392.5;0;200];
p45 = [0;0;0];
p56 = [0;0;0];
p6T = [200;0;0];
 
IRB6640.H=[h1 h2 h3 h4 h5 h6];
IRB6640.P=[p01 p12 p23 p34 p45 p56 p6T];
IRB6640.joint_type=[0 0 0 0 0 0]; % 6R robot
n=6;

N=10;
%N=1;
for j=1:N
    q=ones(6,1)*pi/4;
    [R0T,p0T,JR,Jp]=jac_cal(IRB6640,q,1,eye(3,3),zeros(3,n),zeros(3,n)); 
    qsols = zeros(6,17);
    qsols(:,17) = q;
    
    % solve for q1
    q1vec = -subproblem4(ey,(p0T - R0T*p6T)/norm(p0T - R0T*p6T),ez,0);
    qsols(1,1:8) = q1vec(1);
    qsols(1,9:16) = q1vec(2);
    
    % solve for q3
    q1a = q1vec(1); q1b = q1vec(2);
    q3vec1 = subproblem3(-p34,p23,ey,norm(rot(ez,-q1a)*(p0T - R0T*p6T)-p12));
    q3vec2 = subproblem3(-p34,p23,ey,norm(rot(ez,-q1b)*(p0T - R0T*p6T)-p12));
    qsols(3,1:4) = q3vec1(1);
    qsols(3,5:8) = q3vec1(2);
    qsols(3,9:12) = q3vec2(1);
    qsols(3,13:16) = q3vec2(2);

    % solve for q2
    q3a = q3vec1(1); q3b = q3vec1(2); q3c = q3vec2(1); q3d = q3vec2(2);
    q2vec1 = subproblem3(-p23-rot(ey,q3a)*p34,p12,ey,norm(rot(ez,-q1a)*(p0T - R0T*p6T)));
    q2vec2 = subproblem3(-p23-rot(ey,q3b)*p34,p12,ey,norm(rot(ez,-q1a)*(p0T - R0T*p6T)));
    q2vec3 = subproblem3(-p23-rot(ey,q3c)*p34,p12,ey,norm(rot(ez,-q1b)*(p0T - R0T*p6T)));
    q2vec4 = subproblem3(-p23-rot(ey,q3d)*p34,p12,ey,norm(rot(ez,-q1b)*(p0T - R0T*p6T)));
    qsols(2,1:2) = q2vec1(1);
    qsols(2,3:4) = q2vec1(2);
    qsols(2,5:6) = q2vec2(1);
    qsols(2,7:8) = q2vec2(2);
    qsols(2,9:10) = q2vec3(1);
    qsols(2,11:12) = q2vec3(2);
    qsols(2,13:14) = q2vec4(1);
    qsols(2,15:16) = q2vec4(2);
    
    % solve for q4 and q5
    % rot(k1,-theta1)q = rot(k2, theta2) * p
    % [theta1, theta2] = subproblem2(p, q, k1, k2)
    q2a = q2vec1(1); q2b = q2vec1(2); q2c = q3vec2(1); q2d = q3vec2(2);
    q2e = q2vec3(1); q2f = q2vec3(2); q2g = q2vec4(1); q2h = q2vec4(2);
    [q4vec1,q5vec1] = subproblem2(ex,rot(ey,-q3a)*rot(ey,-q2a)*rot(ez,-q1a)*R0T*ex,ex,ey);
    [q4vec2,q5vec2] = subproblem2(ex,rot(ey,-q3a)*rot(ey,-q2b)*rot(ez,-q1a)*R0T*ex,ex,ey);
    [q4vec3,q5vec3] = subproblem2(ex,rot(ey,-q3b)*rot(ey,-q2c)*rot(ez,-q1a)*R0T*ex,ex,ey);
    [q4vec4,q5vec4] = subproblem2(ex,rot(ey,-q3b)*rot(ey,-q2d)*rot(ez,-q1a)*R0T*ex,ex,ey);
    [q4vec5,q5vec5] = subproblem2(ex,rot(ey,-q3c)*rot(ey,-q2e)*rot(ez,-q1b)*R0T*ex,ex,ey);
    [q4vec6,q5vec6] = subproblem2(ex,rot(ey,-q3c)*rot(ey,-q2f)*rot(ez,-q1b)*R0T*ex,ex,ey);
    [q4vec7,q5vec7] = subproblem2(ex,rot(ey,-q3d)*rot(ey,-q2g)*rot(ez,-q1b)*R0T*ex,ex,ey);
    [q4vec8,q5vec8] = subproblem2(ex,rot(ey,-q3d)*rot(ey,-q2h)*rot(ez,-q1b)*R0T*ex,ex,ey);
    
    qsols(4,1) = q4vec1(1);
    qsols(4,2) = q4vec1(2);
    qsols(4,3) = q4vec2(1);
    qsols(4,4) = q4vec2(2);
    qsols(4,5) = q4vec3(1);
    qsols(4,6) = q4vec3(2);
    qsols(4,7) = q4vec4(1);
    qsols(4,8) = q4vec4(2);
    qsols(4,9) = q4vec5(1);
    qsols(4,10) = q4vec5(2);
    qsols(4,11) = q4vec6(1);
    qsols(4,12) = q4vec6(2);
    qsols(4,13) = q4vec7(1);
    qsols(4,14) = q4vec7(2);
    qsols(4,15) = q4vec8(1);
    qsols(4,16) = q4vec8(2);
    
    qsols(5,1) = q5vec1(1);
    qsols(5,2) = q5vec1(2);
    qsols(5,3) = q5vec2(1);
    qsols(5,4) = q5vec2(2);
    qsols(5,5) = q5vec3(1);
    qsols(5,6) = q5vec3(2);
    qsols(5,7) = q5vec4(1);
    qsols(5,8) = q5vec4(2);
    qsols(5,9) = q5vec5(1);
    qsols(5,10) = q5vec5(2);
    qsols(5,11) = q5vec6(1);
    qsols(5,12) = q5vec6(2);
    qsols(5,13) = q5vec7(1);
    qsols(5,14) = q5vec7(2);
    qsols(5,15) = q5vec8(1);
    qsols(5,16) = q5vec8(2);
    
    % solve for q6
    q4a = q4vec1(1); q4b = q4vec1(2); q4c = q4vec2(1); q4d = q4vec2(2);
    q4e = q4vec3(1); q4f = q4vec3(2); q4g = q4vec4(1); q4h = q4vec4(2);
    q4i = q4vec5(1); q4j = q4vec5(2); q4k = q4vec6(1); q4l = q4vec6(2);
    q4m = q4vec7(1); q4n = q4vec7(2); q4o = q4vec8(1); q4p = q4vec8(2);
    
    q5a = q5vec1(1); q5b = q5vec1(2); q5c = q5vec2(1); q5d = q5vec2(2);
    q5e = q5vec3(1); q5f = q5vec3(2); q5g = q5vec4(1); q5h = q5vec4(2);
    q5i = q5vec5(1); q5j = q5vec5(2); q5k = q5vec6(1); q5l = q5vec6(2);
    q5m = q5vec7(1); q5n = q5vec7(2); q5o = q5vec8(1); q5p = q5vec8(2);
    
    q6vec1 = subproblem1(ey,rot(ey,-q5a)*rot(ex,-q4a)*rot(ey,-q3a)*rot(ey,-q2a)*rot(ez,-q1a)*R0T*ey,ex);
    q6vec2 = subproblem1(ey,rot(ey,-q5b)*rot(ex,-q4b)*rot(ey,-q3a)*rot(ey,-q2a)*rot(ez,-q1a)*R0T*ey,ex);
    q6vec3 = subproblem1(ey,rot(ey,-q5c)*rot(ex,-q4c)*rot(ey,-q3a)*rot(ey,-q2b)*rot(ez,-q1a)*R0T*ey,ex);
    q6vec4 = subproblem1(ey,rot(ey,-q5d)*rot(ex,-q4d)*rot(ey,-q3a)*rot(ey,-q2b)*rot(ez,-q1a)*R0T*ey,ex);
    q6vec5 = subproblem1(ey,rot(ey,-q5e)*rot(ex,-q4e)*rot(ey,-q3b)*rot(ey,-q2c)*rot(ez,-q1a)*R0T*ey,ex);
    q6vec6 = subproblem1(ey,rot(ey,-q5f)*rot(ex,-q4f)*rot(ey,-q3b)*rot(ey,-q2c)*rot(ez,-q1a)*R0T*ey,ex);
    q6vec7 = subproblem1(ey,rot(ey,-q5g)*rot(ex,-q4g)*rot(ey,-q3b)*rot(ey,-q2d)*rot(ez,-q1a)*R0T*ey,ex);
    q6vec8 = subproblem1(ey,rot(ey,-q5h)*rot(ex,-q4h)*rot(ey,-q3b)*rot(ey,-q2d)*rot(ez,-q1a)*R0T*ey,ex);
    q6vec9 = subproblem1(ey,rot(ey,-q5i)*rot(ex,-q4i)*rot(ey,-q3c)*rot(ey,-q2e)*rot(ez,-q1b)*R0T*ey,ex);
    q6vec10 = subproblem1(ey,rot(ey,-q5j)*rot(ex,-q4j)*rot(ey,-q3c)*rot(ey,-q2e)*rot(ez,-q1b)*R0T*ey,ex);
    q6vec11 = subproblem1(ey,rot(ey,-q5k)*rot(ex,-q4k)*rot(ey,-q3c)*rot(ey,-q2f)*rot(ez,-q1b)*R0T*ey,ex);
    q6vec12 = subproblem1(ey,rot(ey,-q5l)*rot(ex,-q4l)*rot(ey,-q3c)*rot(ey,-q2f)*rot(ez,-q1b)*R0T*ey,ex);
    q6vec13 = subproblem1(ey,rot(ey,-q5m)*rot(ex,-q4m)*rot(ey,-q3d)*rot(ey,-q2g)*rot(ez,-q1b)*R0T*ey,ex);
    q6vec14 = subproblem1(ey,rot(ey,-q5n)*rot(ex,-q4n)*rot(ey,-q3d)*rot(ey,-q2g)*rot(ez,-q1b)*R0T*ey,ex);
    q6vec15 = subproblem1(ey,rot(ey,-q5o)*rot(ex,-q4o)*rot(ey,-q3d)*rot(ey,-q2h)*rot(ez,-q1b)*R0T*ey,ex);
    q6vec16 = subproblem1(ey,rot(ey,-q5p)*rot(ex,-q4p)*rot(ey,-q3d)*rot(ey,-q2h)*rot(ez,-q1b)*R0T*ey,ex);
    
    qsols(6,1) = q6vec1(1);
    qsols(6,2) = q6vec2(1);
    qsols(6,3) = q6vec3(1);
    qsols(6,4) = q6vec4(1);
    qsols(6,5) = q6vec5(1);
    qsols(6,6) = q6vec6(1);
    qsols(6,7) = q6vec7(1);
    qsols(6,8) = q6vec8(1);
    qsols(6,9) = q6vec9(1);
    qsols(6,10) = q6vec10(1);
    qsols(6,11) = q6vec11(1);
    qsols(6,12) = q6vec12(1);
    qsols(6,13) = q6vec13(1);
    qsols(6,14) = q6vec14(1);
    qsols(6,15) = q6vec15(1);
    qsols(6,16) = q6vec16(1);
    
    % check forward kinematics still agrees
    for i=1:8
        [R2,p2,JR,Jp]=jac_cal(IRB6640,qsols(:,i),...
                         1,eye(3,3),zeros(3,6),zeros(3,6));
        jac(j,i,1:6,1:6)=[JR;Jp];
        jacsv(j,1:6,i)=svd([JR;Jp]);
        RR(j,i,1:3,1:3)=R2;pp(j,1:3,i)=p2;
        errp(j,i)=norm(p2-p0T);errR(j,i)=norm(R2-R0T);
    end
end

qsols
