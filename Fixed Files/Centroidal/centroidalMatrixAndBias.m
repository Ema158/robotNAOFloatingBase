function [AG,AGpqp] = centroidalMatrixAndBias(H,C,robot)
m = robot.mass;
T = robot.T;
Ic1 = H(1:6,1:6);
F = H(1:6,7:end);
p1G = [Ic1(3,5);Ic1(1,6);Ic1(2,4)]/m;
p1c = C(1:6);
X1G = zeros(6,6);
R01 = T(1:3,1:3,1);
X1G(1:3,1:3) = R01;
X1G(4:6,4:6) = R01;
X1G(1:3,4:6) = -R01*cross_matrix(p1G);
AG = [X1G*Ic1,X1G*F];
AGpqp = X1G*p1c;
end