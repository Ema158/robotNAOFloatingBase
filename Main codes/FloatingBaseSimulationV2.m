function XD = FloatingBaseSimulationV2(t,X0)
global robot
robot = robot_move(robot,X0(1:30));
R01 = robot.T(1:3,1:3,1);
q = X0(1:30); %q = [pB,etaB,qJ]
qD = X0(31:end); %qD = [vB,wB,qjD]
qDDJ = zeros(24,1);
qDDJ(14) = -0.3;
[baseAcc,~] = RNEAFBV2(q,qD,qDDJ); %q = [pB,etaB,qJ] qD = [vB,wB,qjD]
qD(1:3) = R01*X0(31:33);
qD(4:6) = OmeRPY(q(4),q(5))*R01*X0(34:36); %qD = [vB,etaBD,qjD]
w0D = baseAcc(1:3);
v0D = baseAcc(4:6) - cross_matrix(X0(34:36))*X0(31:33);
% v0D = baseAcc(4:6);
qDD = [v0D; w0D; qDDJ]; %qDD = [vBD,wBD,qjDD]
XD = [qD; qDD]; %XD = [vB,wD,qjD][vBD,wBD,qjDD]
end