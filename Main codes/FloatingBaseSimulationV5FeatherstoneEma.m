function XD = FloatingBaseSimulationV5FeatherstoneEma(t,X0)
global robot tau
robot = robot_move(robot,X0(1:30));
q = X0(1:30); %q = [0pB,0etaB,qJ]
qD = X0(31:end); %qD = [0vB,0wB,qjD]
w1 = X0(34:36);
v1 = X0(31:33);
qDDJ = zeros(24,1);
% qDDJ(13) = -0.1;
% qDDJ(18) = -0.1;
qDDJ(1) = -0.1;
[baseAcc,tau] = RNEAFBV5FeatherstoneEma(q,qD,qDDJ); %q = [pB,etaB,qJ] qD = [vB,wB,qjD]
qD(1:3) = v1 + cross_matrix(w1)*q(1:3);
qD(4:6) = OmeRPY(q(6),q(5))*w1; %qD = [vB,etaBD,qjD]
w0D = baseAcc(1:3);
v0D = baseAcc(4:6);
qDD = [v0D; w0D; qDDJ]; %qDD = [vBD,wBD,qjDD]
XD = [qD; qDD]; %XD = [vB,wD,qjD][vBD,wBD,qjDD]
end