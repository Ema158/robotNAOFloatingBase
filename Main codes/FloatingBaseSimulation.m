function XD = FloatingBaseSimulation(t,X0)
global robot
robot = robot_move(robot,X0(1:30));
q = robot.q;
qD = X0(31:end);
qDDJ = zeros(24,1);
qDDJ(13) = -0.1;
[baseAcc,~] = RNEAFB(q,qD,qDDJ); %q = [pB,etaB,qJ] qD = [vB,wB,qjD]
wBD = baseAcc(1:3);
vBD = baseAcc(4:6);
qD(4:6) = OmeRPY(q(4),q(5))*X0(34:36); %qD = [vB,etaBD,qjD]
qDD = [vBD;wBD; qDDJ]; %qDD = [vBD,wBD,qjDD]
XD = [qD; qDD]; %XD = [vB,etaBD,qjD][vBD,wBD,qjDD]
                %X = [pB,etaB,qJ][vB,wB,qjD]
end