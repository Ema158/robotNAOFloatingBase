function XD = FloatingTestM10(t,X0)
global robot tau
robot = robot_move(robot,X0(1:30));
q = X0(1:30); %q = [0pB,0etaB,qJ]
qD = X0(31:end); %qD = [0vB,0wB,qjD]
robot.qD = qD;
w1 = X0(34:36);
v1 = X0(31:33);

[H,C] = ForwardDynamics(q,qD); %
[aBaseFrame1,tau] = testID(H(1:6,1:6),H(1:6,7:end),H(7:end,7:end),C(1:6),C(7:end));
T = robot.T;
X10 = VelocityMatrix(T(:,:,1));
aBaseFrame0 = X10\aBaseFrame1;

qpp = zeros(30,1);
qpp(1:6) = aBaseFrame0;
qpp(7) = -0.1;

qD(1:3) = v1 + cross_matrix(w1)*q(1:3);
qD(4:6) = OmeRPY(q(6),q(5))*w1; %qD = [vB,etaBD,qjD]
qDD = [qpp(4:6);qpp(1:3);qpp(7:end)]; %qDD = [vBD,wBD,qjDD]
XD = [qD; qDD]; %XD = [vB,wD,qjD][vBD,wBD,qjDD]
end

function X = VelocityMatrix(T)
    X = zeros(6,6);
    R = T(1:3,1:3)';
    p = T(1:3,4);
    X(1:3,1:3) = R;
    X(4:6,1:3) = -R*cross_matrix(p);
    X(4:6,4:6) = R;
end