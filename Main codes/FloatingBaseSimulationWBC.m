function XD = FloatingBaseSimulationWBC(t,X0)
global robot
numJoints = robot.rotationalJoints;
robot = robot_move(robot,X0(1:30));
q = X0(1:30); %q = [0pB,0etaB,qJ]
qD = X0(31:end); %qD = [0vB,0wB,qjD]
robot.qD = qD;
w1 = X0(34:36);
v1 = X0(31:33);

[H,C] = ForwardDynamics(q,qD); %
J = [robot.J_RAnkle;robot.J_LAnkle];
Hu = H(1:6,:); Cu = C(1:6); Ju = J';
Ju = Ju(1:6,:);
A1 = [Hu,Ju];
b1 = -Cu;
[JQpqpR,JQpqpL] = JQp_qpFeet(robot);
A2 = [robot.J_RAnkle, zeros(6,12)];
b2 = -JQpqpR;
A3 = [robot.J_LAnkle, zeros(6,12)];
b3 = -JQpqpL;
A = [A1;A2;A3];
b = [b1;b2;b3];
qppRef = PDreferenceAcceleration(q,qD);

wBase = 10000; wJoints = 1000; wForces = 1;
Q = zeros(numJoints+18,numJoints+18);
Qb = wBase*eye(6,6);
QJ = wJoints*eye(numJoints,numJoints);
QF = wForces*eye(12,12);
Q(1:6,1:6) = Qb;
Q(7:6+numJoints,7:6+numJoints) = QJ;
Q(7+numJoints:end,7+numJoints:end) = QF;
Q1 = Q(1:6+numJoints,1:6+numJoints);
p = [-(qppRef')*Q1 zeros(1,12)];

% x = quadprog(Q,p,[],[],A,b);
% qpp = x(1:numJoints+6);
tau = zeros(30,1);
tau(12) = 0.1;
qpp = H\(-C + tau);

qD(1:3) = v1 + cross_matrix(w1)*q(1:3);
qD(4:6) = OmeRPY(q(6),q(5))*w1; %qD = [vB,etaBD,qjD]
qDD = [qpp(4:6);qpp(1:3);qpp(7:end)]; %qDD = [vBD,wBD,qjDD]
XD = [qD; qDD]; %XD = [vB,wD,qjD][vBD,wBD,qjDD]
end