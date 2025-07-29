function XD = FloatingBaseSimulationWBC(t,X0)
global robot
numJoints = robot.rotationalJoints;
robot = robot_move(robot,X0(1:30));
q = X0(1:30); %q = [0pB,0etaB,qJ]
qD = X0(31:end); %qD = [0vB,0wB,qjD]
robot.qD = qD;
w1 = X0(34:36);
v1 = X0(31:33);
T = robot.T;
X10 = VelocityMatrix(T(:,:,1));

[H,C] = ForwardDynamics(q,qD); %

[JSpatialR,JpqpSpatialR,JSpatialL,JpqpSpatialL] = jacobianSpatialNotation(robot,qD);
J = [JSpatialR;JSpatialL];
Hu = H(1:6,:); Cu = C(1:6); Ju = J';
Ju = Ju(1:6,:);
Ju(1:6,1:6) = Ju(1:6,1:6);

A1 = [Hu,Ju];
b1 = -Cu;
A2 = [JSpatialR, zeros(6,12)];
b2 = -JpqpSpatialR;
A3 = [JSpatialL, zeros(6,12)];
b3 = -JpqpSpatialL;
A = [A1;A2;A3];
b = [b1;b2;b3];
qppRef = PDreferenceAcceleration(q,qD);

wBase = 1000; wJoints = 1; wForces = 1; %[A1]
Q = zeros(numJoints+18,numJoints+18);
Qb = wBase*eye(6,6);
QJ = wJoints*eye(numJoints,numJoints);
QF = wForces*eye(12,12);
Q(1:6,1:6) = Qb;
Q(7:6+numJoints,7:6+numJoints) = QJ;
Q(7+numJoints:end,7+numJoints:end) = QF;
Q1 = Q(1:6+numJoints,1:6+numJoints);
p = [-(qppRef')*Q1 zeros(1,12)];

options = optimset('Display', 'off');
x = quadprog(Q,p,[],[],A,b,[],[],[],options);
qpp = x(1:numJoints+6);
aBaseFrame1 = qpp(1:6);
aBaseFrame0 = X10\aBaseFrame1;
qD(1:3) = v1 + cross_matrix(w1)*q(1:3);

qD(4:6) = OmeRPY(q(6),q(5))*w1; %qD = [vB,etaBD,qjD]

qDD = [aBaseFrame0(4:6);aBaseFrame0(1:3);qpp(7:end)]; %qDD = [vBD,wBD,qjDD]
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