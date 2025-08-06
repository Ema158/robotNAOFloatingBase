function XD = FloatingBaseSimulationCentroidalWBCOneFoot(t,X0)
global robot mom F
numJoints = robot.rotationalJoints;
robot = robot_move(robot,X0(1:30));
q = X0(1:30); %q = [0pB,0etaB,qJ]
qD = X0(31:end); %qD = [0vB,0wB,qjD]
robot.qD = qD;
w1 = X0(34:36);
v1 = X0(31:33);
T = robot.T;
X10 = VelocityMatrix(T(:,:,1));

[H,C,Cg] = ForwardDynamicsCentroidal(q,qD);
[AG,AGpqp] = centroidalMatrixAndBias(H,Cg,robot);
qDaux = qD;
qDaux(1:6) = X10*[qDaux(4:6);qDaux(1:3)];
mom = AG*qDaux;

[JSpatialR,JpqpSpatialR,JSpatialL,JpqpSpatialL] = jacobianSpatialNotation(robot,qD);
J = JSpatialR;
Hu = H(1:6,:); Cu = C(1:6); Ju = J';
Ju = Ju(1:6,:);
Ju(1:6,1:6) = Ju(1:6,1:6);

A1 = [Hu,-Ju];
b1 = -Cu;
A2 = [JSpatialR, zeros(6,6)];
b2 = -JpqpSpatialR;
A3 = [JSpatialL, zeros(6,12)];
b3 = -JpqpSpatialL;
A = [A1;A2];
b = [b1;b2];
qppRef = PDreferenceAcceleration(q,qD);
hGp = PDreferenceMomentumRate(AG,qD);

wCoML = 10000; wCoMK = 0; wBase = 1; wJoints = 1; wForces = 1; %[A1]
Q = zeros(numJoints+12,numJoints+12); %joints + 6 (floatingBase) + 12 (external forces on feet)
QJ = zeros(numJoints+6,numJoints+6);
Qc(1:3,1:3) = wCoMK*eye(3,3);
Qc(4:6,4:6) = wCoML*eye(3,3);
QJ(1:6,1:6) = wBase*eye(6,6);
QJ(7:end,7:end) = wJoints*eye(numJoints,numJoints);
QF = wForces*eye(6,6);
Q(1:numJoints+6,1:numJoints+6) = (AG')*Qc*AG + QJ;
Q(7+numJoints:end,7+numJoints:end) = QF;
p = [(AGpqp')*Qc*AG - (hGp')*Qc*AG - (qppRef')*QJ,zeros(1,6)];

options = optimset('Display', 'off');
x = quadprog(Q,p,[],[],A,b,[],[],[],options);
% xAux = Ju*x(31:36);
F(1:3) = x(34:36);
F(4:6) = x(31:33);
% F(4:6) = x(40:42);
qpp = x(1:numJoints+6);
aBaseFrame1 = qpp(1:6);
aBaseFrame0 = X10\aBaseFrame1;
% if abs(t-0.1)<1e-4
%    aBaseFrame0(4) = aBaseFrame0(4) + 10;
% end
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