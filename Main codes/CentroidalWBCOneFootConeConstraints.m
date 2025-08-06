function XD = CentroidalWBCOneFootConeConstraints(t,X0)
    % x->decision variables
    % x = [\ddot{q}_B (6), \ddot{q}_J (24), F_ext (6), cjk (16)] 
    % F_ext = [M_ext, f_ext]
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

[JSpatialR,JpqpSpatialR,~,~] = jacobianSpatialNotation(robot,qD);
J = JSpatialR;
Hu = H(1:6,:); Cu = C(1:6); Ju = J';
Ju = Ju(1:6,:);
Ju(1:6,1:6) = Ju(1:6,1:6);

A1 = [Hu,Ju,zeros(6,16)];
b1 = -Cu;
A2 = [JSpatialR, zeros(6,6), zeros(6,16)];
b2 = -JpqpSpatialR;
M = matrixVertex();
[Afeq,bfeq,Afineq,bfineq] = frictionConstraints(0.7,M);
% A = [A1;A2];
% b = [b1;b2];
A = [A1;A2;Afeq(1:3,:)];
b = [b1;b2;bfeq(1:3)];
qppRef = PDreferenceAcceleration(q,qD);
hGp = PDreferenceMomentumRate(AG,qD);

% wCoML = 10000; wCoMK = 0; wBase = 1; wJoints = 1; wForces = 1; %[A1]
wCoML = 1; wCoMK = 0; wBase = 1; wJoints = 1; wForces = 1; %[A1]
Q = zeros(numJoints+6+6+16,numJoints+6+6+16); %joints + 6 (floatingBase) + 6 (external forces on feet) + 16 coeffForces
QJ = zeros(numJoints+6,numJoints+6); %joints + 6 (floatingBase)
Qc(1:3,1:3) = wCoMK*eye(3,3);
Qc(4:6,4:6) = wCoML*eye(3,3);
QJ(1:6,1:6) = wBase*eye(6,6);
QJ(7:end,7:end) = wJoints*eye(numJoints,numJoints);
QF = wForces*eye(6,6);
Q(1:numJoints+6,1:numJoints+6) = (AG')*Qc*AG + QJ;
Q(7+numJoints:7+numJoints+5,7+numJoints:7+numJoints+5) = QF;
p = [(AGpqp')*Qc*AG - (hGp')*Qc*AG - (qppRef')*QJ, zeros(1,6), zeros(1,16)];

options = optimset('Display', 'off');
% x = quadprog(Q,p,[],[],A,b,[],[],[],options);
x = quadprog(Q,p,-Afineq,bfineq,A,b,[],[],[],options);

F(1:3) = -x(34:36);
F(4:6) = -x(31:33);
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

function [Aeq,beq,Aineq,bineq] = frictionConstraints(mu,matrixVertex)
    % x->decision variables
    % x = [\ddot{q}_B (6), \ddot{q}_J (24), F_ext (6), cjk (16)] 
    % F_ext = [M_ext, f_ext]
    frictionMatrix = [mu,0,-mu,0;
                      0,mu,0,-mu;
                      1,1,1,1];
   Aeq = zeros(6,52); % 6 equations, 52 decision variables
   Aeq(1,34) = -1;
   Aeq(1,37:40) = frictionMatrix(1,:);
   Aeq(1,41:44) = frictionMatrix(1,:);
   Aeq(1,45:48) = frictionMatrix(1,:);
   Aeq(1,49:52) = frictionMatrix(1,:);
   
   Aeq(2,35) = -1;
   Aeq(2,37:40) = frictionMatrix(2,:);
   Aeq(2,41:44) = frictionMatrix(2,:);
   Aeq(2,45:48) = frictionMatrix(2,:);
   Aeq(2,49:52) = frictionMatrix(2,:);
   
   Aeq(3,36) = -1;
   Aeq(3,37:40) = frictionMatrix(3,:);
   Aeq(3,41:44) = frictionMatrix(3,:);
   Aeq(3,45:48) = frictionMatrix(3,:);
   Aeq(3,49:52) = frictionMatrix(3,:);
   
   Aeq(4,31) = -1;
   Aeq(5,32) = -1;
   Aeq(6,33) = -1;
   
   auxMatrix = (matrixVertex{1}*frictionMatrix);
   Aeq(4,37:40) = auxMatrix(1,:);
   Aeq(5,37:40) = auxMatrix(2,:);
   Aeq(6,37:40) = auxMatrix(3,:);
   
   auxMatrix = (matrixVertex{2}*frictionMatrix);
   Aeq(4,41:44) = auxMatrix(1,:);
   Aeq(5,41:44) = auxMatrix(2,:);
   Aeq(6,41:44) = auxMatrix(3,:);
   
   auxMatrix = (matrixVertex{3}*frictionMatrix);
   Aeq(4,45:48) = auxMatrix(1,:);
   Aeq(5,45:48) = auxMatrix(2,:);
   Aeq(6,45:48) = auxMatrix(3,:);
   
   auxMatrix = (matrixVertex{4}*frictionMatrix);
   Aeq(4,49:52) = auxMatrix(1,:);
   Aeq(5,49:52) = auxMatrix(2,:);
   Aeq(6,49:52) = auxMatrix(3,:);
   
   beq= zeros(6,1);
   
   Aineq = zeros(52,52);
   Aineq(37:end,37:end) = eye(16);
   bineq= zeros(52,1);
end

function M = matrixVertex()
    FootWidth = 0.05;
    FootLength = 0.1;
    v = cell(4,1);
    M = cell(4,1);
    v{1} = [FootLength/2;FootWidth/2;0];
    v{2} = [FootLength/2;-FootWidth/2;0];
    v{3} = [-FootLength/2;-FootWidth/2;0];
    v{4} = [-FootLength/2;FootWidth/2;0];
    for i=1:4
        M{i} = cross_matrix(v{i});
    end
end