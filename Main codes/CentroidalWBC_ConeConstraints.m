function XD = CentroidalWBC_ConeConstraints(t,X0)
    % x->decision variables
    % x = [\ddot{q}_B (6), \ddot{q}_J (24), F_ext (6), cjk (16)] 
    % F_ext = [M_ext, f_ext]
global robot mom F FR FL
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
J = [JSpatialR;JSpatialL];
Hu = H(1:6,:); Cu = C(1:6); Ju = J';
Ju = Ju(1:6,:);

A1 = [Hu,-Ju,zeros(6,16),zeros(6,16)];
b1 = -Cu;
A2 = [JSpatialR, zeros(6,12), zeros(6,16), zeros(6,16)];
b2 = -JpqpSpatialR;
A3 = [JSpatialL, zeros(6,12), zeros(6,16), zeros(6,16)];
b3 = -JpqpSpatialL;
M = matrixVertex();
[Afeq,bfeq,Afineq,bfineq] = frictionConstraints(0.7,M);
% A = [A1;A2];
% b = [b1;b2];
A = [A1;A2;A3;Afeq];
b = [b1;b2;b3;bfeq];
qppRef = PDreferenceAcceleration(q,qD);
hGp = PDreferenceMomentumRate(AG,qD);

wCoML = 10000; wCoMK = 0; wBase = 1; wJoints = 1; wForces = 1; %[A1]
% wCoML = 1; wCoMK = 0; wBase = 1; wJoints = 1; wForces = 1; %[A1]
Q = zeros(numJoints+6+12+32,numJoints+6+12+32); %joints + 6 (floatingBase) + 12 (external forces on feet) + 32 coeffForces
QJ = zeros(numJoints+6,numJoints+6); %joints + 6 (floatingBase)
Qc(1:3,1:3) = wCoMK*eye(3,3);
Qc(4:6,4:6) = wCoML*eye(3,3);
QJ(1:6,1:6) = wBase*eye(6,6);
QJ(7:end,7:end) = wJoints*eye(numJoints,numJoints);
QF = wForces*eye(12,12);
Q(1:numJoints+6,1:numJoints+6) = (AG')*Qc*AG + QJ;
Q(7+numJoints:7+numJoints+11,7+numJoints:7+numJoints+11) = QF;
p = [(AGpqp')*Qc*AG - (hGp')*Qc*AG - (qppRef')*QJ, zeros(1,12), zeros(1,32)];

options = optimset('Display', 'off');
% x = quadprog(Q,p,[],[],A,b,[],[],[],options);
x = quadprog(Q,p,-Afineq,bfineq,A,b,[],[],[],options);

F(1:3) = x(34:36);
F(4:6) = x(31:33);
FR = x(31:36);
FL = x(37:42);
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
   Aeq = zeros(6,74); % 6 equations, 74 decision variables
   Aeq(1,34) = -1;
   Aeq(1,43:46) = frictionMatrix(1,:);
   Aeq(1,47:50) = frictionMatrix(1,:);
   Aeq(1,51:54) = frictionMatrix(1,:);
   Aeq(1,55:58) = frictionMatrix(1,:);
   
   Aeq(2,35) = -1;
   Aeq(2,43:46) = frictionMatrix(2,:);
   Aeq(2,47:50) = frictionMatrix(2,:);
   Aeq(2,51:54) = frictionMatrix(2,:);
   Aeq(2,55:58) = frictionMatrix(2,:);
   
   Aeq(3,36) = -1;
   Aeq(3,43:46) = frictionMatrix(3,:);
   Aeq(3,47:50) = frictionMatrix(3,:);
   Aeq(3,51:54) = frictionMatrix(3,:);
   Aeq(3,55:58) = frictionMatrix(3,:);
   
   Aeq(4,31) = -1;
   Aeq(5,32) = -1;
   Aeq(6,33) = -1;
   
   auxMatrix = (matrixVertex{1}*frictionMatrix);
   Aeq(4,43:46) = auxMatrix(1,:);
   Aeq(5,43:46) = auxMatrix(2,:);
   Aeq(6,43:46) = auxMatrix(3,:);
   
   auxMatrix = (matrixVertex{2}*frictionMatrix);
   Aeq(4,47:50) = auxMatrix(1,:);
   Aeq(5,47:50) = auxMatrix(2,:);
   Aeq(6,47:50) = auxMatrix(3,:);
   
   auxMatrix = (matrixVertex{3}*frictionMatrix);
   Aeq(4,51:54) = auxMatrix(1,:);
   Aeq(5,51:54) = auxMatrix(2,:);
   Aeq(6,51:54) = auxMatrix(3,:);
   
   auxMatrix = (matrixVertex{4}*frictionMatrix);
   Aeq(4,55:58) = auxMatrix(1,:);
   Aeq(5,55:58) = auxMatrix(2,:);
   Aeq(6,55:58) = auxMatrix(3,:);
   %
   Aeq(7,40) = -1;
   Aeq(7,59:62) = frictionMatrix(1,:);
   Aeq(7,63:66) = frictionMatrix(1,:);
   Aeq(7,67:70) = frictionMatrix(1,:);
   Aeq(7,71:74) = frictionMatrix(1,:);
   
   Aeq(8,41) = -1;
   Aeq(8,59:62) = frictionMatrix(2,:);
   Aeq(8,63:66) = frictionMatrix(2,:);
   Aeq(8,67:70) = frictionMatrix(2,:);
   Aeq(8,71:74) = frictionMatrix(2,:);
   
   Aeq(9,42) = -1;
   Aeq(9,59:62) = frictionMatrix(3,:);
   Aeq(9,63:66) = frictionMatrix(3,:);
   Aeq(9,67:70) = frictionMatrix(3,:);
   Aeq(9,71:74) = frictionMatrix(3,:);
   
   Aeq(10,37) = -1;
   Aeq(11,38) = -1;
   Aeq(12,39) = -1;
   
   auxMatrix = (matrixVertex{1}*frictionMatrix);
   Aeq(10,59:62) = auxMatrix(1,:);
   Aeq(11,59:62) = auxMatrix(2,:);
   Aeq(12,59:62) = auxMatrix(3,:);
   
   auxMatrix = (matrixVertex{2}*frictionMatrix);
   Aeq(10,63:66) = auxMatrix(1,:);
   Aeq(11,63:66) = auxMatrix(2,:);
   Aeq(12,63:66) = auxMatrix(3,:);
   
   auxMatrix = (matrixVertex{3}*frictionMatrix);
   Aeq(10,67:70) = auxMatrix(1,:);
   Aeq(11,67:70) = auxMatrix(2,:);
   Aeq(12,67:70) = auxMatrix(3,:);
   
   auxMatrix = (matrixVertex{4}*frictionMatrix);
   Aeq(10,71:74) = auxMatrix(1,:);
   Aeq(11,71:74) = auxMatrix(2,:);
   Aeq(12,71:74) = auxMatrix(3,:);
   
   beq= zeros(12,1);
   
   Aineq = zeros(74,74);
   Aineq(43:end,43:end) = eye(32);
   bineq= zeros(74,1);
end

function M = matrixVertex()
    FootWidth = 0.05;
    FootLength = 0.1;
    v = cell(4,1);
    M = cell(4,1);
%     v{1} = [FootLength/2;FootWidth/2;0];
%     v{2} = [FootLength/2;-FootWidth/2;0];
%     v{3} = [-FootLength/2;-FootWidth/2;0];
%     v{4} = [-FootLength/2;FootWidth/2;0];
    
%     v{1} = [0.1;0.025;0];
%     v{2} = [0.1;-0.025;0];
%     v{3} = [-0.05;0.025;0];
%     v{4} = [-0.05;-0.025;0];
    
    v{1} = [0.1;0.01;0];
    v{2} = [0.1;-0.01;0];
    v{3} = [-0.05;0.01;0];
    v{4} = [-0.05;-0.01;0];
    for i=1:4
        M{i} = cross_matrix(v{i});
    end
end