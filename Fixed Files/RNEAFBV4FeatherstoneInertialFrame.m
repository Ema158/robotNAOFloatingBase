%Returns floating base acceleration and joint torques given joints accelerations
function [baseAcc_Frame0] = RNEAFBV4FeatherstoneInertialFrame(q,qD,qDDJ)  
  global robot
  ant =       [0,... 1
             1,... 2
             2,... 3
             3,... 4
             4,... 5
             5,... 6
             6,... 7
             7,... 8
             1,... 9
             9,...10
             10,...11
             11,...12
             12,...13
             13,...14
             14,...15
             1,...16
             16,...17
             17,...18
             18,...19
             19,...20
             1,...21
             21,...22
             22,...23
             23,...24
             24,...25
             1,...26
             26,...27
             27];% 28 
  CoM_j = robot.InertialParameters.CoM;
  M = robot.InertialParameters.masse; % Masas adjuntas a cada marco
  I = robot.InertialParameters.I;
  frames = robot.frames;
  T = robot.T;
  piTi = zeros(4,4,frames);
  X10 = VelocityMatrix(T(:,:,1));
  qD(1:6) = X10*[qD(4:6);qD(1:3)]; %[w,v]
  for i=2:frames
      piTi(:,:,i) = inverseTransMatrix(T(:,:,ant(i)))*T(:,:,i);
  end
  N1 = 2:7; %Not frame 8 beacuase it does not have a joint
  N2 = 9:14; %Not frame 15 beacuase it does not have a joint
  N3 = 16:20;
  N4 = 21:25;
  N5 = 26:27; %Not frame 28 beacuase it does not have a joint 
  virtualFrameCounter = 1;
  [X1,Jc1,pc1] = forwardBackwardOneChain(robot,N1,piTi,qD,qDDJ,virtualFrameCounter);
  virtualFrameCounter = virtualFrameCounter + 1;
  [X2,Jc2,pc2] = forwardBackwardOneChain(robot,N2,piTi,qD,qDDJ,virtualFrameCounter);
  virtualFrameCounter = virtualFrameCounter + 1;
  [X3,Jc3,pc3] = forwardBackwardOneChain(robot,N3,piTi,qD,qDDJ,virtualFrameCounter);
  [X4,Jc4,pc4] = forwardBackwardOneChain(robot,N4,piTi,qD,qDDJ,virtualFrameCounter);
  [X5,Jc5,pc5] = forwardBackwardOneChain(robot,N5,piTi,qD,qDDJ,virtualFrameCounter);
  
  J1 = inertiaMatrix(I{1},M(1),CoM_j{1});
  Jbasec = J1 + X1{1}'*Jc1{1}*X1{1} + X2{1}'*Jc2{1}*X2{1} + X3{1}'*Jc3{1}*X3{1} + X4{1}'*Jc4{1}*X4{1}...
      + X5{1}'*Jc5{1}*X5{1};
  p1 = crf(qD(1:6))*(J1*qD(1:6));
  pBasec = p1 + X1{1}'*pc1{1} + X2{1}'*pc2{1} + X3{1}'*pc3{1} + X4{1}'*pc4{1} + X5{1}'*pc5{1} ;
  baseAcc_Frame1 = -(Jbasec)\pBasec;
  gravityFactor=1;
  baseAcc_Frame0 = X10\baseAcc_Frame1 + [0;0;0;0;0;-9.81*gravityFactor];
  %% Forward pass again
%   VD1 = zeros(6,length(N1));
%   F1 = zeros(6,length(N1));
%   tau1 = zeros(length(N1));
%   [VD1,F1,tau] = forwardAgain(baseAcc_Frame1,X,zetta,length(N1),betac,Jc);
end

function [VD,F,tau] = forwardAgain(baseAcc,X,zetta,N,betac,Jc)
    VD = zeros(6,N);
    F = zeros(6,N);
    tau = zeros(N,1);
    VD(:,1) = baseAcc;
    for i=1:N
        VD(:,i+1) = X{i}*VD(:,i) + zetta{i};
        F(:,i+1) = Jc{i}*VD(:,i+1) + betac{i};
        tau(i) = F(6,i+1);
    end
end

function [X,Jc,pc] = forwardBackwardOneChain(robot,N,T,qD,qDDJ,virtualFrameCounter)
baseDof = 6;
CoM_j = robot.InertialParameters.CoM;
M = robot.InertialParameters.masse; % Masas adjuntas a cada marco
I = robot.InertialParameters.I;
Nsize = length(N);
X = cell(Nsize,1);
velVec = cell(Nsize+1,1);
accVec = cell(Nsize+1,1);
p = cell(Nsize,1);
J = cell(Nsize,1);
velVec{1} = qD(1:6); %baseVel
accVec{1} = zeros(6,1); %baseAcc relative to base
S = [0;0;1;0;0;0];
%% Forward pass
for i=N(1):N(end)
    X{i-N(1)+1} = VelocityMatrix(T(:,:,i));
    velVec{i-N(1)+2} = X{i-N(1)+1}*velVec{i-N(1)+1} + S*qD(i+baseDof-virtualFrameCounter);
    accVec{i-N(1)+2} = X{i-N(1)+1}*accVec{i-N(1)+1} + crm(velVec{i-N(1)+2})*S*qD(i+baseDof-virtualFrameCounter) + S*qDDJ(i-virtualFrameCounter);
    J{i-N(1)+1} = inertiaMatrix(I{i},M(i),CoM_j{i});
    p{i-N(1)+1} = J{i-N(1)+1}*accVec{i-N(1)+2} + crf(velVec{i-N(1)+2})*J{i-N(1)+1}*velVec{i-N(1)+2};
end
%% Backward pass
Jc = cell(Nsize,1);
pc = cell(Nsize,1);
Jc{Nsize} = J{Nsize};
pc{Nsize} = p{Nsize};
for i=(Nsize-1):-1:1
  Jc{i} = J{i} + X{i+1}'*Jc{i+1}*X{i+1};
  pc{i} = p{i} + X{i+1}'*(pc{i+1});
end

end

function X = VelocityMatrix(T)
    X = zeros(6,6);
    R = T(1:3,1:3)';
    p = T(1:3,4);
    X(1:3,1:3) = R;
    X(4:6,1:3) = -R*cross_matrix(p);
    X(4:6,4:6) = R;
end

function J = inertiaMatrix(Ij,mj,CoM_j)
    J = zeros(6,6);
    J(1:3,1:3) = Ij -mj*cross_matrix(CoM_j)*cross_matrix(CoM_j);
    J(1:3,4:6) = mj*cross_matrix(CoM_j);
    J(4:6,1:3) = -mj*cross_matrix(CoM_j);
    J(4:6,4:6) = mj*eye(3,3);
end

function Tinv = inverseTransMatrix(T)
    Tinv = zeros(4,4);
	Tinv(1:3,1:3) = T(1:3,1:3)'; 
    Tinv(1:3,4) = (-T(1:3,1:3)')*T(1:3,4);
    Tinv(4,4) = 1;
end

function m = crm(v)
    m = [cross_matrix(v(1:3)) zeros(3); cross_matrix(v(4:6)) cross_matrix(v(1:3))];
end

function f = crf(v)
f = -crm(v)';
end
    