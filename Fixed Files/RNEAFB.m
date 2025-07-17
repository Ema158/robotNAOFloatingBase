%Returns floating base acceleration and joint torques given joints accelerations
function [baseAcc_Frame0,tau] = RNEAFB(q,qD,qDDJ)  
  global robot
  ant = [0,... 1
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
  piTi(:,:,1) = inverseTransMatrix(T(:,:,1));
  X01 = zeros(6,6);
  X01(1:3,1:3) = piTi(1:3,1:3,1)';
  X01(4:6,1:3) = -piTi(1:3,1:3,1)'*cross_matrix(piTi(1:3,4,1));
  X01(4:6,4:6) = piTi(1:3,1:3,1)';
  for i=2:frames
      piTi(:,:,i) = inverseTransMatrix(T(:,:,ant(i)))*T(:,:,i);
  end
  N1 = 2:7; %Not frame 8 beacuase it does not have a joint
  N2 = 9:14; %Not frame 15 beacuase it does not have a joint
  N3 = 16:20;
  N4 = 21:25;
  N5 = 26:27; %Not frame 28 beacuase it does not have a joint
  virtualFrameCounter = 1;
  [X1,Xstar1,XstarT1,Jc1,betac1,zetta1] = forwardBackwardOneChain(robot,N1,piTi,qD,qDDJ,virtualFrameCounter);
  virtualFrameCounter = virtualFrameCounter + 1;
  [X2,Xstar2,XstarT2,Jc2,betac2,zetta2] = forwardBackwardOneChain(robot,N2,piTi,qD,qDDJ,virtualFrameCounter);
  virtualFrameCounter = virtualFrameCounter + 1;
  [X3,Xstar3,XstarT3,Jc3,betac3,zetta3] = forwardBackwardOneChain(robot,N3,piTi,qD,qDDJ,virtualFrameCounter);
  [X4,Xstar4,XstarT4,Jc4,betac4,zetta4] = forwardBackwardOneChain(robot,N4,piTi,qD,qDDJ,virtualFrameCounter);
  [X5,Xstar5,XstarT5,Jc5,betac5,zetta5] = forwardBackwardOneChain(robot,N5,piTi,qD,qDDJ,virtualFrameCounter);
  
  J1 = inertiaMatrix(I{1},M(1),CoM_j{1});
  Jbasec = J1 + XstarT1{1}*Jc1{1}*X1{1} + XstarT2{1}*Jc2{1}*X2{1} + XstarT3{1}*Jc3{1}*X3{1}...
      + XstarT4{1}*Jc4{1}*X4{1} + XstarT5{1}*Jc5{1}*X5{1};
%   beta1 = betaVector(qD(4:6),M(1),CoM_j{1},I{1});
  beta1 = betaVector(T(1:3,1:3,1)'*qD(4:6),M(1),CoM_j{1},I{1});
  betaBasec = beta1 + XstarT1{1}*(Jc1{1}*zetta1{1}+ betac1{1}) + XstarT2{1}*(Jc2{1}*zetta2{1}+ betac2{1}) + ...
      XstarT3{1}*(Jc3{1}*zetta3{1}+ betac3{1}) + XstarT4{1}*(Jc4{1}*zetta4{1}+ betac4{1}) + XstarT5{1}*(Jc5{1}*zetta5{1}+ betac5{1});
  baseAcc_Frame1 = -(Jbasec)\betaBasec;
  Rot = zeros(6,6);
  Rot(1:3,1:3) = T(1:3,1:3,1);
  Rot(4:6,4:6) = T(1:3,1:3,1);
  baseAcc_Frame0 = Rot*baseAcc_Frame1;
  %% Forward pass again
  VD1 = zeros(6,length(N1));
  F1 = zeros(6,length(N1));
  tau1 = zeros(length(N1));
  [VD1,F1,tau1] = forwardAgain(baseAcc_Frame1,Xstar1,zetta1,length(N1),betac1,Jc1);
  
  VD2 = zeros(6,length(N2));
  F2 = zeros(6,length(N2));
  tau2 = zeros(length(N2));
  [VD2,F2,tau2] = forwardAgain(baseAcc_Frame1,Xstar2,zetta2,length(N2),betac2,Jc2);
  
  VD3 = zeros(6,length(N3));
  F3 = zeros(6,length(N3));
  tau3 = zeros(length(N3));
  [VD3,F3,tau3] = forwardAgain(baseAcc_Frame1,Xstar3,zetta3,length(N3),betac3,Jc3);
  
  VD4 = zeros(6,length(N4));
  F4 = zeros(6,length(N4));
  tau4 = zeros(length(N4));
  [VD4,F4,tau4] = forwardAgain(baseAcc_Frame1,Xstar4,zetta4,length(N4),betac4,Jc4);
  
  VD5 = zeros(6,length(N5));
  F5 = zeros(6,length(N5));
  tau5 = zeros(length(N5));
  [VD5,F5,tau5] = forwardAgain(baseAcc_Frame1,Xstar5,zetta5,length(N5),betac5,Jc5);
  
  tau = [tau1;tau2;tau3;tau4;tau5];
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

function [X,Xstar,XstarT,Jc,betac,zetaVec] = forwardBackwardOneChain(robot,N,T,qD,qDDJ,virtualFrameCounter)
baseDof = 6;
CoM_j = robot.InertialParameters.CoM;
M = robot.InertialParameters.masse; % Masas adjuntas a cada marco
I = robot.InertialParameters.I;
zHat = [0;0;1];
Nsize = length(N);
X = cell(Nsize,1);
Xstar = cell(Nsize,1);
XstarT = cell(Nsize,1);
velVec = cell(Nsize+1,1);
betaVec = cell(Nsize,1);
gammaVec = cell(Nsize,1);
zetaVec = cell(Nsize,1);
J = cell(Nsize,1);
velVec{1} = qD(1:6); %baseAcc
%% Forward pass
for i=N(1):N(end)
  X{i-N(1)+1} = VelocityMatrix(T(:,:,i));
  [Xstar{i-N(1)+1},XstarT{i-N(1)+1}] = ForceMatrix(T(:,:,i));
  velVec{i-N(1)+2} = X{i-N(1)+1}*velVec{i-N(1)+1} + [zHat*qD(i+baseDof-virtualFrameCounter);zeros(3,1)];
  betaVec{i-N(1)+1} = betaVector(velVec{i-N(1)+2}(1:3),M(i),CoM_j{i},I{i});
  gammaVec{i-N(1)+1} = gammaVector(velVec{i-N(1)+1}(1:3),zHat,qD(i+baseDof-virtualFrameCounter),T);
  zetaVec{i-N(1)+1} = gammaVec{i-N(1)+1} + [zHat*qDDJ(i-virtualFrameCounter);zeros(3,1)];
  J{i-N(1)+1} = inertiaMatrix(I{i},M(i),CoM_j{i});
end
%% Backward pass
Jc = cell(Nsize,1);
betac = cell(Nsize,1);
Jc{Nsize} = J{Nsize};
betac{Nsize} = betaVec{Nsize};
for i=(Nsize-1):-1:1
  Jc{i} = J{i} + XstarT{i+1}*Jc{i+1}*X{i+1};
  betac{i} = betaVec{i} + XstarT{i+1}*Jc{i+1}*zetaVec{i+1} + XstarT{i+1}*betac{i+1};
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

function [Xstar,XstarT] = ForceMatrix(T)
    Xstar = zeros(6,6);
    XstarT = zeros(6,6);
    R = T(1:3,1:3)';
    p = T(1:3,4);
    Xstar(1:3,1:3) = R;
    Xstar(1:3,4:6) = -R*cross_matrix(p);
    Xstar(4:6,4:6) = R;
    %
    XstarT(1:3,1:3) = R';
    XstarT(1:3,4:6) = cross_matrix(p)*R';
    XstarT(4:6,4:6) = R';
end

function J = inertiaMatrix(Ij,mj,CoM_j)
    J = zeros(6,6);
    J(1:3,1:3) = Ij;
    J(1:3,4:6) = mj*cross_matrix(CoM_j);
    J(4:6,1:3) = -mj*cross_matrix(CoM_j);
    J(4:6,4:6) = mj*eye(3,3);
end

function beta = betaVector(w,mj,CoM_j,Ij)
    beta = zeros(6,1);
    Fext = zeros(6,1); %Possible external forces
    beta(1:3) = cross_matrix(w)*Ij*w;
    beta(4:6) = cross_matrix(w)*cross_matrix(w)*mj*CoM_j;
    beta = beta + Fext;
end

function gamma = gammaVector(w,z,qD,T)
    R = T(1:3,1:3)';
    p = T(1:3,4);
    gamma = zeros(6,1);
    gamma(1:3) = R*cross_matrix(w)*z*qD;
    gamma(4:6) = R*(cross_matrix(w)*cross_matrix(w)*p);
end

function Tinv = inverseTransMatrix(T)
    Tinv = zeros(4,4);
	Tinv(1:3,1:3) = T(1:3,1:3)'; 
    Tinv(1:3,4) = -T(1:3,1:3)'*T(1:3,4);
    Tinv(4,4) = 1;
end
    