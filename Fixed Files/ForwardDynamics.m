%Returns floating base acceleration and joint torques given joints accelerations
function [H,C] = ForwardDynamics(q,qD)  
  global robot
  ant = robot.ant;
  frames = robot.frames;
  joints = robot.rotationalJoints;
  T = robot.T;
  piTi = zeros(4,4,frames);
  X10 = VelocityMatrix(T(:,:,1));
  piTi(:,:,1) = T(:,:,1);
  qD(1:6) = X10*[qD(4:6);qD(1:3)]; %[w,v]
  for i=2:frames
      piTi(:,:,i) = inverseTransMatrix(T(:,:,ant(i)))*T(:,:,i);
  end
  [X,J,f,CJ,p1c] = computeC(robot,piTi,qD);
  C = [p1c;CJ];
  [HJ,Ic,F2] = computeH(robot,X,J);
  H = zeros(joints+6,joints+6);
  H(1:6,1:6) = Ic{1};
  H(7:end,7:end) = HJ;
  H(1:6,7:end) = F2;
  H(7:end,1:6) = F2';
end


function [X,J,f,C,p1c] = computeC(robot,T,qD)
baseDof = 6;
act = robot.act;
ant = robot.ant;
frames = robot.frames;
joints = robot.rotationalJoints;
CoM_j = robot.InertialParameters.CoM;
M = robot.InertialParameters.masse; % Masas adjuntas a cada marco
I = robot.InertialParameters.I;
X = cell(frames,1);
f = cell(frames,1);
J = cell(frames,1);
velVec = cell(frames,1);
accVec = cell(frames,1);
velVec{1} = qD(1:6); %baseVel
gravityFactor = 1; %9.81
X{1} = VelocityMatrix(T(:,:,1));
accVec{1} = X{1}*[0;0;0;0;0;gravityFactor*9.81]; %baseAcc relative to base
J{1} = inertiaMatrix(I{1},M(1),CoM_j{1});
f{1} = J{1}*accVec{1} + crf(velVec{1})*J{1}*velVec{1};
S = [0;0;1;0;0;0];
%% Forward pass
for i=2:frames
    if i==9
        ema=1;
    end
    if act(i)~=0
        X{i} = VelocityMatrix(T(:,:,i));
        velVec{i} = X{i}*velVec{ant(i)} + S*qD(act(i)+baseDof);
        accVec{i} = X{i}*accVec{ant(i)} + crm(velVec{i})*S*qD(act(i)+baseDof);
        J{i} = inertiaMatrix(I{i},M(i),CoM_j{i});
        f{i} = J{i}*accVec{i} + crf(velVec{i})*J{i}*velVec{i};
    end
end
%% Backward pass
C = zeros(joints,1);
for i=frames:-1:2
  if act(i)~=0
      C(act(i)) = (S')*f{i};
      f{ant(i)} = f{ant(i)} + X{i}'*f{i};
  end
end
p1c = f{1};
end

function [H,Ic,F2] = computeH(robot,X,J)
    act = robot.act;
    ant = robot.ant;
    frames = robot.frames;
    joints = robot.rotationalJoints;
    Ic = cell(frames,1);
    F = cell(frames,1);
    H = zeros(joints,joints);
    S = cell(frames,1);
    F2 = zeros(6,joints);
    for i=1:frames
       Ic{i} = J{i}; 
       if i==1
           S{1} = eye(6,6);
       else
           S{i} = [0;0;1;0;0;0];
       end
    end
    for i=frames:-1:1
        if act(i)~=0 
            Ic{ant(i)} = Ic{ant(i)} + X{i}'*Ic{i}*X{i};
            F{i} = Ic{i}*S{i};
            H(act(i),act(i)) = S{i}'*F{i};
            j = i;
            while ant(j)~=1
               F{i} = X{j}'*F{i}; 
               j = ant(j);
               H(act(j),act(i)) = S{j}'*F{i};
               H(act(i),act(j)) = H(act(j),act(i));
            end
            F2(:,act(i)) = X{j}'*F{i};
        end
        
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
    