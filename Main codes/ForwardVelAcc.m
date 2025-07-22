%Returns floating base acceleration and joint torques given joints accelerations
function accVec = ForwardVelAcc(q,qD)  
  global robot
  ant = robot.ant;
  frames = robot.frames;
  T = robot.T;
  piTi = zeros(4,4,frames);
  X10 = VelocityMatrix(T(:,:,1));
  piTi(:,:,1) = T(:,:,1);
  qD(1:6) = X10*[qD(4:6);qD(1:3)]; %[w,v]
  for i=2:frames
      piTi(:,:,i) = inverseTransMatrix(T(:,:,ant(i)))*T(:,:,i);
  end
  accVec = computeAcc(robot,piTi,qD);
end


function accVec = computeAcc(robot,T,qD)
baseDof = 6;
act = robot.act;
ant = robot.ant;
frames = robot.frames;
X = cell(frames,1);
velVec = cell(frames,1);
accVec = cell(frames,1);
velVec{1} = qD(1:6); %baseVel
gravityFactor = 1; %9.81
X{1} = VelocityMatrix(T(:,:,1));
accVec{1} = X{1}*[0;0;0;0;0;gravityFactor*9.81]; %baseAcc relative to base
S = [0;0;1;0;0;0];
%% Forward pass
for i=2:frames
    X{i} = VelocityMatrix(T(:,:,i));
    if act(i)~=0
        velVec{i} = X{i}*velVec{ant(i)} + S*qD(act(i)+baseDof);
        accVec{i} = X{i}*accVec{ant(i)} + crm(velVec{i})*S*qD(act(i)+baseDof);
    else
        velVec{i} = X{i}*velVec{ant(i)};
        accVec{i} = X{i}*accVec{ant(i)};
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

function Tinv = inverseTransMatrix(T)
    Tinv = zeros(4,4);
	Tinv(1:3,1:3) = T(1:3,1:3)'; 
    Tinv(1:3,4) = (-T(1:3,1:3)')*T(1:3,4);
    Tinv(4,4) = 1;
end

function m = crm(v)
    m = [cross_matrix(v(1:3)) zeros(3); cross_matrix(v(4:6)) cross_matrix(v(1:3))];
end

    