function [Jac,Jpqp] = jacobianSpatialNotation(robot,baseVel1,qD)
act = robot.act;
ant = robot.ant;
frames = robot.frames;
joints = robot.rotationalJoints;
T = robot.T;
S = [0;0;1;0;0;0];
X = cell(1,frames);
Xn = cell(1,frames);
velVec = cell(1,frames);
crossTerm = cell(1,frames);
X{1} = VelocityMatrix(T(:,:,1)); %X10
piTi(:,:,1) = T(:,:,1);
qD(1:6) = X{1}*[qD(4:6);qD(1:3)]; %[w,v]
for i=2:frames
    piTi(:,:,i) = inverseTransMatrix(T(:,:,ant(i)))*T(:,:,i);
end
baseDof = 6;
velVec{1} = baseVel1;
Jac = zeros(6,joints+baseDof);
for i=2:frames
    X{i} = VelocityMatrix(piTi(:,:,i));
    if act(i)~=0
        velVec{i} = X{i}*velVec{ant(i)} + S*qD(act(i)+baseDof);
    else
        velVec{i} = X{i}*velVec{ant(i)};
    end
end
n = 8;
Xn{n-1} = X{n};
Jac(:,act(n-1)+baseDof) = Xn{n-1}*S;
for i=(n-1):-1:2
       Xn{ant(i)} = Xn{i}*X{i}; 
       Jac(:,act(i)+baseDof) = Xn{ant(i)}*S;
end
Jac(:,1:6) = Xn{1}*X{1};
X80 = VelocityMatrix(T(:,:,8)); %X80
% Jac = inv(X80)*Jac;
end

function m = crm(v)
    m = [cross_matrix(v(1:3)) zeros(3); cross_matrix(v(4:6)) cross_matrix(v(1:3))];
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