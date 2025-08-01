function hGd =  PDreferenceMomentumRate(AG,qD)
cpdes = zeros(3,1);
kdes = zeros(3,1);
cdes = [0.05;0.00;0.03];
hGd = zeros(6,1);
global robot
m = robot.mass;
c = robot.CoM;
T = robot.T;
X{1} = VelocityMatrix(T(:,:,1)); %X10
qD(1:6) = X{1}*[qD(4:6);qD(1:3)]; %[w,v]
momentum = AG*qD;
cp = momentum(4:6)/m;
k = momentum(1:3);
Kpl = 10*eye(3,3);
Kdl = 6.32*eye(3,3);
Kdk = Kdl;
hGd(4:6) = m*(Kpl*(cdes-c) + Kdl*(cpdes-cp));
hGd(1:3) = Kdk*(kdes-k);
end

function X = VelocityMatrix(T)
    X = zeros(6,6);
    R = T(1:3,1:3)';
    p = T(1:3,4);
    X(1:3,1:3) = R;
    X(4:6,1:3) = -R*cross_matrix(p);
    X(4:6,4:6) = R;
end