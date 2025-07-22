function qppRef =  PDreferenceAcceleration(q,qp)
global robot
qdes = robot.qdes;
n = length(q);
Kp = 10*eye(n,n);
Kd = 6.32*eye(n,n);
qppRef = Kp*(qdes-q) + Kd*(zeros(n,1)-qp); %q = [0pB,0etaB,qJ] %qD = [0vB,0wB,qjD]
qppRef = [qppRef(4:6);qppRef(1:3);qppRef(7:end)];
end