function qppRef =  PDreferenceAcceleration(q,qp)
global robot
qdes = robot.qdes;
n = length(q);
Kp = 10*eye(n,n);
Kd = 6.32*eye(n,n);
qppRef = Kp*(q-qdes) + Kd*(qp-zeros(n,1));
end