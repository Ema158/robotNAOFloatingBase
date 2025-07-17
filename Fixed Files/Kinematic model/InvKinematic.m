function qD = InvKinematic(Qp,robot)
%Inverse Kinematic from Q parameter variables 

aux = zeros(3,30);
aux(:,4:6) = eye(3);
J = [J_state_v(robot);robot.J_CoM;aux]; 
qD = J\Qp;

end

