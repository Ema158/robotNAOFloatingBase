function qD = InvKinematicTest1(Qp,robot)
aux = zeros(3,30);
aux(:,4:6) = eye(3);
J = [J_state_vTest1(robot);robot.J_CoM;aux]; 
qD = J\Qp;
end

