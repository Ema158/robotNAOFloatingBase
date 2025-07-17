function qD = InvKinematicTest2(Qp,robot)
aux = zeros(3,30);
aux(:,4:6) = eye(3);
J = [J_state_vTest2(robot);robot.J_CoM;aux]; 
qD = J\Qp;
end

