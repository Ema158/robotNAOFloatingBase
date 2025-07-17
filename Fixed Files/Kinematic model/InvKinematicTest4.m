function qD = InvKinematicTest4(Qp,robot)
aux = zeros(3,30);
aux(:,4:6) = eye(3);
J = [J_state_vTest4(robot);robot.J_CoM;aux]; 
qD = J\Qp;
end

