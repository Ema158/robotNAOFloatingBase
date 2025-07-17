function J_h = J_state_vTest1(robot)  
%Computation of the jacobian of h function for virtual constraint
baseDof = 6;
J_h = zeros(robot.joints-baseDof,robot.joints);
%% Right leg
J_h(1:6,1+baseDof:6+baseDof) = eye(6,6);
%% Left leg
J_h(7:12,7+baseDof:12+baseDof) = eye(6,6);
%% Arms and head
J_h(13:22,13+baseDof:22+baseDof) = eye(10,10);
J_h(23:24,23+baseDof:24+baseDof) = eye(2,2);
end

