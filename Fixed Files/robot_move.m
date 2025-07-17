%global robot
function robot=robot_move(robot,q)
%New joints configuration of the robot
robot.q = q;
robot.T = DGM(robot);
[robot.CoM,robot.J_CoM,robot.J_RAnkle,robot.J_LAnkle,robot.crossM,robot.J_CoMs] = compute_comV2(robot);
end

