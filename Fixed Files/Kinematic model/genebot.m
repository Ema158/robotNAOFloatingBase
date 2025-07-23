function robot = genebot()
%%Generation of the robot datas

%% Number of joints
joints = 30; %6 floating base, 12 legs, 10 arms, 2 head
links = 25;
frames = 28;
rotationalJoints = 24;
%% ZERO joint positions
q = zeros(joints,1); 

%% Joint velocities
qD = zeros(joints,1);

%% Creating the robot structure
robot = struct('joints',joints,'q',q,'qD',qD);
%% Robot antecedent and joint list
robot.ant = [0,... 1
             1,... 2
             2,... 3
             3,... 4
             4,... 5
             5,... 6
             6,... 7
             7,... 8
             1,... 9
             9,...10
             10,...11
             11,...12
             12,...13
             13,...14
             14,...15
             1,...16
             16,...17
             17,...18
             18,...19
             19,...20
             1,...21
             21,...22
             22,...23
             23,...24
             24,...25
             1,...26
             26,...27
             27];% 28  
robot.act = [0,1,2,3,4,5,6,0,7,8,9,...
        10,11,12,0,13,14,15,16,17,18,19,...
        20,21,22,23,24,0];
%% CONSTANT transformation matrices to convert make all matrices 0Ti be aligned with the world frame (frame 0) at ZERO position
% ------------------------------------------
robot.links = links;
robot.frames = frames;
robot.rotationalJoints = rotationalJoints;
robot.q = q;
robot.T = DGM(robot);

Tconst = zeros(4,4,28);
for i=1:27
    Tconst(:,:,i) = eye(4);
    R = robot.T(1:3,1:3,i);
    Tconst(1:3,1:3,i) = R';
end    
robot.Tconst = Tconst;

%% Assigning DEFAULT joint positions
baseDof = 6;
q = zeros(joints,1);

q(1) = -0.04;
q(2) = 0.01; %0.05
q(3) = -0.02; %0.25
q(4) = 0.2;
q(5) = 0.2;
q(6) = 0;
%
q(1+baseDof) = 0;
q(2+baseDof) = 0; 
q(3+baseDof) = -0.6; %-0.3
q(4+baseDof) = 0.7;  %0.3
q(5+baseDof) = -0.1;
q(6+baseDof) = 0;

q(7+baseDof) = 0;
q(8+baseDof) = 0;
q(9+baseDof) = -0.2; %-0.3
q(10+baseDof) = 0.8; %0.3
q(11+baseDof) = -0.6; 
q(12+baseDof) = 0;
%
q(13+baseDof) = 1.6; %1.6
q(14+baseDof) = 0;
q(15+baseDof) = 0; %1.6
q(16+baseDof) = 0; %0.2
q(17+baseDof) = 0.0;

q(18+baseDof) = -1.6; %-1.6
q(19+baseDof) = 0;
q(20+baseDof) = 0; %1.6
q(21+baseDof) = 0; %-0.2
q(22+baseDof) = 0.0;

q(23+baseDof)=0;
q(24+baseDof)=0;
robot.q = q;
robot.qdes = desiredPosture(joints);
robot.T = DGM(robot); % Modelo geométrico directo (calcula las matrices de transformación elementales del robot),
                       % y las asigna a la estructura "robot" en la variable T.                       
%% Robot mass information
InertialParameters = Mass_information();
M = InertialParameters.masse;
robot.mass = sum(M);
robot.InertialParameters = InertialParameters;
% 
[robot.CoM,robot.J_CoM,robot.J_RAnkle,robot.J_LAnkle,robot.crossM,robot.J_CoMs] = compute_comV2(robot);
robot_draw(robot)
%% RAnkle frame (8) to world frame
robot.foot_Rf = [ 0  0  1 0;
                  0 -1  0 0;
                  1  0  0 0;
                  0 0 0 1];
%% LAnkle frame (15) to world frame
robot.foot_Lf = [ 0 0 1 0;
                  0 -1 0 0;
                  1 0 0 0;
                  0 0 0 1];
%% Hip frame (1) to world frame
robot.torso_Rf=[1 0 0 0; 
               0 1 0 0;
               0 1 1 0;
               0 0 0 1];

end

