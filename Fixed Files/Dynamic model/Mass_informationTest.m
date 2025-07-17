% If you want to fold the code for each section in YOUR MATLAB go to
%    Editor/Debugger (MATLAB Preferences) > Code Folding and enable "Sections" by checking on it
function PI = Mass_informationTest()
%% Version of the  23 April 2025

%% I.   Masses - Center of Masses - Inertia Tensors
%  ============================================
% 
% The centers of masses positions and the inertial tensors are described
% relatively to the Aldebaran's local coordinate system of the current 
% solid (S) (o, R).
% The inertia tensors are expressed around the mass center. 
% i.e. ^SI0_i where S is a local fixed frame of Aldebaran and I0 is the
% inertia tensor
% 
% All solids (S) and Aldebaran's local coordinate system are described 
% relatively to the zero posture: standing with straight legs and arms 
% pointing forwards).
% 
% Masses are expressed in kg
% Estos valores fueron tomados de la documenacion de aldebaran
%http://doc.aldebaran.com/2-1/family/robots/masses_robot.html
M = zeros(8,1);

% CoM = [X_G ; Y_G ; Z_G]; kg/m²
%
% I= [I_{xx} , I_{xy} , I_{xz} ;
%     I_{yx} , I_{yy} , I_{yz} ;
%     I_{zx} , I_{zy} , I_{zz}]; kg/m²

%% Trunk
% Torso
% ------
M(1)   = 1.0496;

CoM{1} = zeros(3,1);

I{1}          = eye(3,3);
			
% Legs
% Rigth Pelvis
% RHipYawPitch
M(2) = 0.06981;

CoM{2} = zeros(3,1);

I{2}= eye(3,3);    
     
%% Right Hip          
% RHipRoll
%----------------
M(3) = 0.14053;

CoM{3} = zeros(3,1);

I{3}= eye(3,3);
	
% Right Thigh
% RHipPitch
%------------
M(4) = 0.38968;

CoM{4} = zeros(3,1);

I{4}= eye(3,3);
	
% Right Tibia
% RKneePitch
%------------
M(5) = 0.30142;

CoM{5} = zeros(3,1);

I{5}= eye(3,3);
	
% Right Ankle
% RAnklePitch
%-------------
M(6) = 0.13416;

CoM{6} = zeros(3,1);

I{6}= eye(3,3);     
%Right Foot    
% RAnkleRoll
%---------
M(7) = 0.17184;

CoM{7} = zeros(3,1);

I{7}= eye(3,3); 
 %--------------------- virtual frame in the tip of the left foot
M(8) = 0;
CoM{8} = zeros(3,1);
I{8}= zeros(3,3);
%% III. Transformation matrices w.r.t frame 0 (Tip of the RFoot)
joints = 12;
q = zeros(joints,1); 
qD = zeros(joints,1);
robot = struct('joints',joints,'q',q,'qD',qD);
T0=DGM(robot);
  
%% IV.  Calculation of I. in M-DH frames

% Base change I_j = inv(P_j) I_j_s P_j

% Transport I_j = I_j_s + M Y_s
% with Y_s = [b^2+c^2,-a*b,-a*c;
%             -a*b,c^2+a^2,-b*c;
%             -a*c,-b*c,a^2+b^2]:
% Knowing CoM = [a;b;c];

% Transport of the CoM 
% d_xg = d_x - d_x_0
% d_yg = d_y - d_y_0
% d_zg = d_z - d_z_0

% % Rotation matrices 
P = cell(1,8);
for i = 1:8
	P{i} = inv(T0(1:3,1:3,i));  % P = jR0
end
PI.IG1 = I{1};  
for i = 1 : 8
    inter = CoM{i};
    CoM{i} = P{i}*inter; 

    Y = [inter(2)^2+inter(3)^2,-inter(1)*inter(2),-inter(1)*inter(3);
     -inter(1)*inter(2),inter(3)^2+inter(1)^2,-inter(2)*inter(3);
     -inter(1)*inter(3),-inter(2)*inter(3),inter(1)^2+inter(2)^2];

    I{i} = I{i} + M(i) * (Y);   % application of translation  % SIj = bIb + Mj*Yj  "S" is the Aldebaran frame 
    I{i} = P{i} * I{i} * P{i}'; % application of rotation % jIj = jRb bIj bRj 
end

% % ==============================================================================================
% % The NEXT part of the code is added in order to concentrate the mass of the robot in one point
% % ==============================================================================================
global alphaM % alphaM = [0-1]. Allow to remove gradually the value of all masses and inertias of the robot
% alphaM = 0 -> all masses are 0 -> all mass is concentrated in one point
% alphaM = 1 -> all masses are normal -> all masses are distributed
% If alphaM is empty it means it was not initialized by other codes, therefore it is not required to do this 
if ~isempty(alphaM) 
    if alphaM > 1
        alphaM = 1;
    elseif alphaM < 0
        alphaM = 0;
    end
    
    MassTorso = 0;
    for i = 1:7
        I{i} = alphaM*I{i};
        MassTorso = MassTorso + (1-alphaM)*M(i);
        M(i) = alphaM*M(i);
    end
    
    % % Assignment of the masss to the frame 15 (hip),
    M(7)= M(7) + MassTorso;% Rest of the mass asigned to frame 15
    % If less than 20% of the mass is concentrated in the frame 15 the same CoM position (of frame 15) will be used, but...
    if alphaM < 0.75
        CoM{7} = [0; 0.015; -0.015]; % we change the CoM position of frame 15 if more than 20% of the mass are concentred in this frame...
    end
end
% % ==============================================================================================
PI.masse = M;
PI.CoM = CoM; % ^jCoM, it is the CoM of link j w.r.t. frame j
PI.I = I; %  ^jIj i.e. is the inertia tensor of link j, located in the origin of frame j

 