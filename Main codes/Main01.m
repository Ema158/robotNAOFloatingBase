close all;
clear all
clc
tic; % Start the timer
% ---------------------------------------------------------------------------------------------
currentfolder = pwd; % save current path
cd ..                % go one folder down (go out of the current folder and stay in the previous one)
add_paths;           % add all folders where all files are founded in order to acces to them
cd(currentfolder);   % return to the original path
% ---------------------------------------------------------------------------------------------

echo on
% Testing Essential dynamics standing on one foot
% ==============================
% Creation: 28/Ago/2024
% Last modification: -/--/--
% --------------------------------------------------------------------------------------------------
% Robot stands in one foot and regulates 
% -----------------------------------------------------------------------------------------------------
% --------------------------------------------------------------------------------------------------
% 
%
echo off

global gait_parameters
global robot coms
coms=1;

% Parameters
% ---------------------------------------------------------
Nao_param = ParamTest1();
% ---------------------------------------------------------
gait_parameters = Nao_param.gait_parameters;
% 
% T = gait_parameters.T;
% g = gait_parameters.g;
% z0 = gait_parameters.z_i;
% S = gait_parameters.S;
% D = gait_parameters.y_ffoot_i;

% CHOSING CONTROLLED VARIABLE FILES
% -------------------------------------------------------------------------------------------------
global OptionContVar  % Option to chose the controlled variables "hd", "hpd" and "hppd"
% 1 -> Controlled variables defined by polynomials w.r.t. time, IMPACT is considered. Files: "hd_Polyn", "hpd_Polyn_t" and "hppd_Polyn_t". 
% 2 -> Controlled variables defined by cycloidal motion w.r.t. time, IMPACT is NOT considered. Files: "hd_CycMotion_t", "hpd_CycMotion_t" and "hppd_CycMotion_t". 
% 3 -> Controlled variables defined by polynomials w.r.t. "x" of the CoM, IMPACT is considered. Files: "hd_Polyn", "hpd_Polyn_x" and "hppd_Polyn_x". 
OptionContVar = Nao_param.ControlledVariableOption;
% -------------------------------------------------------------------------------------------------

% GENERAL OPTIONS
% ----------------------------------------------------
DataName = 'InfoNAO_FloatingBaseTest1';
% DataName = 'InfoNAO_OutCoMWebots';
robot = genebot();

gait_parameters.transition = false; 
PolyCoeff = Coeff_DesiredTrajectories_t_ver2(robot,gait_parameters);
gait_parameters.PolyCoeff = PolyCoeff;
%Initial conditions
X0 = [0;0.05;0.28;0;0;0;... %x,y,z,roll,pitch,yaw
      0;0;0;0;0;0]; %xp,yp,zp,rollp,pitchp,yawp
[t,Xt] = EssModelFloatingBaseTest1(X0,gait_parameters);
%% End of the code
% ----------------------------------------------------------------------------------------------
cd ..                  % Go down one folder (go out from the current folder and stay in the previous one)
remove_paths;          % Remove the paths of the folders added at the begining
cd(currentfolder);     % Return to the original folder
% ----------------------------------------------------------------------------------------------
