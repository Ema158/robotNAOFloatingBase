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
DataName = 'InfoNAO_FloatingBaseTest2';
% robot = genebot();
% time_step = 0.01;
% T = 4;
% samples = T/time_step;
% current_time = 0;
% X0 = zeros(60,1);
% X0(1:30) = robot.q; %[pB,etaB,qJ]
% X0(31:60) = zeros(30,1); %[vB,wB,qDJ]
% Xt = zeros(samples+1,length(X0));
% Xt(1,:) = X0';
% for i=1:samples
%     timespan = [current_time, current_time + time_step];
%     Xtaux = ode4(@FloatingBaseSimulation,timespan,X0);
%     Xt(i+1,:) = Xtaux(end,:);
%     X0 = Xtaux(end,:)';
%     current_time = current_time + time_step;
% end

robot = genebot();
time_step = 0.01;
T = 5;
samples = T/time_step;
current_time = 0;
X0 = zeros(60,1);
X0(1:30) = robot.q; %[pB,etaB,qJ]
X0(31:60) = zeros(30,1); %[vB,wB,qDJ]
Xt = zeros(samples+1,length(X0));
Xt(1,:) = X0';
for i=1:samples
    timespan = [current_time, current_time + time_step];
    Xtaux = ode4(@FloatingBaseSimulationV2,timespan,X0);
    Xt(i+1,:) = Xtaux(end,:);
    X0 = Xtaux(end,:)';
    current_time = current_time + time_step;
end
%% End of the code
% ----------------------------------------------------------------------------------------------
cd ..                  % Go down one folder (go out from the current folder and stay in the previous one)
remove_paths;          % Remove the paths of the folders added at the begining
cd(currentfolder);     % Return to the original folder
% ----------------------------------------------------------------------------------------------
