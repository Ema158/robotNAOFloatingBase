function [CoM,J_CoM,J_RAnkle,J_LAnkle,crossM,J_CoMs] = compute_comV2(robot)

%% CoM position/velocity

%Mass informations
M = robot.InertialParameters.masse;
CoM_j = robot.InertialParameters.CoM;
%///////////////////////////////////////////////////////////////////////////
%Initialisations
CoM = [0;0;0];
J_CoMs = zeros(3,robot.joints,28);
J_CoM = zeros(3,robot.joints);
T = robot.T;
ant = robot.ant;  
act= robot.act;
crossM = zeros(3,3,28);

%Computation of the cross matrices
for i = 2:28
    crossM(:,:,i) = cross_matrix(T(1:3,3,i)); % Transforma el vector "a" de la matriz T = [s n a p; 0 0 0 1]; en una matriz antisimétrica
end
MS_b = T(1:3,:,1)*[CoM_j{1};1];
Jaux = baseJacobian(T(:,:,1),MS_b);
Jb = Jaux(4:6,:);
Jb = M(1)*Jb;
J_CoM(1:3,1:6) = Jb;
CoM = CoM + M(1)*MS_b;
for j = 2 : 28
    if M(j) ~=0
        %Center of mass position (X,Y)
        MS_j = T(1:3,:,j) * [CoM_j{j};1]; % Este es el vector de posicion del CoM_j respecto al marco 0
        CoM = CoM + M(j) * MS_j;         % De igual forma, aquí se va calculando el CoM total del robot "Sum(j=1:36) m* ^0Com_j"
        %Center of mass velocity (XD,YD)
        J_X = zeros(3,robot.joints);  
        Jaux = baseJacobian(T(:,:,1),MS_j);
        J_X(1:3,1:6) = Jaux(4:6,:);
        F = j;% Se empiezan a crear los jacobianos para cada marco( los que no tienen masa asignada seran una matriz de ceros)
        while F~=1    % ¿Llegamos al marco 1? Si si, sale del "while", si no, continúa... (F nunca es menor que 1)
            % Aqui se va calculando cada columna de la matriz J_CoM_j. Nótese que se "brinca" las columnas que
            % corresponden a los marcos que NO tienen asignada una masa.. Nótese que los jacobianos empiezan de ADELANTE pa'trás
            if act(F)~=0 % ¿El marco F tiene una articulacion (q)? Si si, entra al "if", si no no =P
               J_X(:,act(F)-1+6) =crossM(:,:,F)*(MS_j-T(1:3,4,F)); %col  -> 0a_j X (0Pcom_j - 0P_j)
            end
            F = ant(F);
        end
        J_CoMs(:,:,j)=J_X;               % Se asigna 
        J_CoM = J_CoM+M(j)*J_X;
     end
end

CoM = CoM / robot.mass;   % = 1/mT Sum_{i=1}^n a_j X ^jCom_j   donde  jCom_j es el vector constante de CoM del del eslabon j 
                          % respecto al marco j, y a_j es el vector de la matriz "snap" del marco "j"
J_CoM = J_CoM/robot.mass;

%% J_Ankle is the jacobian of the ankle for x,y position and leg tip for z
% position
F = 8;
J_RAnkle = zeros(6,robot.joints);
Jb_RAnkle = baseJacobian(T(:,:,1),T(1:3,4,8));
J_RAnkle(1:6,1:6) = Jb_RAnkle;
while F~=1
        if act(F)~=0 
            J = crossM(:,:,F)*(T(1:3,4,8)-T(1:3,4,F));
            J_RAnkle(1:3,act(F)+6) = J(1:3);
            J_RAnkle(4:6,act(F)+6) = T(1:3,3,F);
        end
        F = ant(F);
end

F = 15;
J_LAnkle = zeros(3,robot.joints);
Jb_LAnkle = baseJacobian(T(:,:,1),T(1:3,4,15));
J_LAnkle(1:6,1:6) = Jb_LAnkle;
while F~=1
        if act(F)~=0 
            J = crossM(:,:,F)*(T(1:3,4,15)-T(1:3,4,F));
            J_LAnkle(1:3,act(F)+6) =J(1:3);
            J_LAnkle(4:6,act(F)+6) =T(1:3,3,F);
        end
        F = ant(F);
end

end

function Jb = baseJacobian(Tbase,pi)
    Jb = zeros(6,6);
    Jb(1:3,1:3) = eye(3,3);
    Jb(4:6,4:6) = eye(3,3);
    basePosition = Tbase(1:3,4);
    Jb(1:3,4:6) = cross_matrix(basePosition-pi);
end










