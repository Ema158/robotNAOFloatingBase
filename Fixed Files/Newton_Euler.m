function [F0,M0,Tau] = Newton_Euler(q,qD,qDD)
baseDof = 6;
%NEWTON_EULER Algorithm to compute torques (model single mass)
PI=Mass_information();
%init;
% g=9.81;
g=0;
Ia = actuator_inertia();
Tau = zeros(24,1);
frame=[2,3,4,5,6,7,9,10,11,12,13,14,16,17,18,19,20,21,22,23,24,25,26,27];

tree1 =[1,2,3,4,5,6];% Right Leg
tree2 =[7,8,9,10,11,12];% Left Leg
tree3 = [13,14,15,16,17];% Right arm
tree4 = [18,19,20,21,22];% Left arm
tree5 = [23,24]; % Head

M= PI.masse;
I =PI.I;
IG1 = PI.IG1;
COM = PI.CoM;
% ============
robot=genebot;
robot.q=q;
T=DGM(robot);
ft = zeros(3,27);
mt = zeros(3,27);
%%
v1 = q(1:3);
RPY = q(4:6);
RPYp = qD(4:6);
w1 = inv(OmeRPY(RPY(1),RPY(2)))*qD(4:6);
v1D = qDD(1:3);
w1D = inv(OmeRPY(RPY(1),RPY(2)))*qD(4:6) + OmeInvDotRPY(RPY(1),RPY(2),RPYp(1),RPYp(2))*qD(4:6);

R=T(1:3,1:3,1);
O1Oc1 = R*COM{1};
vc1 = v1 + cross_matrix(w1)*O1Oc1;
vc1D = qDD(1:3) + cross_matrix(inv(OmeRPY(RPY(1),RPY(2)))*qDD(4:6) + OmeInvDotRPY(RPY(1),RPY(2),RPYp(1),RPYp(2))*qD(4:6))*O1Oc1...
    + cross_matrix(inv(OmeRPY(RPY(1),RPY(2)))*qD(4:6))*cross_matrix(inv(OmeRPY(RPY(1),RPY(2)))*qD(4:6))*O1Oc1;
%Initialisation
omei = w1;
omeDi = w1D;
vDi = v1D+[0;0;g];
ft(:,1) = M(1)*vc1;
mt(:,1) = (R*IG1*R')*(inv(OmeRPY(RPY(1),RPY(2)))*qDD(4:6) + OmeInvDotRPY(RPY(1),RPY(2),RPYp(1),RPYp(2))*qD(4:6))+...
    cross_matrix(inv(OmeRPY(RPY(1),RPY(2)))*qD(4:6))*(R*IG1*R')*inv(OmeRPY(RPY(1),RPY(2)))*qD(4:6) + cross_matrix(O1Oc1)*ft(:,1);

%Tree 1 Initialisation
ome =omei;
omeD = omeDi;
vD = vDi;
tree=tree1;

%Forward algorithm
% =========================================
for i=1:numel(tree)
    if i~=1
    L = T(1:3,4,frame(tree(i)))-T(1:3,4,frame(tree(i-1))); % 0L = 0Pi - OP{i-1}
    else
    L = T(1:3,4,frame(tree(i)))-T(1:3,4,1);  % 0L = 0P2 - OP1
    end
    % Note que en las siguientes 2 ecuaciones se usan los valores anteriores de w, i.e "w_{i-1}" luego en la 3era ecuacion de
    % calcula el nuevo w, i.e "w_i"
    % 0vp = 0vp_a  +  0wp X 0L  +  0w x 0w x 0L
    vD = vD + cross_matrix(omeD)*L+cross_matrix(ome)*(cross_matrix(ome)*L);
    % 0wp = 0wp_a  +  0a*qpp  +  0w x 0a*qp 
    omeD = omeD + T(1:3,3,frame(tree(i)))*qDD(tree(i)+baseDof-1)+cross_matrix(ome)*T(1:3,3,frame(tree(i)))*qD(tree(i)+baseDof-1);
    % w = w_a + qp*0a
    ome = ome + qD(tree(i)+baseDof-1)*T(1:3,3,frame(tree(i)));

    R=T(1:3,1:3,frame(tree(i)));  % R = 0^R_{i}
    S = R*COM{frame(tree(i))}; % S = 0Pcom = 0^R_i i^Pcom
    % 0Finercial = m[0vp  +  0wp x 0Pcom  +  0w x 0w x 0Pcom]
    ft(:,tree(i)) = M(frame(tree(i)))*(vD+cross_matrix(omeD)*S + cross_matrix(ome)*(cross_matrix(ome)*S));
    % 0Minercial = m*0Pcom X 0vp  +  0Ij*wp  +  w x 0Ij x ome    
    mt(:,tree(i))= M(frame(tree(i)))*cross_matrix(S)*vD + R*I{frame(tree(i))}*R'*omeD + cross_matrix(ome)*R*I{frame(tree(i))}*R'*ome;
end

%Tree 2 Initialisation
ome =omei;
omeD = omeDi;
vD = vDi;
tree = tree2;

%Forward algorithm
for i=1:numel(tree)
    if i~=1
    L = T(1:3,4,frame(tree(i)))-T(1:3,4,frame(tree(i-1)));
    else
    L = T(1:3,4,frame(tree(i)))-T(1:3,4,1);
    end
    vD = vD + cross_matrix(omeD)*L+cross_matrix(ome)*(cross_matrix(ome)*L);
    omeD = omeD+T(1:3,3,frame(tree(i)))*qDD(tree(i)+baseDof-2)+cross_matrix(ome)*T(1:3,3,frame(tree(i)))*qD(tree(i)+baseDof-2);
    ome = ome + qD(tree(i)+baseDof-2)*T(1:3,3,frame(tree(i)));

    R=T(1:3,1:3,frame(tree(i)));
    S = R*COM{frame(tree(i))};
    ft(:,tree(i)) = M(frame(tree(i)))*(vD+cross_matrix(omeD)*S+cross_matrix(ome)*(cross_matrix(ome)*S));
    mt(:,tree(i))= M(frame(tree(i)))*cross_matrix(S)*vD+R*I{frame(tree(i))}*R'*omeD+cross_matrix(ome)*R*I{frame(tree(i))}*R'*ome;
end

%Tree 3 Initialisation
ome =omei;
omeD = omeDi;
vD = vDi;
tree=tree3;

%Forward algorithm
for i=1:numel(tree)
    if i~=1
    L = T(1:3,4,frame(tree(i)))-T(1:3,4,frame(tree(i-1)));
    else
    L = T(1:3,4,frame(tree(i)))-T(1:3,4,1);
    end
    vD = vD + cross_matrix(omeD)*L+cross_matrix(ome)*(cross_matrix(ome)*L);
    omeD = omeD+T(1:3,3,frame(tree(i)))*qDD(tree(i)+baseDof-3)+cross_matrix(ome)*T(1:3,3,frame(tree(i)))*qD(tree(i)+baseDof-3);
    ome = ome + qD(tree(i)+baseDof-3)*T(1:3,3,frame(tree(i)));
    R=T(1:3,1:3,frame(tree(i)));
    S = R*COM{frame(tree(i))};
    ft(:,tree(i)) = M(frame(tree(i)))*(vD+cross_matrix(omeD)*S+cross_matrix(ome)*(cross_matrix(ome)*S));
    mt(:,tree(i))= M(frame(tree(i)))*cross_matrix(S)*vD+R*I{frame(tree(i))}*R'*omeD+cross_matrix(ome)*R*I{frame(tree(i))}*R'*ome;
end

%Tree 4 Initialisation
ome =omei;
omeD = omeDi;
vD = vDi;
tree=tree4;

%Forward algorithm
for i=1:numel(tree)
    if i~=1
    L = T(1:3,4,frame(tree(i)))-T(1:3,4,frame(tree(i-1)));
    else
    L = T(1:3,4,frame(tree(i)))-T(1:3,4,1);
    end
    vD = vD + cross_matrix(omeD)*L+cross_matrix(ome)*(cross_matrix(ome)*L);
    omeD = omeD+T(1:3,3,frame(tree(i)))*qDD(tree(i)+baseDof-3)+cross_matrix(ome)*T(1:3,3,frame(tree(i)))*qD(tree(i)+baseDof-3);
    ome = ome + qD(tree(i)+baseDof-3)*T(1:3,3,frame(tree(i)));
    R=T(1:3,1:3,frame(tree(i)));
    S = R*COM{frame(tree(i))};
    ft(:,tree(i)) = M(frame(tree(i)))*(vD+cross_matrix(omeD)*S+cross_matrix(ome)*(cross_matrix(ome)*S));
    mt(:,tree(i))= M(frame(tree(i)))*cross_matrix(S)*vD+R*I{frame(tree(i))}*R'*omeD+cross_matrix(ome)*R*I{frame(tree(i))}*R'*ome;
end

%Tree 5 Initialisation
ome =omei;
omeD = omeDi;
vD = vDi;
tree = tree5;

%Forward algorithm
for i=1:numel(tree)
    if i~=1
    L = T(1:3,4,frame(tree(i)))-T(1:3,4,frame(tree(i-1)));
    else
    L = T(1:3,4,frame(tree(i)))-T(1:3,4,1);
    end
    vD = vD + cross_matrix(omeD)*L+cross_matrix(ome)*(cross_matrix(ome)*L);
    omeD = omeD+T(1:3,3,frame(tree(i)))*qDD(tree(i)+baseDof-3)+cross_matrix(ome)*T(1:3,3,frame(tree(i)))*qD(tree(i)+baseDof-3);
    ome = ome + qD(tree(i)+baseDof-3)*T(1:3,3,frame(tree(i)));
    R=T(1:3,1:3,frame(tree(i)));
    S = R*COM{frame(tree(i))};
    ft(:,tree(i)) = M(frame(tree(i)))*(vD+cross_matrix(omeD)*S+cross_matrix(ome)*(cross_matrix(ome)*S));
    mt(:,tree(i))= M(frame(tree(i)))*cross_matrix(S)*vD+R*I{frame(tree(i))}*R'*omeD+cross_matrix(ome)*R*I{frame(tree(i))}*R'*ome;
end

%% Backward algorithm 
% =========================================
f =[0;0;0];
m = [0;0;0];

%Tree 5
tree = tree5;
for i = numel(tree):-1:1
    if i == numel(tree)
        L=[0;0;0];
    else
        L = T(1:3,4,frame(tree(i+1)))-T(1:3,4,frame(tree(i)));
    end
    % 0M = 0Minercial + 0M_{i+1} + 0L x 0F
    m = mt(:,tree(i))+m+cross_matrix(L)*f;
    % 0F = 0Finercial + 0F_{i+1}
    f = ft(:,tree(i))+f;
    % Las 3 componentes del par estarían dadas por ^iTau = ^iR0 0M, sin embargo ya que el par solo se aplica en "z" sólo se
    % calcula el ultimo rengon, i.e. ^iTau_z = ^iR0(i,:) 0M -> [0ax 0ay 0az]*[0Mx 0My 0Mz]
    Tau(tree(i))=m'*T(1:3,3,frame(tree(i)))+Ia(tree(i)+baseDof-3)*qDD(tree(i)+baseDof-3); % Así estaba... + Ia(i)*qDD(i); pero así no es por que no toma las inercias de los actuadores superiores
    % Ésto sería lo mismo:
    % Par = T(1:3,1:3,frame(tree(i)))'*m; % iR0*0M y luego se toma el 3er elemento
    % Tau(tree(i))= Par(3) + Ia(i)*qDD(i);  % iTau = iMz + iIa*qpp
    % NOTE que el par "tau" está expresado en su PROPIO MARCO
end
f_temp5 = f;
m_temp5 = m;

f =[0;0;0];
m = [0;0;0];

%Tree 4
tree = tree4;
for i = numel(tree):-1:1
    if i == numel(tree)
        L=[0;0;0];
    else
        L = T(1:3,4,frame(tree(i+1)))-T(1:3,4,frame(tree(i)));
    end
    % 0M = 0Minercial + 0M_{i+1} + 0L x 0F
    m = mt(:,tree(i))+m+cross_matrix(L)*f;
    % 0F = 0Finercial + 0F_{i+1}
    f = ft(:,tree(i))+f;
    % Las 3 componentes del par estarían dadas por ^iTau = ^iR0 0M, sin embargo ya que el par solo se aplica en "z" sólo se
    % calcula el ultimo rengon, i.e. ^iTau_z = ^iR0(i,:) 0M -> [0ax 0ay 0az]*[0Mx 0My 0Mz]
    Tau(tree(i))=m'*T(1:3,3,frame(tree(i)))+Ia(tree(i)+baseDof-3)*qDD(tree(i)+baseDof-3); % Así estaba... + Ia(i)*qDD(i); pero así no es por que no toma las inercias de los actuadores superiores
    % Ésto sería lo mismo:
    % Par = T(1:3,1:3,frame(tree(i)))'*m; % iR0*0M y luego se toma el 3er elemento
    % Tau(tree(i))= Par(3) + Ia(i)*qDD(i);  % iTau = iMz + iIa*qpp
    % NOTE que el par "tau" está expresado en su PROPIO MARCO
end
f_temp4 = f;
m_temp4 = m;

%Tree 3
f = [0;0;0];
m = [0;0;0];

tree = tree3;
for i = numel(tree):-1:1
    if i == numel(tree)
        L=[0;0;0];
    else
        L = T(1:3,4,frame(tree(i+1)))-T(1:3,4,frame(tree(i)));
    end
    m = mt(:,tree(i))+m+cross_matrix(L)*f;
    f = ft(:,tree(i))+f;    
    Tau(tree(i))=m'*T(1:3,3,frame(tree(i)))+Ia(tree(i))*qDD(tree(i)+baseDof-3); % Así estaba... + Ia(i)*qDD(i); pero así no es 
    % Ésto sería lo mismo:
    % Par = T(1:3,1:3,frame(tree(i)))'*m; % iR0*0M y luego se toma el 3er elemento
    % Tau(tree(i))= Par(3) + Ia(i)*qDD(i);  % iTau = iMz + iIa*qpp
end
f_temp3 = f;
m_temp3 = m;

%Tree 2
f =[0;0;0];
m = [0;0;0];
tree = tree2;
for i = numel(tree):-1:1
    if i == numel(tree)
        L=[0;0;0];
    else
        L = T(1:3,4,frame(tree(i+1)))-T(1:3,4,frame(tree(i)));
    end
    m = mt(:,tree(i))+m+cross_matrix(L)*f;
    f = ft(:,tree(i))+f;
    Tau(tree(i))=m'*T(1:3,3,frame(tree(i)))+Ia(tree(i))*qDD(tree(i)+baseDof-2); % Así estaba... + Ia(i)*qDD(i); pero así no es 
    % Ésto sería lo mismo:
    % Par = T(1:3,1:3,frame(tree(i)))'*m; % iR0*0M y luego se toma el 3er elemento
    % Tau(tree(i))= Par(3) + Ia(i)*qDD(i);   % iTau = iMz + iIa*qpp
end
f_temp2= f;
m_temp2 = m;

%Tree 1
f =[0;0;0];
m = [0;0;0];
tree = tree1;
for i = numel(tree):-1:1
    if i == numel(tree)
        L=[0;0;0];
    else
        L = T(1:3,4,frame(tree(i+1)))-T(1:3,4,frame(tree(i)));
    end
    m = mt(:,tree(i))+m+cross_matrix(L)*f;
    f = ft(:,tree(i))+f;
    Tau(tree(i))=m'*T(1:3,3,frame(tree(i)))+Ia(tree(i))*qDD(tree(i)+baseDof-1); % Así estaba... + Ia(i)*qDD(i); pero así no es 
    % Ésto sería lo mismo:
    % Par = T(1:3,1:3,frame(tree(i)))'*m; % iR0*0M y luego se toma el 3er elemento
    % Tau(tree(i))= Par(3) + Ia(i)*qDD(i);   % iTau = iMz + iIa*qpp
end
f_temp1= f;
m_temp1 = m;

%Base
f = ft(:,1) + f_temp1 + f_temp2 + f_temp3 + f_temp4 + f_temp5;

O1O2 = T(1:3,4,2)-T(1:3,4,1);
O1O9 = T(1:3,4,9)-T(1:3,4,1);
O1O16 = T(1:3,4,16)-T(1:3,4,1);
O1O21 = T(1:3,4,21)-T(1:3,4,1);
O1O26 = T(1:3,4,26)-T(1:3,4,1);
    
m = mt(:,1) + m_temp1 + cross_matrix(O1O2)*f_temp1 + m_temp2 + cross_matrix(O1O9)*f_temp2 +...
    m_temp3 + cross_matrix(O1O16)*f_temp3 + m_temp4 + cross_matrix(O1O21)*f_temp4 +...
    m_temp5 + cross_matrix(O1O26)*f_temp5;

F0 = f; % En realidad esta es la fuerza en el marco 2 respecto a cero, i.e. 0f2
M0 = m;


end
