function T = DGM(robot)
%% computing of the geometric model (transformations matrix) of the robot
%theta = robot.q;
frames = 28;
T=zeros(4,4,frames);
Temp =zeros(4,4,25);
ant =       [0,... 1
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
%//////////////////////////////////////////////////////////////////////////

%Base
xb = robot.q(1);
yb = robot.q(2);
zb = robot.q(3);
rollb = robot.q(4);
thetab = robot.q(5);
phib = robot.q(6);
Tbase(1:3,1:3) = RPY(rollb,thetab,phib);
Tbase(1:3,4) = [xb;yb;zb];
Tbase(4,1:4) = [0,0,0,1];
%
baseDof = 6;
theta(1)=robot.q(1+baseDof);
theta(2)=robot.q(2+baseDof) + 3*(pi/4);
theta(3)=robot.q(3+baseDof);
theta(4)=robot.q(4+baseDof);
theta(5)=robot.q(5+baseDof);
theta(6)=robot.q(6+baseDof);

theta(7)=robot.q(7+baseDof)-pi/2;
theta(8)=robot.q(8+baseDof)+pi/4;
theta(9)=robot.q(9+baseDof);
theta(10)=robot.q(10+baseDof);
theta(11)=robot.q(11+baseDof);
theta(12)=robot.q(12+baseDof);

theta(13)=robot.q(13+baseDof);
theta(14)=robot.q(14+baseDof) + pi/2;
theta(15)=robot.q(15+baseDof);
theta(16)=robot.q(16+baseDof);
theta(17)=robot.q(17+baseDof);

theta(18)=robot.q(18+baseDof);
theta(19)=robot.q(19+baseDof) + pi/2;
theta(20)=robot.q(20+baseDof);
theta(21)=robot.q(21+baseDof);
theta(22)=robot.q(22+baseDof);

theta(23)=robot.q(23+baseDof);
theta(24)=robot.q(24+baseDof)-pi/2;
theta(25)=-pi/2;
%///////////////////////////////////////////////////////////////////////////
%Calculo de todas las Matrices de transformacion homogeneas ^i_jT en una matriz global 
mat_t=mat_trans(theta(1:25));
%Separacion de las Matrices de transformacion homogeneas ^i_jT en matrices independientes (Temp) 
Temp(:,:,1)= mat_t(1:4,:);
for j = 2 : 25
    Temp(:,:,j) = mat_t(4*(j-1)+1:4*j,:);    
end
T_M0R =        [0          -1     0      0; 
               0.7071      0  0.7071    0;
               -0.7071     0   0.7071   0;
               0        0       0      1];
T_M0L =        [1     0          0      0; 
               0    0.7071   0.7071     0;
               0     -0.7071   0.7071   0;
   0        0       0      1];
T_Lfoot =        [1    0   0   -0.0452; 
                  0    1   0     0;
                  0    0   1     0;
                  0    0   0     1];
T_Rfoot =        [1    0   0   -0.0452; 
                  0    1   0     0;
                  0    0   1     0;
                  0    0   0     1];
%Calculo de las matrices de transformacion respecto al marco cero ^0_jT 
T(:,:,1) = Tbase;
T(:,:,2) = Tbase*T_M0R*Temp(:,:,1);
for j = 2 : 6
    T(:,:,j+1) = T(:,:,ant(j+1)) * Temp(:,:,j); %
end
T(:,:,8) = T(:,:,7)*T_Rfoot;

T(:,:,9) = Tbase*T_M0L*Temp(:,:,7);
for j = 9 : 13
    T(:,:,j+1) = T(:,:,ant(j+1)) * Temp(:,:,j-1); %
end
T(:,:,15) = T(:,:,14)*T_Lfoot;

T(:,:,16) = Temp(:,:,13);
T(1:3,4,16) = (T(1:3,4,16) + [0;-0.1;0.1615]);
T(:,:,16) = Tbase*T(:,:,16);
for j = 16 : 19
    T(:,:,j+1) = T(:,:,ant(j+1)) * Temp(:,:,j-2); %
end

T(:,:,21) = Temp(:,:,18);
T(1:3,4,21) = (T(1:3,4,21) + [0;0.1;0.1615]);
T(:,:,21) = Tbase*T(:,:,21);
for j = 21 : 24
    T(:,:,j+1) = T(:,:,ant(j+1)) * Temp(:,:,j-2); %
end

T(:,:,26) = Temp(:,:,23);
T(1:3,4,26) = (T(1:3,4,26) + [0;0;0.1615]);
T(:,:,26) = Tbase*T(:,:,26);
for j = 26 : 27
    T(:,:,j+1) = T(:,:,ant(j+1)) * Temp(:,:,j-2); %
end

end

function R0b = RPY(Roll,Pitch,Yaw)
    R0b = [cos(Yaw)*cos(Pitch), cos(Yaw)*sin(Pitch)*sin(Roll) - sin(Yaw)*cos(Roll), cos(Yaw)*sin(Pitch)*cos(Roll) + sin(Yaw)*sin(Roll);
       sin(Yaw)*cos(Pitch), sin(Yaw)*sin(Pitch)*sin(Roll) + cos(Yaw)*cos(Roll), sin(Yaw)*sin(Pitch)*cos(Roll) - cos(Yaw)*sin(Roll);
       -sin(Pitch),         cos(Pitch)*sin(Roll),                          cos(Pitch)*cos(Roll)];
end



