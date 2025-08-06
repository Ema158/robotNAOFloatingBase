function robot_drawDoubleForce(robot,FR,FL)
% Modified: 25-jul-2018 -> Victor 
% x0 -> Coordinate of the support foot in X direction
% y0 -> Coordinate of the support foot in Y direction
% alpha -> Orientation of the robot in the plane X-Y
global coms
%Generates a 3D-plot of the robot
T = robot.T;
ant = robot.ant;  
[x,y,z]=sphere;
CoM_Rotated = robot.CoM;
COM = surf(x/100+CoM_Rotated(1), y/100+CoM_Rotated(2), z/100+CoM_Rotated(3));
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on;
set(COM,'FaceColor',[0 0 0]);
set(COM,'EdgeColor',[0 0 0]);
% %Support foot
% -------------------------
temp = T(1:3,4,6);  
x0f = temp(1);
y0f = temp(2);
z0f = temp(3);
% les coins ont des coordonnées fixes dans le repère 13
XAVG = T(:,:,7)*[-0.0460; -0.025; 0.1; 1];%* Cambie el valor de -0.0674 a -0.04660
XAVD = T(:,:,7)*[-0.0460; +0.025; 0.1; 1];%*
XARD = T(:,:,7)*[-0.0460; +0.025; -0.055; 1];%*
XARG = T(:,:,7)*[-0.0460; -0.025; -0.055; 1];%*
% Como deberia ser
% XAVG(3)=0;
% XAVD(3)=0;
% XARG(3)=0;
% XARD(3)=0;
% boucle : 1 = cheville, 2(avg)3 4 5 : 4 coins
% 1 1 1 1 2
% 2 3 2 4 3
% 3 4 5 5 4
% 3 4 5 5 5
xf=[x0f x0f x0f x0f XAVG(1);
    XAVG(1) XAVD(1) XAVG(1) XARG(1) XAVD(1);
    XAVD(1) XARD(1) XARG(1) XARD(1) XARD(1);
    XAVD(1) XARD(1) XARG(1) XARD(1) XARG(1)];
yf=[y0f y0f y0f y0f XAVG(2);
    XAVG(2) XAVD(2) XAVG(2) XARG(2) XAVD(2);
    XAVD(2) XARD(2) XARG(2) XARD(2) XARD(2);
    XAVD(2) XARD(2) XARG(2) XARD(2) XARG(2)];
zf=[z0f z0f z0f z0f XAVG(3);
    XAVG(3) XAVD(3) XAVG(3) XARG(3) XAVD(3);
    XAVD(3) XARD(3) XARG(3) XARD(3) XARD(3);
    XAVD(3) XARD(3) XARG(3) XARD(3) XARG(3)];
h=patch(xf,yf,zf,'g');
set(h,'edgecolor','g');
 
%Support leg
ext1 = T(1:3,4,2);
surf(x/200+ext1(1),y/200+ext1(2),z/200+ext1(3));
ext2 = T(1:3,4,1);
plot3([ext1(1) ext2(1)],[ext1(2) ext2(2)],[ext1(3) ext2(3)],'g','LineWidth',2);
for i = 5:6
    ext1 = T(1:3,4,i);
    surf(x/200+ext1(1),y/200+ext1(2),z/200+ext1(3));

    ext2 = T(1:3,4,ant(i));
    plot3([ext1(1) ext2(1)],[ext1(2) ext2(2)],[ext1(3) ext2(3)],'g','LineWidth',2);
end

% %Swing foot
% -------------------------
% el tobillo tiene sus coordenadas conocidas en el marco 12
temp = T(1:3,4,13);  
x0f = temp(1);
y0f = temp(2);
z0f = temp(3);
% les coins ont des coordonnées fixes dans le repère 13
XAVG = T(:,:,14)*[-0.0460; -0.025; 0.1; 1];%* Cambie el valor de -0.0674 a -0.04660
XAVD = T(:,:,14)*[-0.0460; +0.025; 0.1; 1];%*
XARD = T(:,:,14)*[-0.0460; +0.025; -0.055; 1];%*
XARG = T(:,:,14)*[-0.0460; -0.025; -0.055; 1];%*
% Como deberia ser
% XAVG(3)=0;
% XAVD(3)=0;
% XARG(3)=0;
% XARD(3)=0;
% boucle : 1 = cheville, 2(avg)3 4 5 : 4 coins
% 1 1 1 1 2
% 2 3 2 4 3
% 3 4 5 5 4
% 3 4 5 5 5
xf=[x0f x0f x0f x0f XAVG(1);
    XAVG(1) XAVD(1) XAVG(1) XARG(1) XAVD(1);
    XAVD(1) XARD(1) XARG(1) XARD(1) XARD(1);
    XAVD(1) XARD(1) XARG(1) XARD(1) XARG(1)];
yf=[y0f y0f y0f y0f XAVG(2);
    XAVG(2) XAVD(2) XAVG(2) XARG(2) XAVD(2);
    XAVD(2) XARD(2) XARG(2) XARD(2) XARD(2);
    XAVD(2) XARD(2) XARG(2) XARD(2) XARG(2)];
zf=[z0f z0f z0f z0f XAVG(3);
    XAVG(3) XAVD(3) XAVG(3) XARG(3) XAVD(3);
    XAVD(3) XARD(3) XARG(3) XARD(3) XARD(3);
    XAVD(3) XARD(3) XARG(3) XARD(3) XARG(3)];
 h=patch(xf,yf,zf,'r');
 set(h,'edgecolor','r');

%Swing leg
ext1 = T(1:3,4,9);
surf(x/200+ext1(1),y/200+ext1(2),z/200+ext1(3));
ext2 = T(1:3,4,1);
plot3([ext1(1) ext2(1)],[ext1(2) ext2(2)],[ext1(3) ext2(3)],'r','LineWidth',2);
for i = 12:13
    ext1 = T(1:3,4,i);
    ext2 = T(1:3,4,ant(i));
    
    plot3([ext1(1) ext2(1)],[ext1(2) ext2(2)],[ext1(3) ext2(3)],'r','LineWidth',2);
    surf(x/200+ext2(1),y/200+ext2(2),z/200+ext2(3));
end

ext1 = T(1:3,4,16);
surf(x/200+ext1(1),y/200+ext1(2),z/200+ext1(3));
ext2 = T(1:3,4,1);
plot3([ext1(1) ext2(1)],[ext1(2) ext2(2)],[ext1(3) ext2(3)],'g','LineWidth',2);  
%The right arm
for i = 17:20
    ext1 = T(1:3,4,i);
    ext2 = T(1:3,4,ant(i));
    plot3([ext1(1) ext2(1)],[ext1(2) ext2(2)],[ext1(3) ext2(3)],'g','LineWidth',1.5);
    surf(x/200+ext2(1),y/200+ext2(2),z/200+ext2(3));
end

ext1 = T(1:3,4,21);
surf(x/200+ext1(1),y/200+ext1(2),z/200+ext1(3));
ext2 = T(1:3,4,1);
plot3([ext1(1) ext2(1)],[ext1(2) ext2(2)],[ext1(3) ext2(3)],'r','LineWidth',2); 
%The left arm
for i = 22:25
    ext1 = T(1:3,4,i);    
    ext2 = T(1:3,4,ant(i));
    plot3([ext1(1) ext2(1)],[ext1(2) ext2(2)],[ext1(3) ext2(3)],'r','LineWidth',1.5);
    surf(x/200+ext2(1),y/200+ext2(2),z/200+ext2(3));
end

 %Union de los dos brazos
 ext1 = T(1:3,4,16); 
 ext2 = T(1:3,4,21);
 plot3([ext1(1) ext2(1)],[ext1(2) ext2(2)],[ext1(3) ext2(3)],'c','LineWidth',1.5);
 %Punto medio de la union de los dos brazos
 xm=(T(1,4,16)-T(1,4,21))/2+T(1,4,21);
 ym=(T(2,4,16)-T(2,4,21))/2+T(2,4,21);
 zm=(T(3,4,16)-T(3,4,21))/2+T(3,4,21);
 pm = [xm;ym;zm];
 surf(x/200+pm(1),y/200+pm(2),z/200+pm(3));
 %Union del punto medio con la cabeza
 ext1 = T(1:3,4,26); 
 plot3([ext1(1) pm(1)],[ext1(2) pm(2)],[ext1(3) pm(3)],'r','LineWidth',1.5);
 %Agregar un trazo mas a la cabeza
 ext1 = T(1:3,4,27); 
 ext2 = T(1:3,4,28);
 plot3([ext1(1) ext2(1)],[ext1(2) ext2(2)],[ext1(3) ext2(3)],'m','LineWidth',1.5);
 
 grid on
 axis equal
 
 %Grafico de los centros de masa de cada eslabon
 if coms==1
     CoM_j = robot.InertialParameters.CoM;
     MS_j=zeros(3,28);
     for j=1:28
         MS_j(:,j) = T(1:3,:,j) * [CoM_j{j};1];
         plot3(MS_j(1,j),MS_j(2,j),MS_j(3,j),'*');
     end
 end
%Force plot
scale = 1/50;
ext1 = T(1:3,4,8) + [(-FR(2)/FR(6));(FR(1)/FR(6));0];
ext2 = ext1 + scale*FR(4:6);
plot3([ext1(1) ext2(1)],[ext1(2) ext2(2)],[ext1(3) ext2(3)],'k','LineWidth',1);
 
ext1 = T(1:3,4,15) + [(-FL(2)/FL(6));(FL(1)/FL(6));0];
ext2 = ext1 + scale*FL(4:6);
plot3([ext1(1) ext2(1)],[ext1(2) ext2(2)],[ext1(3) ext2(3)],'k','LineWidth',1); 
 
end
