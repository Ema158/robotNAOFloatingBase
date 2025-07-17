function GlobalSupportfoot = animationStandingOnefoot(stored,Nsteps,name,framerate,alpha)
% Modified: 25-jul-2018 -> Victor 
% alpha -> Orientation of the robot in the plane X-Y

%ANIMATION of a robot given the joints values
% stored : configuration et vitesse articulaire généré dans le reportoire
% motion sim; Nsteps doit être inférieur ou egal au nombre de pas simulé
% nom du fichier avi

if nargin == 4
    alpha = 0;
end

robot=genebot();
fig = figure('Position',[0 0 1000 1000]);
subplot(2,2,1);
axis([-0.3,0.3,-0.3,0.3,0,0.6]);
grid on;
set(gcf,'Renderer','zbuffer')
hold on;
subplot(2,2,2);
axis([-0.3,0.3,-0.3,0.3,0,0.6]);
grid on;
view(0,0);
set(gcf,'Renderer','zbuffer')
hold on;
subplot(2,2,3);
axis([-0.3,0.3,-0.3,0.3,0,0.6]);
grid on;
view(-90,0);
set(gcf,'Renderer','zbuffer')
hold on;
subplot(2,2,4);
axis([-0.3,0.3,-0.3,0.3,0,0.6]);
grid on;
view(90,0);
set(gcf,'Renderer','zbuffer')
hold on;
writerobj = VideoWriter(strcat(name,'.avi'));
% writerobj = VideoWriter(strcat(name), 'MPEG-4');
writerobj.FrameRate = framerate; % The smaller FrameRate the slower the video (Frames per second) 
open(writerobj);
x0=0;
y0=0;
COM_trace = [];
global GlobalCoMX GlobalCoMY
index = 1;
GlobalSupportfoot = zeros(3,Nsteps);  % X_s, Y_s and Psi_s
GlobalSupportfoot(1,1) = x0;
GlobalSupportfoot(2,1) = y0;
GlobalSupportfoot(3,1) = alpha;
for k = 1:Nsteps
    % stored a eté stocké pas par pas : qs : vecteur articulaire pour le
    % pas k : qs =matrice de dimension n (31) par le nombre d'échantillon
    qs=stored{k,1};
    n=numel(qs(1,:));
    for i = 1:n        
        robot=robot_move(robot,qs(:,i));   
            for l=1:4
                subplot(2,2,l);
                cla;
                robot_draw(robot);
%                 if l==3
                    axis([-0.3, 0.3,-0.3, 0.3, -0.4 0.4]);
%                 end
            end
        writeVideo(writerobj,getframe(fig));
        if (k==1 && i==1) || (k==Nsteps && i==n)
            for cont = 1:framerate   % Recording of the initial and final position ("framerate" times)
                                     %  in order to see the initial and final configuration of the Robot
                writeVideo(writerobj,getframe(fig));
            end
        end
        index = index + 1;
    end

end
close(writerobj);