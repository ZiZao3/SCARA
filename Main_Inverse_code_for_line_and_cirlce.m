clear;
L1=5;
L2=4;
coor = [10 10];
elbow = 1;
n=50;
th1=linspace(0,2*pi,n);
th2=linspace(0,-2*pi,n);
traj=[];

x_traj= 1*cos(linspace(0,2*pi,50))+3;  %%use this to trace circle
y_traj= 1*sin(linspace(0,2*pi,50))+3;   %%use this to trace circle
%x_traj= (linspace(0,0,50))+3;    %%use this to trace line
%y_traj= (linspace(0,2,50))+3;    %%use this to trace line

for i=1:1:length(th1)
    clf; %clear previous plots on the figure
   x_des=x_traj(i);
   y_des=y_traj(i);
   elbow = true;
    theta=Invkin_2link(L1,L2,[x_des y_des],elbow); %Calculating forward kinematics and finding the positions of joint 2 and end-effector
    [pos1,pos2]=ForwKin_2link(L1,L2,theta);
    %thata=[theta];
    traj=[traj;pos2] %Saving the co-ordinates of end-effector in trajectory variable
    plotupd_2link(pos1, pos2,traj) %Call the plotting function 
    pause(0.05); 
end

pause(2);
figure;   

%plotting co-ordinates of the end-effector
subplot(1,1,1);
plot(traj(:,1),'LineWidth',2,'Color','Red');
hold on;
plot(traj(:,2),'LineWidth',2,'Color','Blue');
xlabel('Representative Time');
ylabel('Co-ordinate');
legend('X','Y');
title('End-effector position');
pause(2)
% %%
% %Workspace
% 
% %Define the two lengths of links
% %L1=5;
% %L2=2;
% 
% %Define the input vectors for the two joint angles
% %n=50;
% 
% %Creating new input vectors for calculating workspace
% figure;
% th1=linspace(0,2*pi,n);
% th2=linspace(0,2*pi,n);
% 
% traj_workspace=[];
% 
% 
% for i=1:1:n
%     for j=1:1:n
%         %clf;
%         q=[th1(i),th2(j)];
%         [p1,p2]=ForwKin_2link(L1,L2,q);
%         traj_workspace=[traj_workspace;p2]; %Saving the end-effector variables
%         %plotupd_2link(p1, p2,traj_workspace)
%     end
% end
% 
% plotupd_2link(p1, p2,traj_workspace)
% title('Workspace');
% 






