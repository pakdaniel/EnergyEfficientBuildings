clear all; close all; clc;
s = tf('s');

%%
mkdir week4
currentFolder = pwd;

%%
clc; close all;
K = 1.625; %degF/%
tau = 173; %seconds
delay = 86; %seconds
setpoint = 60; %degF

ti = 0;
tf = 1000;
dt = 0.01;
t = ti:dt:tf; %time vector
N = 3; %Order of Pade approximation

P = K/(tau*s+1);
P_delay_exact = P*exp(-delay*s);
P_delay_approx = pade(P_delay_exact,N);

step_options = stepDataOptions('StepAmplitude',setpoint);

%% OL Step Response
[step_plant,~] = step(P,t,step_options);
[step_plant_td_approx,~] = step(P_delay_approx,t,step_options);
[step_plant_td_exact,~] = step(P_delay_exact,t,step_options);

figure();
plot(t,step_plant,'LineWidth',1.2);
hold on;
plot(t,step_plant_td_exact,'LineWidth',1.2);
hold on;
plot(t,step_plant_td_approx,'LineWidth',1.2);
hold off;

title('OL Step Response');
xlabel('Time (s)');
ylabel('Temperature (°F)');
legend('No Delay','Exact Delay',sprintf('Approx Delay N=%d',N),'Location','best');
grid on;
grid minor;

%% CL Step Response with No Delay
clc;
%PID Gains
Kp = 50;
Ki = 3.5;
Kd = 20;

C_p = Kp;
C_pi = Kp+Ki/s;
C_pid = Kp+Ki/s+Kd*s;

% fprintf('%d \n',Ki/Kp)
[fb_p,~] = step(feedback(P*C_p,1),t,step_options);
[fb_pi,~] = step(feedback(P*C_pi,1),t,step_options);
[fb_pid,~] = step(feedback(P*C_pid,1),t,step_options);

figure();
plot(t(1:5000),fb_p(1:5000));
hold on;
plot(t(1:5000),fb_pi(1:5000));
hold on;
plot(t(1:5000),fb_pid(1:5000));
hold on;
% plot(t(1:5000),setpoint*ones(5000),'--k');
% hold off;
title(sprintf('Closed Loop Step Response P=%d I=%0.1f D=%d', Kp,Ki,Kd));
xlabel('Time (s)');
ylabel('Temperature (°F)');
legend('P','PI','PID','Location','best');
grid on;
grid minor;
% saveas(gcf,fullfile(strcat(currentFolder,'\week4'),'CL PID of Plant'),'png');
%%
step(feedback(P*C_p,1),step_options);
step(feedback(P*C_pi,1),step_options);
step(feedback(P*C_pid,1),step_options);
%% Disturbance Rejection with No Delay
figure();
step(P/(P*C_pid+1),step_options);
ylabel('Temperature (°F)')
% legend('No Delay',sprintf('Delay N=%d',N),'Location','best');
grid on;
grid minor;
title('Disturbance Rejection');
saveas(gcf,fullfile(strcat(currentFolder,'\week4'),'Disturbance Rejection of Plant'),'png');

%% Root locus
z1 = Ki/Kp;
C_pi_rl = (s+z1)/s;
z2 = Kp/Kd;
z3 = Ki/Kd;
C_pid_rl = (s^2+z2*s+z3)/s;

figure(); 
rlocus(P);
title('Root Locus of Plant Using P Controller');
% saveas(gcf,fullfile(strcat(currentFolder,'\week4'),'RL of Plant using P'),'png');

figure();
rlocus(P*C_pi_rl);
title('Root Locus of Plant Using PI Controller');
% saveas(gcf,fullfile(strcat(currentFolder,'\week4'),'RL of Plant using PI'),'png');

figure();
rlocus(P*C_pid_rl);
title('Root Locus of Plant Using PID Controller');
% saveas(gcf,fullfile(strcat(currentFolder,'\week4'),'RL of Plant using PID'),'png');

figure(); 
rlocus(P_delay_approx);
title('Root Locus of Plant with Delay Using P Controller');
% saveas(gcf,fullfile(strcat(currentFolder,'\week4'),'RL of Plant with Delay using P'),'png');
% margin(P_delay_approx)

figure();
rlocus(P_delay_approx*C_pi_rl);
title('Root Locus of Plant with Delay Using PI Controller');
% saveas(gcf,fullfile(strcat(currentFolder,'\week4'),'RL of Plant with Delay using PI'),'png');
% margin(P_delay_approx*C_pi_rl)

figure();
rlocus(P_delay_approx*C_pid_rl);
title('Root Locus of Plant with Delay Using PID Controller');
% saveas(gcf,fullfile(strcat(currentFolder,'\week4'),'RL of Plant with Delay using PID'),'png');
% margin(P_delay_approx*C_pid_rl)

%%
C_p = Kp;
C_pi = Kp+Ki/s;
C_pid = Kp+Ki/s+Kd*s;

[fb_p,~] = step(feedback(P_delay_approx*C_p,1),t,step_options);
[fb_pi,~] = step(feedback(P_delay_approx*C_pi,1),t,step_options);
[fb_pid,~] = step(feedback(P_delay_approx*C_pid,1),t,step_options);

figure();
plot(t(1:3000),fb_p(1:3000));
hold on;
plot(t(1:3000),fb_pi(1:3000));
hold on;
plot(t(1:3000),fb_pid(1:3000));
hold on;
plot(t(1:3000),setpoint*ones(3000),'--k');
hold off;
title(sprintf('Closed Loop Step Response P=%d I=%0.1f D=%d', Kp,Ki,Kd));
xlabel('Time (s)');
ylabel('Temperature (°F)');
legend('P','PI','PID','Location','best');
grid on;
grid minor;
saveas(gcf,fullfile(strcat(currentFolder,'\week4'),'CL PID of Plant with Delay'),'png');

%% Disturbance Rejection with No Delay
figure();
step(P_delay_approx/(P_delay_approx*C_pid+1),step_options);
ylabel('Temperature (°F)')
% legend('No Delay',sprintf('Delay N=%d',N),'Location','best');
grid on;
grid minor;
title('Disturbance Rejection');
% saveas(gcf,fullfile(strcat(currentFolder,'\week4'),'Disturbance Rejection of Plant with Delay'),'png');

%%
figure();
step(feedback(P_delay_approx*C_pid/1000,1),step_options);
title(sprintf('Closed Loop Step Response (C/1000)'));
grid on;
grid minor;
ylabel('Temperature (°F)')
saveas(gcf,fullfile(strcat(currentFolder,'\week4'),'CL PID of Plant with Delay with Changed C'),'png');

figure();
step(feedback(P_delay_approx,C_pid/1000),step_options);
title(sprintf('Disturbance Rejection (C/1000)'));
grid on;
grid minor;
ylabel('Temperature (°F)')
saveas(gcf,fullfile(strcat(currentFolder,'\week4'),'Disturbance Rejection of Plant with Delay with Changed C'),'png');

