clear all; close all; clc;
s = tf('s');
currentFolder = pwd;

%% Folder Path to Save Images
mkdir week5

%% Pade Approxmition Relative Error Plot

pade_t = -1.8:0.001:1.8;
actual = exp(-pade_t);
pade_0_0 = ones(size(pade_t));
pade_1_1 = (2-pade_t)./(2+pade_t);
pade_2_2 = (pade_t.^2-6*pade_t+12)./(pade_t.^2+6*pade_t+12);
pade_3_3 = (-pade_t.^3+12*pade_t.^2-60*pade_t+120)./(pade_t.^3+12*pade_t.^2+60*pade_t+120);

%Plot of Functions
figure();
plot(pade_t,actual,'LineWidth',1.2);
hold on;
plot(pade_t,pade_0_0,'LineWidth',1.2);
hold on;
plot(pade_t,pade_1_1,'LineWidth',1.2);
hold on;
plot(pade_t,pade_2_2,'LineWidth',1.2);
hold off;
title('Approximating e^{-x} Using Pade Approximation','FontSize',16);
xlabel('s','FontSize',12)
ylabel('Value','FontSize',12)
legend('e^x','0,0','1,1','2,2');
grid on;
grid minor;
ylim([0,13])
saveas(gcf,fullfile(strcat(currentFolder,'\week5'),'Approximating exponential'),'png');

%Plot of Relative Error
figure();
plot(pade_t,100*abs(actual-pade_1_1)./abs(actual),'LineWidth',1.2);
hold on;
plot(pade_t,100*abs(actual-pade_2_2)./abs(actual),'LineWidth',1.2);
hold off;
title('Relative Error of Pade Approximation','FontSize',16);
xlabel('s','FontSize',12)
ylabel('Relative Error (%)','FontSize',12)
legend('1,1','2,2');
grid on;
grid minor;
saveas(gcf,fullfile(strcat(currentFolder,'\week5'),'Approximating exponential RE'),'png');

%% Example of SIMC in action
g = ((-0.3*s+1)*(0.08*s+1))/((2*s+1)*(s+1)*(0.4*s+1)*(0.2*s+1)*(0.05*s+1)^3);
g_prime = exp(-1.47*s)/(2.5*s+1);
g_prime = pade(g_prime,3);

figure();
step(g);
hold on;
step(g_prime);
hold off;
legend('Original Plant','SIMC Plant N=3','Location','best');
ylim([-0.2,1.2]);
grid on;
grid minor;
title('Open Loop Step Response','FontSize',16);
% saveas(gcf,fullfile(strcat(currentFolder,'\week5'),'OL Plant vs SIMC Plant Example'),'png');

figure();
step(feedback(g,1));
hold on;
step(feedback(g_prime,1));
hold off;
legend('Original Plant','SIMC Plant N=3','Location','best');
grid on;
grid minor;
title('Closed Loop Step Response','FontSize',16);
% saveas(gcf,fullfile(strcat(currentFolder,'\week5'),'Feedback Plant vs SIMC Plant Example'),'png');

%% Plant Parameters and Time Vector
clc; close all; 

K = 1.625; %degF/%
tau = 173; %seconds
delay = 86; %seconds THETA
setpoint = 60; %degF
step_options = stepDataOptions('StepAmplitude',setpoint);

%Time Vector
t_inital = 0;
t_final = 1000;
dt = 0.01;
t = t_inital:dt:t_final; 

%Gains from BMS
%PID Gains
Kp = 50;
Ki = 3.5;
Kd = 20;
%% SIMC First-Order Model (Page 57 in Modern Controls)
% https://folk.ntnu.no/skoge/publications/2012/skogestad-improved-simc-pid/old-submitted/simcpid.pdf
% page 12
N = 10; %Order of Pade approximation

P = K/(tau*s+1);
P_delay_exact = P*exp(-delay*s);
P_delay_approx = pade(P_delay_exact,N);
tau_array = [1, 10, 100]; %tuning parameter

gain_array_SIMC = zeros(3,3);

% for ii = 1:3
%     tau_c = tau_array(ii);
%     tau_I = min(tau,4*(tau_c+delay));
%     K_c = (1/K)*tau/(tau_c+delay);
%     K_pid = K_c*(1+1/(tau_I*s));
%     [num,den] = tfdata(K_pid,'v');
%     
%     gain_array_SIMC(ii,1) = tau_c;
%     gain_array_SIMC(ii,2) = num(1)/den(1);
%     gain_array_SIMC(ii,3) = num(2)/den(1);
%     
%     L = P*K_pid;
%     L_delay_approx = P_delay_approx*K_pid;
%     L_delay_exact = P_delay_exact*K_pid;
%     [y,t] = step(feedback(L_delay_approx,1),step_options);
%     
%     stepinfo(y,t)
%     
%     step(feedback(L,1),step_options);
%     hold on;
%     step(feedback(L_delay_exact,1),step_options);
%     hold on;
%     step(feedback(L_delay_approx,1),step_options);
%     hold off;
%     legend('Plant','Plant with Exact Delay','Plant with Approx Delay','Location','best');
%     grid on;
%     grid minor;
%     title("Closed-Loop Response with SIMC \tau_c = " + tau_c);
%     saveas(gcf,currentFolder + "\week5\" + "SIMC with Real Plant tauc" + tau_c,'png');
% end

figure();
for ii = 1:3
    tau_c = tau_array(ii);
    tau_I = min(tau,4*(tau_c+delay));
    K_c = (1/K)*tau/(tau_c+delay);
    K_pid = K_c*(1+1/(tau_I*s));
    [num,den] = tfdata(K_pid,'v');
    L_delay_approx = P_delay_approx*K_pid;
    step(feedback(L_delay_approx,1),step_options);
    stepinfo(feedback(L_delay_approx,1))
    hold on;
end
hold off;
legend('\tau_c = 1', '\tau_c = 10', '\tau_c = 100');
legend('\tau_c = 1', '\tau_c = 10', '\tau_c = 100');
grid on;
grid minor;
title("Closed-Loop Response with SIMC",'FontSize',16);
saveas(gcf,currentFolder + "\week5\" + "SIMC CL",'png');
%% Disturbance Rejection SIMC First-Order Model (Page 57 in Modern Controls)
% https://folk.ntnu.no/skoge/publications/2012/skogestad-improved-simc-pid/old-submitted/simcpid.pdf
% page 12
% N = 10; %Order of Pade approximation
% 
% P = K/(tau*s+1);
% P_delay_exact = P*exp(-delay*s);
% P_delay_approx = pade(P_delay_exact,N);
% tau_array = [1, 10, 100]; %tuning parameter
% 
% gain_array_SIMC = zeros(3,3);
% 
% for ii = 1:3
%     tau_c = tau_array(ii);
%     tau_I = min(tau,4*(tau_c+delay));
%     K_c = (1/K)*tau/(tau_c+delay);
%     K_pid = K_c*(1+1/(tau_I*s));
%     [num,den] = tfdata(K_pid,'v');
%     
%     gain_array_SIMC(ii,1) = tau_c;
%     gain_array_SIMC(ii,2) = num(1)/den(1);
%     gain_array_SIMC(ii,3) = num(2)/den(1);
%     
%     L = P*K_pid;
%     L_delay_approx = P_delay_approx*K_pid;
%     L_delay_exact = P_delay_exact*K_pid;
%     [y,t] = step(feedback(P_delay_approx,K_pid),step_options);
%     stepinfo(y,t)
%     
%     stepinfo(y,t)
%     figure;
%     step(feedback(P,K_pid),step_options);
%     hold on;
%     step(feedback(P_delay_exact,K_pid),step_options);
%     hold on;
%     step(feedback(P_delay_approx,K_pid),step_options);
%     hold off;
%     legend('Plant','Plant with Exact Delay','Plant with Approx Delay','Location','best');
%     grid on;
%     grid minor;
%     title("Closed-Loop Disturbance Rejection with SIMC \tau_c = " + tau_c);
%     saveas(gcf,currentFolder + "\week5\" + "Disturbance Rejection SIMC" + tau_c,'png');
% end
figure();
for ii = 1:3
    tau_c = tau_array(ii);
    tau_I = min(tau,4*(tau_c+delay));
    K_c = (1/K)*tau/(tau_c+delay);
    K_pid = K_c*(1+1/(tau_I*s));
    [num,den] = tfdata(K_pid,'v');
    L_delay_approx = P_delay_approx*K_pid;
    step(feedback(P_delay_approx,K_pid),step_options);
    stepinfo(P_delay_approx/(1+P_delay_approx*K_pid))

    hold on;
end
hold off;
legend('\tau_c = 1', '\tau_c = 10', '\tau_c = 100');
legend('\tau_c = 1', '\tau_c = 10', '\tau_c = 100');
grid on;
grid minor;
title("Disturbance Rejection with SIMC",'FontSize',16);
saveas(gcf,currentFolder + "\week5\" + "SIMC Disturbance",'png');

%% Margins for SIMC
for ii = 1:3
    tau_c = tau_array(ii);
    tau_I = min(tau,4*(tau_c+delay));
    K_c = (1/K)*tau/(tau_c+delay);
    K_pid = K_c*(1+1/(tau_I*s));
    L_delay_exact = P_delay_exact*K_pid;
    figure();
    margin(L_delay_exact); 
    legend(['$\tau_{c}=$', num2str(tau_c)], 'Interpreter','latex','Location','best');
    legend(['$\tau_{c}=$', num2str(tau_c)]);
%     saveas(gcf,currentFolder + "\week5\" + "Margin with Real Plant tauc" + tau_c,'png');
end
% hold off;
% legend('$\tau_{c}=1$','$\tau_{c}=10$','$\tau_{c}=100$','Interpreter','latex','Location','best');
% legend({'$\tau_{c}=1$','$\tau_{c}=10$','$\tau_{c}=100$'}); %call legend twice due to error in margin()

%% Improved SIMC PI-rule for first-order with delay process. (page 24)
gain_array_Improved_SIMC = zeros(3,3);

% for ii = 1:3
%     tau_c = tau_array(ii);
%     tau_I = min(tau+delay/3,4*(tau_c+delay));
%     K_c = (1/K)*(tau+delay/3)/(tau_c+delay);
%     K_pid = K_c*(1+1/(tau_I*s));
% 
%     [num,den] = tfdata(K_pid,'v');
%     
%     gain_array_Improved_SIMC(ii,1) = tau_c;
%     gain_array_Improved_SIMC(ii,2) = num(1)/den(1);
%     gain_array_Improved_SIMC(ii,3) = num(2)/den(1);
%     
%     L = P*K_pid;
%     L_delay_approx = P_delay_approx*K_pid;
%     L_delay_exact = P_delay_exact*K_pid;
% 
%     [y,t] = step(feedback(L_delay_approx,1),step_options);
%     
%     stepinfo(y,t)  
%   
%     figure();
%     
%     margin(L_delay_approx);
%     step(feedback(L,1),step_options);
%     hold on;
%     step(feedback(L_delay_exact,1),step_options);
%     hold on;
%     step(feedback(L_delay_approx,1),step_options);
%     hold off;
%     legend('Plant','Plant with Exact Delay','Plant with Approx Delay','Location','best');
%     grid on;
%     grid minor;
%     title("Closed-Loop Response with Improved SIMC \tau_c = " + tau_c);
%     saveas(gcf,currentFolder + "\week5\" + "Improved SIMC with Real Plant tauc" + tau_c,'png');
% 
% end

figure();
for ii = 1:3
    tau_c = tau_array(ii);
    tau_I = min(tau+delay/3,4*(tau_c+delay));
    K_c = (1/K)*(tau+delay/3)/(tau_c+delay);
    K_pid = K_c*(1+1/(tau_I*s));
    [num,den] = tfdata(K_pid,'v');
    L_delay_approx = P_delay_approx*K_pid;
    step(feedback(L_delay_approx,1),step_options);
    stepinfo(feedback(L_delay_approx,1))
    hold on;
end
hold off;
legend('\tau_c = 1', '\tau_c = 10', '\tau_c = 100');
legend('\tau_c = 1', '\tau_c = 10', '\tau_c = 100');
grid on;
grid minor;
title("Closed-Loop Response with Improved SIMC",'FontSize',16);
saveas(gcf,currentFolder + "\week5\" + "Improved SIMC CL",'png');


%% Improved SIMC Disturbance Rejection PI-rule for first-order with delay process. (page 24)
gain_array_Improved_SIMC = zeros(3,3);

% for ii = 1:3
%     tau_c = tau_array(ii);
%     tau_I = min(tau+delay/3,4*(tau_c+delay));
%     K_c = (1/K)*(tau+delay/3)/(tau_c+delay);
%     K_pid = K_c*(1+1/(tau_I*s));
% 
%     [num,den] = tfdata(K_pid,'v');
%     
%     gain_array_Improved_SIMC(ii,1) = tau_c;
%     gain_array_Improved_SIMC(ii,2) = num(1)/den(1);
%     gain_array_Improved_SIMC(ii,3) = num(2)/den(1);
%     
%     L = P*K_pid;
%     L_delay_approx = P_delay_approx*K_pid;
%     L_delay_exact = P_delay_exact*K_pid;
% 
%     [y,t] = step(feedback(P_delay_approx,K_pid),step_options);
%     
%     stepinfo(y,t)  
%   
%     figure();
%     
%     step(feedback(P,K_pid),step_options);
%     hold on;
%     step(feedback(P_delay_exact,K_pid),step_options);
%     hold on;
%     step(feedback(P_delay_approx,K_pid),step_options);
%     hold off;
%     legend('Plant','Plant with Exact Delay','Plant with Approx Delay','Location','best');
%     grid on;
%     grid minor;
%     title("Closed-Loop Disturbance Rejection with Improved SIMC \tau_c = " + tau_c);
%     saveas(gcf,currentFolder + "\week5\" + "Improved SIMC Disturbance Rejection tauc" + tau_c,'png');
% 
% end

figure();
for ii = 1:3
    tau_c = tau_array(ii);
    tau_I = min(tau+delay/3,4*(tau_c+delay));
    K_c = (1/K)*(tau+delay/3)/(tau_c+delay);
    K_pid = K_c*(1+1/(tau_I*s));
    [num,den] = tfdata(K_pid,'v');
    L_delay_approx = P_delay_approx*K_pid;
    step(feedback(P_delay_approx,K_pid),step_options);

    hold on;
end
hold off;
legend('\tau_c = 1', '\tau_c = 10', '\tau_c = 100');
legend('\tau_c = 1', '\tau_c = 10', '\tau_c = 100');
grid on;
grid minor;
title("Disturbance Rejection with Improved SIMC",'FontSize',16);
saveas(gcf,currentFolder + "\week5\" + "Improved SIMC Disturbance",'png');

%% Root Locus
z1 = 0.0058;
C_pi_rl = (s+z1)/s;
figure();
rlocus(P_delay_approx*C_pi_rl);
figure();
step(feedback(1.1092*P_delay_approx*C_pi_rl,1));

%% Margin
P_delay_approx = pade(P_delay_exact,3);

tau_c = 100;
tau_I = min(tau+delay/3,4*(tau_c+delay));
K_c = (1/K)*(tau+delay/3)/(tau_c+delay);
K_pid = K_c*(1+1/(tau_I*s));
L_delay_approx = P_delay_approx*K_pid;
figure();
margin(L_delay_approx);
saveas(gcf,currentFolder + "\week5\" + "bode100",'png');


%% IMC-Based PID and Deadtime
% https://rpi.edu/dept/chem-eng/WWW/faculty/bequette/courses/cpc/IMC_PID.pdf
P = K/(tau*s+1);
P_delay_exact = P*exp(-delay*s);
P_delay_approx = pade(P_delay_exact,3);

lambda = 141;
K_c = (tau+0.5*delay)/(K*(lambda+0.5*delay));
tau_I = tau+0.5*delay;
tau_D = tau*delay/(2*tau+delay);

K_pid = K_c*(1+1/(tau_I*s)+tau_D*s);


L = P*K_pid;
L_delay_approx = P_delay_approx*K_pid;
L_delay_exact = P_delay_exact*K_pid;

step(feedback(L,1),step_options);
hold on;
% step(feedback(L_delay_exact,1),step_options);
% hold on;
step(feedback(L_delay_approx,1),step_options);
hold off;
legend('Plant','Plant with Exact Delay','Plant with Approx Delay','Location','best');
grid on;
grid minor;

%% Improved IMC-Based PI Design for a First-Order + Deadtime Process
g = K/((tau+0.5*delay)*s+1);
lambda = 1;
K_c = (tau+0.5*delay)/(K*lambda);
tau_I = tau+0.5*delay;

K_pid = K_c*(1+1/(tau_I*s));

step(feedback(g*K_pid,1));


