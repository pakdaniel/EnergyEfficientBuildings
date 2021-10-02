clear all; close all; clc;
s = tf('s');

%% RUN SIMULINK MODEL FIRST TO EXPORT SAT Struct
t = linspace(0,940,941);
u = 18.5*sin(2*pi*t/725+pi/2)+18.5;
start_temp = 60;

figure();
plot(t,u,'LineWidth',1.2);
hold on;
plot(SAT.time,SAT.signals.values.'.*u+start_temp,'LineWidth',1.2);
hold off;
xlim([0,940]);
legend('Input', 'Response','Location','southeast')
xlabel('Time (s)', 'FontSize', 12);
title('Simulated Response for 1/13/2021');

grid on;
grid minor;
%saveas(gcf,'Jan 13 CL Response','png');


