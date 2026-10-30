
% controller design project

clear
clc
close all

run('System_define.m');

s = tf('s');
Gest = 1/((s+1)^4);

% FOPDT model estimation
[K0,L0,T0] = get_fod(Gest,0);
G_freq = exp(-L0*s)*K0/(T0*s+1);

[K1,L1,T1] = get_fod(Gest,1);
G_step = exp(-L1*s)*K1/(T1*s+1);

G_opt = opt_app(Gest,0,1,1);

figure(1)
step(Gest,G_freq,G_step,G_opt);
legend show
grid on
set(findall(gcf,'Type','Line'),'LineWidth',1.5)

% G_step is chosen
[Kc,pp,wc,wp] = margin(Gest);
Tc = 2*pi/wc;
N = 10;

% ZN for PI PID PI-D for tf response
figure(2); hold on; title('ZN tf')
plot(0:1:50,ones(51,1)*0.5,'k-.')
plot(0:1:50,ones(51,1)*0.525,'k-.')
plot(0:1:50,ones(51,1)*0.475,'k-.')
plot(0:1:50,ones(51,1)*0.6,'k-.')

[Gc,H,Kp,Ti,Td]=ziegler_nic(2,[K1,L1,T1,N]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,H,Kp,Ti,Td]=ziegler_nic(3,[K1,L1,T1,N]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,H,Kp,Ti,Td]=ziegler_nic(4,[K1,L1,T1,N]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

legend('','','','','PI','PID','PI-D')
grid on
set(findall(gcf,'Type','Line'),'LineWidth',1.5)
set(gca,'GridAlpha',0.15,'MinorGridAlpha',0.1)

% ZN for PI PID PI-D for freq response
figure(3); hold on; title('ZN freq')
plot(0:1:50,ones(51,1)*0.5,'k-.')
plot(0:1:50,ones(51,1)*0.525,'k-.')
plot(0:1:50,ones(51,1)*0.475,'k-.')
plot(0:1:50,ones(51,1)*0.6,'k-.')

[Gc,H,Kp,Ti,Td]=ziegler_nic(2,[Kc,Tc,N]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,H,Kp,Ti,Td]=ziegler_nic(3,[Kc,Tc,N]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,H,Kp,Ti,Td]=ziegler_nic(4,[Kc,Tc,N]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

legend('','','','','PI','PID','PI-D')
grid on
set(findall(gcf,'Type','Line'),'LineWidth',1.5)
set(gca,'GridAlpha',0.15,'MinorGridAlpha',0.1)

% refined ZN
figure(4); hold on; title('refined ZN')
plot(0:1:50,ones(51,1)*0.5,'k-.')
plot(0:1:50,ones(51,1)*0.525,'k-.')
plot(0:1:50,ones(51,1)*0.475,'k-.')
plot(0:1:50,ones(51,1)*0.6,'k-.')

[Gc,H,Kp,Ti,Td,beta]=rziegler_nic([K1,L1,T1,N,Kc,Tc]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

grid on
set(findall(gcf,'Type','Line'),'LineWidth',1.5)
set(gca,'GridAlpha',0.15,'MinorGridAlpha',0.1)

% CHR PI PID PI-D for set point and no overshoots
figure(5); hold on; title('CHR sp os=0')
plot(0:1:50,ones(51,1)*0.5,'k-.')
plot(0:1:50,ones(51,1)*0.525,'k-.')
plot(0:1:50,ones(51,1)*0.475,'k-.')
plot(0:1:50,ones(51,1)*0.6,'k-.')

[Gc,H,Kp,Ti,Td]=chr_pid(2,1,[K1,L1,T1,N,0]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,H,Kp,Ti,Td]=chr_pid(3,1,[K1,L1,T1,N,0]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

legend('','','','','PI','PID')
grid on
set(findall(gcf,'Type','Line'),'LineWidth',1.5)
set(gca,'GridAlpha',0.15,'MinorGridAlpha',0.1)

% CHR PI PID PI-D for set point and 20% overshoots
figure(6); hold on; title('CHR sp os=20%')
plot(0:1:50,ones(51,1)*0.5,'k-.')
plot(0:1:50,ones(51,1)*0.525,'k-.')
plot(0:1:50,ones(51,1)*0.475,'k-.')
plot(0:1:50,ones(51,1)*0.6,'k-.')

[Gc,H,Kp,Ti,Td]=chr_pid(2,1,[K1,L1,T1,N,1]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,H,Kp,Ti,Td]=chr_pid(3,1,[K1,L1,T1,N,1]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

legend('','','','','PI','PID')
grid on
set(findall(gcf,'Type','Line'),'LineWidth',1.5)
set(gca,'GridAlpha',0.15,'MinorGridAlpha',0.1)

% wjc
figure(7); hold on; title('wjc')
plot(0:1:50,ones(51,1)*0.5,'k-.')
plot(0:1:50,ones(51,1)*0.525,'k-.')
plot(0:1:50,ones(51,1)*0.475,'k-.')
plot(0:1:50,ones(51,1)*0.6,'k-.')

[Gc,Kp,Ti,Td]=wjcpid([K1,L1,T1,N]); H=1;
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

grid on
set(findall(gcf,'Type','Line'),'LineWidth',1.5)
set(gca,'GridAlpha',0.15,'MinorGridAlpha',0.1)

% CC PI PID PI-D 
figure(8); hold on; title('CC')
plot(0:1:50,ones(51,1)*0.5,'k-.')
plot(0:1:50,ones(51,1)*0.525,'k-.')
plot(0:1:50,ones(51,1)*0.475,'k-.')
plot(0:1:50,ones(51,1)*0.6,'k-.')

[Gc,H,Kp,Ti,Td]=cohen_pid(2,1,[K1,L1,T1,N]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,H,Kp,Ti,Td]=cohen_pid(3,1,[K1,L1,T1,N]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,H,Kp,Ti,Td]=cohen_pid(4,1,[K1,L1,T1,N]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

legend('','','','','PI','PID','PI-D')
grid on
set(findall(gcf,'Type','Line'),'LineWidth',1.5)
set(gca,'GridAlpha',0.15,'MinorGridAlpha',0.1)

% CC revisited PI PID PI-D 
figure(9); hold on; title('CC revisited')
plot(0:1:50,ones(51,1)*0.5,'k-.')
plot(0:1:50,ones(51,1)*0.525,'k-.')
plot(0:1:50,ones(51,1)*0.475,'k-.')
plot(0:1:50,ones(51,1)*0.6,'k-.')

[Gc,H,Kp,Ti,Td]=cohen_pid(2,2,[K1,L1,T1,N]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,H,Kp,Ti,Td]=cohen_pid(3,2,[K1,L1,T1,N]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,H,Kp,Ti,Td]=cohen_pid(4,2,[K1,L1,T1,N]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

legend('','','','','PI','PID','PI-D')
grid on
set(findall(gcf,'Type','Line'),'LineWidth',1.5)
set(gca,'GridAlpha',0.15,'MinorGridAlpha',0.1)

% AH PI PID for both tf and freq responses
figure(10); hold on; title('AH')
plot(0:1:50,ones(51,1)*0.5,'k-.')
plot(0:1:50,ones(51,1)*0.525,'k-.')
plot(0:1:50,ones(51,1)*0.475,'k-.')
plot(0:1:50,ones(51,1)*0.6,'k-.')

[Gc,H,Kp,Ti,Td]=astrom_hagglund(1,1,[K1,L1,T1,N]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,H,Kp,Ti,Td]=astrom_hagglund(2,1,[K1,L1,T1,N]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,H,Kp,Ti,Td]=astrom_hagglund(1,2,[K1,Kc,Tc,N]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,H,Kp,Ti,Td]=astrom_hagglund(2,2,[K1,Kc,Tc,N]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

legend('','','','','PI tf','PID tf','PI freq','PID freq')
grid on
set(findall(gcf,'Type','Line'),'LineWidth',1.5)
set(gca,'GridAlpha',0.15,'MinorGridAlpha',0.1)

% Opt-pid PI PID PI-D for tf response and sp
figure(11); hold on; title('opt-pid sp tf')
plot(0:1:50,ones(51,1)*0.5,'k-.')
plot(0:1:50,ones(51,1)*0.525,'k-.')
plot(0:1:50,ones(51,1)*0.475,'k-.')
plot(0:1:50,ones(51,1)*0.6,'k-.')

[Gc,Kp,Ti,Td,H] = Optimum(2,1,[K1,L1,T1,N,1]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,Kp,Ti,Td,H] = Optimum(3,1,[K1,L1,T1,N,1]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,Kp,Ti,Td,H] = Optimum(4,1,[K1,L1,T1,N,1]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,Kp,Ti,Td,H] = Optimum(2,1,[K1,L1,T1,N,2]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,Kp,Ti,Td,H] = Optimum(3,1,[K1,L1,T1,N,2]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,Kp,Ti,Td,H] = Optimum(4,1,[K1,L1,T1,N,2]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,Kp,Ti,Td,H] = Optimum(2,1,[K1,L1,T1,N,3]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,Kp,Ti,Td,H] = Optimum(3,1,[K1,L1,T1,N,3]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,Kp,Ti,Td,H] = Optimum(4,1,[K1,L1,T1,N,3]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

legend('','','','','PI ISE','PID ISE','PI-D ISE','PI ISTE','PID ISTE',...
    'PI-D ISTE','PI IST2E','PID IST2E','PI-D IST2E')
grid on
set(findall(gcf,'Type','Line'),'LineWidth',1.5)
set(gca,'GridAlpha',0.15,'MinorGridAlpha',0.1)

% Opt-pid PI PID PI-D for freq response and sp
figure(12); hold on; title('opt-pid sp freq')
plot(0:1:50,ones(51,1)*0.5,'k-.')
plot(0:1:50,ones(51,1)*0.525,'k-.')
plot(0:1:50,ones(51,1)*0.475,'k-.')
plot(0:1:50,ones(51,1)*0.6,'k-.')

[Gc,Kp,Ti,Td,H] = Optimum(2,1,[K1,L1,T1,N,Kc,Tc,K1*Kc]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,Kp,Ti,Td,H] = Optimum(3,1,[K1,L1,T1,N,Kc,Tc,K1*Kc]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,Kp,Ti,Td,H] = Optimum(4,1,[K1,L1,T1,N,Kc,Tc,K1*Kc]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

legend('','','','','PI','PID','PI-D')
grid on
set(findall(gcf,'Type','Line'),'LineWidth',1.5)
set(gca,'GridAlpha',0.15,'MinorGridAlpha',0.1)

% comparing results
figure(13); hold on; title('final compare')
plot(0:1:50,ones(51,1)*0.5,'k-.')
plot(0:1:50,ones(51,1)*0.525,'k-.')
plot(0:1:50,ones(51,1)*0.475,'k-.')
plot(0:1:50,ones(51,1)*0.6,'k-.')

[Gc,H,Kp,Ti,Td]=chr_pid(3,1,[K1,L1,T1,N,1]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,Kp,Ti,Td]=wjcpid([K1,L1,T1,N]); H=1;
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,H,Kp,Ti,Td]=astrom_hagglund(2,1,[K1,L1,T1,N]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

[Gc,Kp,Ti,Td,H] = Optimum(3,1,[K1,L1,T1,N,2]);
out = sim('Control.slx');
plot(out.simout.time,out.simout.data)

legend('','','','','CHR PID sp os=20%','wjc','AH PID tf',...
    'Optpid PID ISTE sp tf ')
grid on
set(findall(gcf,'Type','Line'),'LineWidth',1.5)
set(gca,'GridAlpha',0.15,'MinorGridAlpha',0.1)

% best controller is wjc PID
[Gc,Kp,Ti,Td]=wjcpid([K1,L1,T1,N]); H=1;
out = sim('Control.slx');

