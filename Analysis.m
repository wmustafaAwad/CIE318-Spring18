

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Root Locus Analysis
%% Uncompansated Design
M = 0.5;
m = 0.2;
b = 0.1;
I = 0.006;
g = 9.8;
l = 0.3;
q = (M+m)*(I+m*l^2)-(m*l)^2;
s = tf('s');
pendulum = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);

rlocus(pendulum)
title('Root Locus of uncompansated system')
% The system is unstable with a pole in the right half plane
%% Adding a pole at the origin to cancel the zero at the origin and eliminate the R.H.P branch 
C = 1/s;
rlocus(C*pendulum)
title('Root Locus with Integral Control')
%% Getting the location of open-loop poles and zeros 
zeros = zero(C*P_pend)
poles = pole(C*P_pend)
%% PID Controller
z = [-3 -4];
p = 0;
k = 1;
C = zpk(z,p,k);
rlocus(C*pendulum)
title('Root Locus with PID Controller')
%% Feedback
K = 21.7;
T = feedback(pendulum,K*C);
t=0:0.01:8.5; 
impulse(T,t)
%rlocus(T)
title('Impulse Disturbance Response of Pendulum Angle under PID Control');
%% Cart Position 
CartPosition=(((I+m*I^2)/q)*s^2-(m*g*I/q))/(s^4+(b*(I+m*I^2))*s^3/q-((M+m)*m*g*I)*s^2/q-b*m*g*I*s/q);
Tcart= feedback(1,pendulum*C)*CartPosition;
t=0:0.01:8.5; 
%impulse(T2,t); 
rlocus(Tcart)
%title('Impulse Disturbance Response of cart position under PID Control');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Frequency Domain Analysis

M = 0.5; %kg %Cart mass
m = 0.2; %kg %Pendulum mass
b = 0.1; %N/m/sec %coefficient of friction of the cart with the ground
I = 0.006;%mass of moment of inertia of the pendulum kg.m^2
g = 9.8; %m/s^2
l = 0.3; %m %length of pendulum to cetner of mass %equals to length of pendulum in our case

q = (M+m)*(I+m*l^2)-(m*l)^2;
s = tf('s');

P_cart = (((I+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 + (b*(I + m*l^2))*s^3/q - ((M + m)*m*g*l)*s^2/q - b*m*g*l*s/q);

P_pend = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);

sys_tf = [P_cart ; P_pend];

inputs = {'u'};
outputs = {'x'; 'phi'};

set(sys_tf,'InputName',inputs)
set(sys_tf,'OutputName',outputs)



[zeros poles] = zpkdata(P_pend,'v')
 
%%
 figure
 bodeplot(P_pend)
 figure
 nyquist(P_pend)

 %%
%Adding an integrator
figure
bodeplot(P_pend*1/s)
figure
nyquist(P_pend*1/s)

 %% Adding a zero at -1 (s+1)
 figure
bodeplot(P_pend*(1/s)*(s+1))
figure
nyquist(P_pend*(1/s)*(s+1))

%% Adding ONE MORE zero at -1 (s+1)^2
figure
bodeplot(P_pend*(1/s)*(s+1)^2)
figure
nyquist(P_pend*(1/s)*(s+1)^2)

%% Gain Adjustment Data
K=3.508;
 [Gm,Pm,Wgm,Wpm] = margin(P_pend*K*1/s*(s+1)^2) 
%% Gain Adjustment Plots
K=10;
 figure 
 nyquist(P_pend*K*1/s*(s+1)^2)
 figure 
 margin(P_pend*K*1/s*(s+1)^2) %%Bode Plot with labeled margins
 
 
  
 %% Impulse Response of the System
 t=0:0.01:1;
 K=50;
 TF=feedback(P_pend,K*5*(1/s)*(s+1)*(s+3));
 impulse(TF,t)
 
 