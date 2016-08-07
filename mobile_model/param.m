close all
%Brush DC Motor Experiment Parameter
K=1;
r=0.065; %m
M=20; %kg
l=0.2; %m
I=0.8; %kg*r^2
B=0.1; %
BI=0.1;

ppr=1024*5;
control_ts=0.01;

%%%%% V %%%%%%%
kpv=200;
kiv=10;
Cv_zpk=zpk(-kiv/kpv,0,kpv);
Gv_zpk=zpk([],[-B/M],r*K/M);
Cv=tf(Cv_zpk);
Gv=tf(Gv_zpk);
Hv=1;
clooptf_v=feedback(Cv*Gv,Hv)
figure;margin(Cv*Gv*Hv);grid on;
figure;step(clooptf_v);grid on;


%%%%% W %%%%%%%
kpw=10;
kiw=0.05;
Cw_zpk=zpk(-kiw/kpw,0,kpw);
Gw_zpk=zpk([],[-BI/I],l*r*K/I);
Cw=tf(Cw_zpk);
Gw=tf(Gw_zpk);
Hw=1;
clooptf_w=feedback(Cw*Gw,Hw)
figure;margin(Cw*Gw*Hw);grid on;
figure;step(clooptf_w);grid on;