% ASEN 2003 - Lab 6 - Control Theory with Rotary Position
% 4/17/2019 - Group 5 - Jashan Chopra, Aiden Wilson, Hugo Stetz, Adam
% Elsayed

%{
This is the main script for the ASEN2003 Dynamics Lab #6

Function Goals
    1)  develop a MATLAB simulation of the closed loop behavior of the rigid arm ( Eq. 17 )
    2)  determine the parameters for the equations of motion derived
    3)  investigate the behavior of the step response of the flexible
        Arm system for different values of proportional and derivative gains
    4)  compute gain values for the rigid arm that will have less than 5% overshoot
        and achieve a 5% settling time in less than 0.15 seconds.

    5)  Choose one set of gains that does not overshoot, achieves 5% settling
        time in less than 0.5 seconds, and reduces tip deflection to less than 0.01m
        of residual vibrations. [Same lead up process as rigid arm]

    % Equations to look up: tf, step, lsim

%}
function [t1,thetaL,def]=lab6_flex(k1,k2,k3,k4)

%% Define Initial Variables
kG = 33.3;          % [no units]
kM = .0401;         % [V / rad / sec]
fc=1.8;             % Hz
Jflexible = .0145;  % [kg*m^2]
JL=.014;
Jhub=0.0005;
kArm=JL*(2*pi*fc)^2;
Rm = 19.2;          % [ohms] % armature resistance
L=0.45;             % m
%Contstants
p1=-kG^2*kM^2/(Jhub*Rm);
q1=kArm/(L*Jhub);
r1=kG*kM/(Jhub*Rm);
p2=kG^2*kM^2*L/(Jhub*Rm);
q2=-kArm*(Jhub+JL)/(JL*Jhub);
r2=-kG*kM*L/(Jhub*Rm);

%Create lambda values
lambda0=k1*(q1*r2-r1*q2);
lambda1=p1*q2-q1*p2+k3*(q1*r2-r1*q2)+k2*(p2*r1-r2*p1);
lambda2=-q2+k1*r1+k2*r2+k4*(p2*r1-r2*p1);
lambda3=-p1+k3*r1+k4*r2;
%% Flexible Arm Simulation - Desired theta
num1 = [k1*r1 0 k1*(q1*r2-r1*q2)];       % numerator

% denominator functions
d4 = 1;
d3 = lambda3;
d2 = lambda2;
d1 = lambda1;
d0 = lambda0;
den = [d4 d3 d2 d1 d0]; %den is the same for both transfer functions

sysTF1 = tf(num1,den);                        % construct transfer function

% Step Response
[x1,t1] = step(sysTF1);                    % compute step response
thetad = .3;                            % theta step
thetaL = 2*thetad*x1 - thetad;           % scale the step values

%Plot theta vs time
figure(1)
plot(t1,thetaL)
xlabel('Time (s)'); ylabel('Angular Position (rad)');
title('Flexible Arm Control: Angular Position');


%% Flexible Arm Simulation - Deflection rate
num2 = [k2*r2 k2*(p2*r1-r2*p1) 0];

sysTF2 = tf(num2,den);

% Step Response
[def,t2] = step(sysTF2);                    % compute step response

%Plot theta vs time
figure(2)
plot(t2,def)
xlabel('Time (s)'); ylabel('Tip Deflection (m)');
title('Flexible Arm Control: Tip Deflection');

end
