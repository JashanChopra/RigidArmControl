% ASEN 2003 - Lab 6 - Control Theory with Rotary Position
% 4/10/2019 - Group 5 - Jashan Chopra, Aiden Wilson, Hugo Stetz, Adam
% Elsayed

% This script compares the experimental data versus the predicted model behavior
% using selected values from the rigid arm simulation testing. [Rigid Bar]

clc; clear all; clf;

%% Simulate results with selected Kp/Kd values & plot experimental alongside

  % initial variables
  kG = 33.3;          % [no units]
  kM = .0401;         % [V / rad / sec]
  Jrigid = .002;      % [kg*m^2]
  Jflexible = .0145;  % [kg*m^2]
  Rm = 19.2;          % [ohms] % armature resistance

  kD_test = [.5];            % selected deriv control parameters
  kP_test = [30];            % selected prop control parameters

  numFiles = 1;         % number of files used
  filename = {'Group5Tests_Kp30_Kd05'};

for k = 1:numFiles

          num = (kP_test(k)*kG*kM) / (Jrigid * Rm);       % numerator

          % denominator functions
          d2 = 1;
          d1 = ((kG^2*kM^2) / (Jrigid * Rm)) + ((kD_test(k) * kG * kM) / (Jrigid * Rm));
          d0 = (kP_test(k)*kG*kM) / (Jrigid * Rm);
          den = [d2 d1 d0];

          sysTF = tf(num,den);                    % construct transfer function

          %% Step Response
          [x,t] = step(sysTF);                    % compute step response
          thetad = .3;                            % theta step
          thetaL = 2*thetad*x - thetad;           % scale the step values

          fitmodel = fit(t,thetaL,'cubicinterp');     % alternate method
          thetaLdot = differentiate(fitmodel,t);

          % read data
          [time,theta,thetaDot,posRef,voltage,err,Kp,Kd] = dataRead(filename{k});

          % get the overshoot values
          index = find(theta > posRef);                             % find overshoot
          timeOver = time(index(3)); thetaOver = theta(index(3));   % overshoot
          timeSet = find(time <= .15);                              % settling


          % Plotting
          figure(k)
          plot(time,theta)                % experimental theta
          hold on
          plot(time,posRef,'--')          % experimental position reference
          plot(t,thetaL)                  % theoretical theta
          plot(timeOver,thetaOver,'*')    % overshoot point
          plot(time(timeSet(end)),theta(timeSet(end)),'o')
          titleText = sprintf('Theta over time - Kp: %3.1f - Kd: %1.2f',Kp,Kd);
          title(titleText)
          ylabel('Theta [Rad]')
          xlabel('Time [s]')
          legend('Actual Theta','Reference Pos','Theoretical Pos','Overshoot Point','Settling Time')

          % cut off experimental angular velocity past theoretical
          index = find(time > t(end));
          time(index) = []; thetaDot(index) = [];

          % plotting angular velocity
          figure(k + numFiles)
          plot(time,thetaDot)             % experimental theta dot
          hold on
          plot(t,thetaLdot)            % theoretical
          titleText = sprintf('Angular Velocity - Kp: %3.1f - Kd: %1.2f',Kp,Kd);
          title(titleText)
          ylabel('Angular Velocity [rad/s]')
          xlabel('Time [s]')
          legend('Actual','Experimental')
end
%{
% Plotting
          figure(k)
          yyaxis left
          plot(time,theta)          % experimental theta
          hold on
          plot(time,posRef)         % experimental position reference
          plot(t,thetaL)            % theoretical theta
          titleText = sprintf('Theta over time - Kp: %3.1f - Kd: %1.2f',Kp,Kd');
          title(titleText)
          ylabel('Theta [Rad]')
          xlabel('Time [s]')

          yyaxis right
          plot(time,thetaDot)       % right axis has angular vel
          ylabel('Rotational Velocity [rad/s]')
          legend('Actual Theta','Reference Pos','Theoretical Pos','Angular Velocity')
%}
