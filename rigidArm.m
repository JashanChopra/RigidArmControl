% ASEN 2003 - Lab 6 - Control Theory with Rotary Position
% 4/10/2019 - Group 5 - Jashan Chopra, Aiden Wilson, Hugo Stetz, Adam
% Elsayed

%{
This is the main script for the ASEN2003 Dynamics Lab #6

Script Goals
    1)  develop a MATLAB simulation of the closed loop behavior of the rigid arm ( Eq. 17 )
    2)  determine the parameters for the equations of motion derived
    3)  investigate the behavior of the step response of the Rigid
        Arm system for different values of proportional and derivative gains
    4)  compute gain values for the rigid arm that will have less than 5% overshoot
        and achieve a 5% settling time in less than 0.15 seconds.

    5)  Choose one set of gains that does not overshoot, achieves 5% settling
        time in less than 0.5 seconds, and reduces tip deflection to less than 0.01m
        of residual vibrations. [Same lead up process as rigid arm]
%}
clc; clear all; clf;

%% Define Initial Variables
kG = 33.3;          % [no units]
kM = .0401;         % [V / rad / sec]
Jrigid = .002;      % [kg*m^2]
Jflexible = .0145;  % [kg*m^2]
Rm = 19.2;          % [ohms] % armature resistance

%% Rigid Arm Simulation
    %% Closed Loop Simulation

    %wn = sqrt((kP * kG * kM) / (Jrigid * Rm));                  % natural freq
    %z = (kG^2*kM^2 + kD*kG*kM) / (2*sqrt(kP*kG*kM*Jrigid*Rm));  % damping factor

    numP = 30;                          % number of Kp trials
    kD = linspace(.25,.5,8);            % derivative gain from .25 to .5
    kP = zeros(length(kD),numP);        % proportional gain zeros matrix
    for k = 1:length(kD)                % fill out proportional gain values
        kP(k,:) = linspace(1,50,numP);
    end

    for i = 1:length(kD)                % iterate over deriv gain values
        for j = 1:numP                  % iterate over prop gain values

            num = (kP(i,j)*kG*kM) / (Jrigid * Rm);       % numerator

            % denominator functions
            d2 = 1;
            d1 = ((kG^2*kM^2) / (Jrigid * Rm)) + ((kD(i) * kG * kM) / (Jrigid * Rm));
            d0 = (kP(i,j)*kG*kM) / (Jrigid * Rm);
            den = [d2 d1 d0];

            sysTF = tf(num,den);                        % construct transfer function

            %% Step Response
            [x,t] = step(sysTF);                    % compute step response
            thetad = .3;                            % theta step
            thetaL = 2*thetad*x - thetad;           % scale the step values
            
            fitmodel = fit(t,thetaL,'cubicinterp');     % alternate method 
            thetaLdot = differentiate(fitmodel,t);

            % thetaLdot = diff(thetaL) ./ diff(t);        % derivative of position
            % thetaL(end) = [];                           % remove last value
            % t(end) = [];                                % remove last value

            vIn = kP(i,j)*(thetad-thetaL) + kD(i)*(-thetaLdot); % input voltage

            H = subplot(2,(length(kD)/2),i);                            % create subplot
            textT{i} = sprintf('Step Response : kD = %1.2f',kD(i));     % title text
            textL{j} = sprintf('kP = %2.0f',kP(i,j));                   % legend text
            cutoff = length(find(vIn > 10)) / length(vIn)*100;          % cutoff points
                if cutoff > 20 || (max(thetaL)-thetad)/thetad > 0.05
                    plot(t,thetaL,'--');                                % plot
                    hold on
                    kP(i,j) = NaN;                                      % remove value after plotting
                else
                    plot(t,thetaL)                                      % plot
                end
                hold on
                title('Step Response')                                  % plotting specifics
                xlabel('Time [S]')
                ylabel('Position [Rad]')
                axis([0 .15 -thetad (thetad + .1)])
        end
        title(textT{i})
        % legend(textL{1},textL{2},textL{3},textL{4},textL{5});
    end

%% Results and Analysis

    % pick some values and get the actual data and plot those

    %  plot and compare the experimental results with the model results
    %  [rigid arm]
        % from section 3 part 3
        % and from some other case that we choose to perform
        % label the overshoot and 5% settling time

    %   plot and compare the experimental results with the model results
    %   [flex arm]
