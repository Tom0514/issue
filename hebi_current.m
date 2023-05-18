%% clear all
clear;close all;clc

%% device search
HebiLookup.initialize();
disp(HebiLookup);

%% set group
family = 'Arm';
names = 'Elbow';
group = HebiLookup.newGroupFromNames(family, names);

%% set parameters

% Feedback frequency
group.setFeedbackFrequency(1000);

% Command life time
group.setCommandLifetime(0.25);

% Safety parameters
safetyParams = group.getSafetyParams();
HebiUtils.sendWithRetry(group, 'SafetyParams', safetyParams);

% Gain parameters
gains = group.getGains();
gains.controlStrategy = 1;
group.send('gains', gains);
HebiUtils.sendWithRetry(group, 'gains', gains, 'persist', true);

%% The current controller
% PID parameter
kP = 0.2;
kI = 50.0;
kD = 0.0;

duration = 10;

cmd = CommandStruct();
group.startLog( 'dir', 'current' );

fbk = group.getNextFeedbackFull();
tPrev = fbk.hwTxTime;

iError = zeros(1);
lastError = 0 - fbk.motorCurrent;

i = 1;
t0 = tic();
while toc(t0) < duration

    fbk = group.getNextFeedback(fbk);

    t = fbk.hwTxTime;
    dt =  t - tPrev;
    tPrev = t;

    % MPC 100Hz, u(2) is the first control input
    if toc(t0) < duration
        j = floor(toc(t0)/0.01) + 2;
    else
        j = 1001;
    end

    error = u_sim(j) - fbk.motorCurrent;

    pwm = kP .* error;

    iError = iError + error .* dt;
    pwm = pwm + kI .* iError;

    dError = (error - lastError) ./ dt;
    lastError = error;
    pwm = pwm + kD .* dError;
    
    cmd.effort = pwm;                      
    group.send(cmd);

    i = i + 1;

end

log = group.stopLog();
HebiUtils.plotLogs( log, 'motorCurrent' );
