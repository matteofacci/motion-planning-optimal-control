%PROGRAMMA PRINCIPALE PER DETERMINARE IL CONTROLLO OTTIMO IN UN MODELLO
%UNICICLO
%--------------------------------------------------------------------------
clc; clear all; close all; warning off;

%%

global W1 W2 K3 % weights of the functional
global N St interval % simulation parameters
global x0 y0 theta0 x1 y1 theta1 % initial conditions and final orientation
global x y theta u % state and control variables

% Add that folder plus all subfolders to the path.
addpath(genpath('utilities'));
addpath(genpath('videos'));

%% Starting and ending poses by user input

PICK_POSITION = 0;

if PICK_POSITION == 0
    % Starting conditions
    % Device abscissa and ordinate input (in meters)
    x0 = value_from_user('Initial abscissa in meters [0 m]: ',0);
    y0 = value_from_user('Initial ordinate in meters [0 m]: ',0);

    % Device starting orientation input (in degrees)
    theta0_grad = value_from_user('Initial orientation in degrees [0 °]: ',0);
    theta0 = mod(deg2rad(theta0_grad),2*pi); % conversion to radians

    % Device abscissa and ordinate input (in meters)
    x1 = value_from_user('Final abscissa in meters [0 m]: ',0);
    y1 = value_from_user('Final ordinate in meters [0 m]: ',0);

    % Device final orientation input (in degrees)
    theta1_grad = value_from_user('Final orientation in degrees [0 °]: ',0);
    theta1 = mod(deg2rad(theta1_grad),2*pi); % conversion to radians

else
    maxPoses = 2;

    f = figure(1);
    %plot_unicycle(x0,y0,theta0,wheelBase,wheelWidth,wheelDiam,bodyLength,bodyWidth)
    abscissae = [0,20];
    ordinates = [0,20];
    xlim(abscissae)
    ylim(ordinates)
    grid on
    axis equal
    hold on
    position = pickPosition(f,maxPoses);
    hold off

    x0 = position(1,1);
    y0 = position(1,2);
    x1 = position(2,1);
    y1 = position(2,2);

    % Device starting orientation input (in degrees)
    theta0_grad = value_from_user('Initial orientation in degrees [0 °]: ',0);
    theta0 = mod(deg2rad(theta0_grad),2*pi); % conversion to radians
%     % Device final orientation input (in degrees)
%     theta1_grad = value_from_user('Final orientation in degrees [0 °]: ',0);
%     theta1 = deg2rad(theta1_grad); % conversion to radians

    theta1 = computeTheta(x0,y0,x1,y1);
    %theta1 = mod(theta1,2*pi);
end



%% Simulation parameters

simulationParams;

%% Check starting pose and interval with threshold

% Initial interval (normal domain with respect to x axis)

% If the device is in interval 2 (between the two circumferences - close to the goal)
if (x1-x0)^2+(y1-y0)^2<threshold(2)^2 && (x1-x0)^2+(y1-y0)^2>=threshold(3)^2
    interval=2;
    % If it is in the interval 3 (goal pose)
elseif (x1-x0)^2+(y1-y0)^2<threshold(3)^2
    interval=3;
else
    interval=1;
end

if interval==1
    fprintf('The device is initially located in I1.\n\n');
elseif interval==2
    fprintf('The device is initially located in I2.\n\n');
else
    fprintf('The device is already in the desired position.\n');
    fprintf('No optimization needed.\n\n');
end

%% Simulation time and initialization

% Default simulation parameters
t_tot_user=10; % experiment duration in seconds

% Simulation parameters changed by the user
t_tot=input('Experiment duration in seconds [10 s]: ');
if isempty(t_tot)
    t_tot = t_tot_user;
end

n_samples_user = t_tot*2; % at least two samples per second

N=n_samples_user;
St=t_tot/n_samples_user; % sample time

% Generation of the time axis
time=0:St:t_tot; % N+1 elements

% Unnecessary input of fmincon
A=[];
B=[];
Aeq=[];
Beq=[];

% Initialization of state and input vectors
x_opt=x0;
y_opt=y0;
theta_opt=theta0;
u1_opt=[];
u2_opt=[];

completed=false; % flag

% Preparation of vectors x y theta u1 and u2 referred to the entire time
% interval for comparison
zero_vec=[];
x_comp=[];
y_comp=[];
theta_comp=[];
u1_comp=[];
u2_comp=[];

fprintf('\n\n');
disp('START EXPERIMENT');

%% Main loop (FMINCON)

timeInterval=0;
switchingInstant=[0,0]; % vector containing the definitive switch instants

while N>0

    disp('-------------------------------------------------------------')
    timeInterval=timeInterval+1;
    t_start=(n_samples_user-N)*St; % corresponds to time(Ntot-N + 1);
    start_index=n_samples_user-N+1;
    disp(['Starting analysis of time interval ',num2str(timeInterval),' from t_start = ',num2str(t_start),' to t_end = ',num2str(n_samples_user*St)])
    disp(['N = ',num2str(N)])
    disp(['Device in the interval I',num2str(interval)])

    [LB,UB,U0] = input_bounds(N,u1_lb,u1_ub,u2_lb,u2_ub);

    % Current discontinuous term weight
    W1=W1_var(interval);
    W2=W2_var(interval);
    K3=K3_var(interval);

    disp(['Variable weight W1 equal to ',num2str(W1)])
    disp(['Variable weight W2 equal to ',num2str(W2)])

    % Different operation for I1 and I2 and the one below the minimum threshold I3.
    % For all it is necessary to find the input (fmincon), except for interval I3
    % with the input set to zero

    %% Optimal control strategy

    % Target : obtain optimal global variables x,y,theta,u

    if interval<3 % if the device is in the intervals I1 or I2

        % Call function fmincon
        [optim_input,cost,EF,optim_output,lambda]...
            = fmincon('functional',U0,A,B,Aeq,Beq,LB,UB,[],options);
    end

    if interval == 3 % calculate the evolution for null input
        u_null=zeros(1,2*N);
        zeroCost=functional(u_null);
        completed=true;
    end

    % For the current time interval, I find, if it exists,
    % the switching instant corresponding to the achievement of one of the threshold values.

    % N input samples, N + 1 status samples,
    % with the first element equal to the initial status, therefore useless to verify
    L=2;


    while and((x(L)-x1)^2+(y(L)-y1)^2>=threshold(interval+1)^2 ...  % while the device is in the interval ...
            && (x(L)-x1)^2+(y(L)-y1)^2<threshold(interval)^2, L<=N) % length(x)=N+1
        disp(['L = ',num2str(L),'; device still in interval ',num2str(interval)])
        L=L+1; % iterate until you get to the end of the interval
    end

    if L==N+1 % x(N+1)=x(N Tc)
        disp('No (further) switching performed within the time')
        completed=true;
    end

    if and((x(L)-x1)^2+(y(L)-y1)^2<threshold(interval+1)^2, not(completed))
        if interval==1
            switchingInstant(interval)=(L-1)*St+t_start;
            % Calculation of reaction time between threshold and switch
            dSwitch1=((x(L)-x1)^2+(y(L)-y1)^2)^(1/2); % first sample position after the switch
            dThresholdSwitch=threshold(2)-dSwitch1; % distance between the sample and the threshold
            tReaction=dThresholdSwitch/abs(u(L-1)); % reaction time for the switch
        end
        if interval==2
            switchingInstant(interval)=(L-1)*St+t_start;
            % Calculation of total arrival time between threshold and switch
            dSwitch2=((x(L)-x1)^2+(y(L)-y1)^2)^(1/2); % first sample position after the switch
            dSogliaSwitch2=threshold(3)-dSwitch2; % distance between the sample and the final threshold
            tStop=dSogliaSwitch2/abs(u(L-1)); % reaction time for null input
        end
        interval=interval+1;
        disp(['Switching on instant ',num2str((L-1)*St+t_start),'; switch to interval (upper) ',num2str(interval)])
    end

    if and((x(L)-x1)^2+(y(L)-y1)^2>=threshold(interval)^2, not(completed))
        interval=interval-1;
        disp(['L = ',num2str(L),'; switch to interval (lower) ',num2str(interval)])
    end

    t_end=(n_samples_user-N+(L-1))*St;
    end_index=(n_samples_user-N+(L-1)+1);
    disp(['Time interval considered from t_start = ',num2str(t_start),' to t_end = ',num2str(t_end)])

    %% Update of state and input vectors

    [x,y,theta,u,x0,y0,theta0,x_opt,y_opt,theta_opt,u1_opt,u2_opt,x_comp,y_comp,theta_comp,u1_comp,u2_comp] = update_variables(timeInterval,N,L,x,y,theta,u,x_opt,y_opt,theta_opt,u1_opt,u2_opt,zero_vec,x_comp,y_comp,theta_comp,u1_comp,u2_comp);

    % Remaining samples
    N=N-(L-1);
    zero_vec=[zero_vec,zeros(1,L-1)];
end

disp('-------------------------------------------------------------')

%% Simulation results

if switchingInstant(1)~=0
    fprintf('Switching instant I1-->I2: %f s\n',switchingInstant(1));
    fprintf('Time of exceeding the threshold: %f s\n', switchingInstant(1)-tReaction);
    fprintf('Reaction time: %f s\n',tReaction);
    fprintf('Distance traveled between threshold and switch: %f m\n', dThresholdSwitch);
end
if switchingInstant(2)~=0
    fprintf('Total time taken to reach the goal: %f s\n',switchingInstant(2)-tStop);
    fprintf('Experiment duration: %f s\n',switchingInstant(2));
    disp('END.')
end
if switchingInstant(1)~=0 && switchingInstant(2)==0
    fprintf('Not enough time to reach the goal.\nIncrease experiment duration.\nEND.\n');
end
fprintf('-------------------------------------------------------------\n');

% Table with results
if interval == 3 && or(switchingInstant(1)~=0,switchingInstant(2)~=0)
    Tab = build_table(switchingInstant,threshold,time,x_opt,y_opt,theta_opt,u1_opt,u2_opt,x1,y1);
    u1_comp = [u1_comp,[0;0;0]];
    u2_comp = [u2_comp,[0;0;0]];
    disp(Tab);

    %% Plots and video

    plotsFlag = string_from_user('Show plots: Y/N [N]: ','N');
    videoFlag = string_from_user('Create video: Y/N [N]: ','N');

    if or(plotsFlag == 'Y', plotsFlag == 'y')
        plots;
    end

    if or(videoFlag == 'Y', videoFlag == 'y')
        createVideo;
        movefile('*mp4','videos');
    end

end




