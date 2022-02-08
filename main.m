%PROGRAMMA PRINCIPALE PER DETERMINARE IL CONTROLLO OTTIMO IN UN MODELLO
%UNICICLO
%--------------------------------------------------------------------------
clc; clear all; close all; warning off;

%%

global W1 W2 % weights of the functional
global N St interval % simulation parameters
global x0 y0 theta0 x1 y1 theta1 % initial conditions and final orientation
global x y theta u % state and control variables

% Add that folder plus all subfolders to the path.
addpath(genpath('utilities'));
addpath(genpath('videos'));

%% Starting and ending poses by user input

% Starting conditions
% Device abscissa and ordinate input (in meters)
x0 = input('Initial abscissa in meters [0 m]: '); 
if isempty(x0)
    x0 = 0;
end

y0 = input('Initial ordinate in meters [0 m]: ');
if isempty(y0)
    y0 = 0;
end

% Device starting orientation input (in degrees)
theta0_grad = input('Initial orientation in degrees [0 °]: ');
if isempty(theta0_grad)
    theta0_grad = 0;
end
theta0 = deg2rad(theta0_grad); % conversion to radians

% Ending conditions
% Device abscissa and ordinate input (in meters)
x0 = input('Final abscissa in meters [0 m]: '); 
if isempty(x0)
    x1 = 0;
end

y0 = input('Final ordinate in meters [0 m]: ');
if isempty(y0)
    y1 = 0;
end

% Device final orientation input (in degrees)
theta1_grad = input('Final orientation in degrees [0 °]: ');
if isempty(theta1_grad)
    theta1_grad = 0;
end
theta1 = deg2rad(theta1_grad); % conversion to radians

%% Simulation parameters

simulationParams;

%% Check starting pose and interval with threshold

% Initial interval (normal domain with respect to x axis)

% If the device is in interval 2 (between the two circumferences - close to the goal)
if x0^2+y0^2<threshold(2)^2 && x0^2+y0^2>=threshold(3)^2
    interval=2;
% If it is in the interval 3 (goal pose)
elseif x0^2+y0^2<threshold(3)^2
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
    
    % Initialization of the quantities dependent on N
    % Definition of dimension and constraints on input for fmincon
    one_vec=ones(1,N); % creates vector of unit values
    u1_min=u1_lb*one_vec;
    %if intervallo==1
    %    U1min=U1inf(1)*vettore; 
    %else
    %    U1min=U1inf(2)*vettore;
    %end
    u1_max=u1_ub*one_vec;
    u2_min=u2_lb*one_vec;
    u2_max=u2_ub*one_vec;

    LB=horzcat(u1_min,u2_min); % vector of size 2N
    UB=horzcat(u1_max,u2_max);
    U0=0.2*UB; % initial input not null but low enough
    
    % Current discontinuous term weight
    W1=W1_var(interval);
    W2=W2_var(interval);

    disp(['Variable weight W1 equal to ',num2str(W1)])
    disp(['Variable weight W2 equal to ',num2str(W2)])
    
    % Different operation for I1 and I2 and the one below the minimum threshold I3.
    % For all it is necessary to find the input (fmincon), except for interval I3 
    % with the input set to zero

    %% Optimal control strategy

    % Target : obtain optimal global variables x,y,theta,u

    if interval<3 % if the device is in the intervals I1 or I2  
        
        % Call function fmincon
        options = optimoptions('fmincon','Display','off',...
            'ConstraintTolerance',1e-12,'StepTolerance',1e-10,'MaxFunctionEvaluations',1e10, ...
            'OptimalityTolerance',1e-12);

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
    
   
    while and(x(L)^2+y(L)^2>=threshold(interval+1)^2 ...  % while the device is in the interval ...
            && x(L)^2+y(L)^2<threshold(interval)^2, L<=N) % length(x)=N+1
        disp(['L = ',num2str(L),'; device still in interval ',num2str(interval)])
        L=L+1; % iterate until you get to the end of the interval
    end
    
    if L==N+1 % x(N+1)=x(N Tc)
        disp('No (further) switching performed within the time')
        completed=true;
    end
    
    if and(x(L)^2+y(L)^2<threshold(interval+1)^2, not(completed))
        if interval==1
            switchingInstant(interval)=(L-1)*St+t_start;
            % Calculation of reaction time between threshold and switch
            dSwitch1=(x(L)^2+y(L)^2)^(1/2); % first sample position after the switch
            dThresholdSwitch=threshold(2)-dSwitch1; % distance between the sample and the threshold
            tReaction=dThresholdSwitch/abs(u(L-1)); % reaction time for the switch
        end
        if interval==2
            switchingInstant(interval)=(L-1)*St+t_start;
            % Calculation of total arrival time between threshold and switch
            dSwitch2=(x(L)^2+y(L)^2)^(1/2); % first sample position after the switch
            dSogliaSwitch2=threshold(3)-dSwitch2; % distance between the sample and the final threshold
            tStop=dSogliaSwitch2/abs(u(L-1)); % reaction time for null input
        end
        interval=interval+1;
        disp(['Switching on instant ',num2str((L-1)*St+t_start),'; switch to interval (upper) ',num2str(interval)])
    end
    
    if and(x(L)^2+y(L)^2>=threshold(interval)^2, not(completed))
        interval=interval-1;
        disp(['L = ',num2str(L),'; switch to interval (lower) ',num2str(interval)])
    end
    
    t_end=(n_samples_user-N+(L-1))*St;
    end_index=(n_samples_user-N+(L-1)+1);
    disp(['Time interval considered from t_start = ',num2str(t_start),' to t_end = ',num2str(t_end)])
    
    %% Update of state and input vectors
    
    % Subarray of the state referred to the interval, with the initial state 
    % already contained in the previous subarray as the final state
    x_temp=x(2:L);
    y_temp=y(2:L);
    theta_temp=theta(2:L);
    
    % Concatenation from the initial state
    x_temp=[x_opt,x_temp];
    y_temp=[y_opt,y_temp];
    theta_temp=[theta_opt,theta_temp];
    
    % Subarray of the inputs referred to the interval
    u1_temp=u(1:L-1);
    u2_temp=u(N+1:N+L-1);
    
    % Concatenation of the input from the start
    u1_temp=[u1_opt,u1_temp];
    u2_temp=[u2_opt,u2_temp];
    
    % Evolution until the end, for comparison
    x_comp(timeInterval,:)=[zero_vec,x]; % extracts the time interval row from the matrix
    y_comp(timeInterval,:)=[zero_vec,y];
    theta_comp(timeInterval,:)=[zero_vec,theta];
        
    % Input extended to the whole interval
    u1_comp(timeInterval,:)=[zero_vec,u(1:N)];
    u2_comp(timeInterval,:)=[zero_vec,u(N+1:2*N)];
    
    % Update the variables to repeat the loop

    x_opt=x_temp;
    y_opt=y_temp;
    theta_opt=theta_temp;

    u1_opt=u1_temp;
    u2_opt=u2_temp;
    
    % New starting conditions
    x0=x_opt(length(x_opt));
    y0=y_opt(length(y_opt));
    theta0=theta_opt(length(theta_opt));
    
    % Remaining samples
    N=N-(L-1);
    zero_vec=[zero_vec,zeros(1,L-1)];
    
end

disp('-------------------------------------------------------------')

%% Simulation results

% Table with results
if switchingInstant(1)~=0 && switchingInstant(2)~=0
    j=1;
    while time(j)<=switchingInstant(2)
        if x_opt(j)^2+y_opt(j)^2 >= threshold(2)^2
            Interval(j,1) = 1;
        elseif x_opt(j)^2+y_opt(j)^2 <= threshold(3)^2
            Interval(j,1) = 3;
        else
            Interval(j,1) = 2;
        end
        Time(j,1) = time(j);
        X1(j,1) = x_opt(j);
        X2(j,1) = y_opt(j);
        Theta_Radians(j,1) = theta_opt(j);
        k=floor(rad2deg(theta_opt(j))/360); % floor arrotonda il rapporto all'intero inferiore
        Theta_Degrees(j,1) = rad2deg(theta_opt(j))-k*360; % angolo equivalente in gradi
        V_Linear(j,1) = u1_opt(j);
        V_Angular(j,1) = u2_opt(j);
        j=j+1;
    end
    
    table(Time,Interval,X1,X2,Theta_Radians,Theta_Degrees,v_linear,v_angular)
end

disp('-------------------------------------------------------------')
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
else
    fprintf('Not enough time to reach the goal.\nIncrease experiment duration.\nEND.');
end
    fprintf('-------------------------------------------------------------\n');

%% Plots and video

prompt = 'Show plots Y/N [N]: ';
plotsFlag = input(prompt,'s');
if isempty(plotsFlag)
    plotsFlag = 'N';
end

prompt = 'Create video: Y/N [N]: ';
videoFlag = input(prompt,'s');
if isempty(videoFlag)
    videoFlag = 'N';
end

if or(plotsFlag == 'Y', plotsFlag == 'y')
    plots;
end

if or(videoFlag == 'Y', videoFlag == 'y')
    createVideo;
    movefile('*mp4','videos');
end


