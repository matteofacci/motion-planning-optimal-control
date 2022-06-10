clc; clear all; close all; warning off;

%%

global W1 W2 K3 % weights of the functional
global N St interval % simulation parameters
global x0 y0 theta0 x1 y1 theta1 % initial conditions and final orientation
global x y theta u % state and control variables

% Add that folder plus all subfolders to the path.
addpath(genpath('utilities'));
addpath(genpath('videos'));

PICK_POSITION = 1;
maxPoses = 10;
returnToBase = 1;
printFigures = 0; % Set 1 for printing figures in eps format


%% Starting and ending poses by user input

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

    f = figure(1);
    abscissae = [0,35];
    ordinates = [0,35];
    xlim(abscissae)
    ylim(ordinates)
    grid on
    axis equal
    hold on

    viaPoint = pickPosition(f,maxPoses,'r',"Point");

    % Device starting orientation input (in degrees)
    theta0_grad = 0;
    theta0 = mod(deg2rad(theta0_grad),2*pi); % conversion to radians

    viaPoint(end+1,:) = viaPoint(1,:);
    viaPoint(1,3)=theta0;
    for i = 2 : length(viaPoint(:,1))-1
        viaPoint(i,3) = computeTheta(viaPoint(i+1,1),viaPoint(i+1,2),viaPoint(i,1),viaPoint(i,2));
    end

    if returnToBase == 0
        viaPoint(end,:) = [];
    end

    [maxPoses,numCols] = size(viaPoint);

end


%% Simulation parameters

simulationParams;

grid on
axis equal
for i = 1:maxPoses
    plot_unicycle(viaPoint(i,1),viaPoint(i,2),viaPoint(i,3),wheelBase,wheelWidth,wheelDiam,bodyLength,bodyWidth)
end

if printFigures == 1
    print('figure_13','-depsc2','-r300'); % saves the file in eps format with name
end

hold off

pause(0.5);

fprintf('\n');
disp('START EXPERIMENT');

for seq = 1:maxPoses-1

    %% Check starting pose and interval with threshold

    if seq == 1
        x_start = viaPoint(seq,1);
        y_start = viaPoint(seq,2);
        theta_start = viaPoint(seq,3);
    else
        x_start = table2array(Tab{seq-1}(end,'X'));
        y_start = table2array(Tab{seq-1}(end,'Y'));
        theta_start = table2array(Tab{seq-1}(end,'Theta_Radians'));
    end

    x_final = viaPoint(seq+1,1);
    y_final = viaPoint(seq+1,2);
    theta_final = viaPoint(seq+1,3);

    % Calculate the threshold corresponding to the distance that separates the
    % device from the goal
    distance0=sqrt((x_final-x_start)^2+(y_final-y_start)^2);
    % fprintf('Distance from the goal: %f m\n', distance0);

    threshold(1)=1000*distance0; % for simplicity, increase the threshold of the I1 interval as desired
    % Predetermined thresholds based on the problem
    threshold(2)=thresholdValue; % functional switch
    threshold(3)=0.5; % value of the switching threshold 3 (for u_null) [m]
    threshold(4)=0; % additional threshold


    % Initial interval
    % If the device is in interval 2 (between the two circumferences - close to the goal)
    if (x_final-x_start)^2+(y_final-y_start)^2<threshold(2)^2 && (x_final-x_start)^2+(y_final-y_start)^2>=threshold(3)^2
        interval=2;
        % If it is in the interval 3 (goal pose)
    elseif (x_final-x_start)^2+(y_final-y_start)^2<threshold(3)^2
        interval=3;
    else
        interval=1;
    end

    startingInterval = interval;

    %% Simulation time and initialization

    t_tot_user = 0;
    totExperiments = 20;
    attempt = 1;
    step = 1;

    while attempt <= totExperiments

    switchingInstant=[0,0]; % vector containing the definitive switch instants
    interval = startingInterval;

        while switchingInstant(2) == 0 && interval ~= 3

            x0 = x_start;
            y0 = y_start;
            theta0 = theta_start;
            x1 = x_final;
            y1 = y_final;
            theta1 = theta_final;

            % Default simulation parameters
            t_tot_user = t_tot_user+step;

            t_tot = t_tot_user;
            % n_samples_user = t_tot; % one sample per second
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

            %% Main loop (FMINCON)

            while N>0

                t_start=(n_samples_user-N)*St; % corresponds to time(Ntot-N + 1);
                start_index=n_samples_user-N+1;

                [LB,UB,U0] = input_bounds(N,u1_lb,u1_ub,u2_lb,u2_ub);

                % Current discontinuous term weight
                W1=W1_var(interval);
                W2=W2_var(interval);
                K3=K3_var(interval);

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
                    L=L+1; % iterate until you get to the end of the interval
                end

                if L==N+1 % x(N+1)=x(N Tc)
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
                end

                if and((x(L)-x1)^2+(y(L)-y1)^2>=threshold(interval)^2, not(completed))
                    interval=interval-1;
                end

                t_end=(n_samples_user-N+(L-1))*St;
                end_index=(n_samples_user-N+(L-1)+1);

                %% Update of state and input vectors

                [x,y,theta,u,x0,y0,theta0,x_opt,y_opt,theta_opt,u1_opt,u2_opt] = update_variables(N,L,x,y,theta,u,x_opt,y_opt,theta_opt,u1_opt,u2_opt);

                % Remaining samples
                N=N-(L-1);
            end


        end
        evaluation_matrix{attempt,1,:} = time;
        evaluation_matrix{attempt,2,:} = x_opt;
        evaluation_matrix{attempt,3,:} = y_opt;
        evaluation_matrix{attempt,4,:} = theta_opt;
        evaluation_matrix{attempt,5,:} = u1_opt;
        evaluation_matrix{attempt,6,:} = u2_opt;
        evaluation_matrix{attempt,7,:} = switchingInstant;
        evaluation_matrix{attempt,8,:} = tReaction;
        evaluation_matrix{attempt,9,:} = dThresholdSwitch;
        evaluation_matrix{attempt,10,:} = tStop;
        evaluation_matrix{attempt,11,:} = x_final;
        evaluation_matrix{attempt,12,:} = y_final;
        evaluation_matrix{attempt,13,:} = theta_final;

        attempt = attempt+1;

    end

    [standing_matrix{seq},switchingInstant,time,x_opt,y_opt,theta_opt,u1_opt,u2_opt,tReaction,dThresholdSwitch,tStop] = overall_standings(evaluation_matrix);

    disp('-------------------------------------------------------------')

    %% Simulation results

    if switchingInstant(1)~=0
        fprintf('Sequence %d\n', seq);
        fprintf('Switching instant I1-->I2: %f s\n',switchingInstant(1));
        fprintf('Time of exceeding the threshold: %f s\n', switchingInstant(1)-tReaction);
        fprintf('Reaction time: %f s\n',tReaction);
        fprintf('Distance traveled between threshold and switch: %f m\n', dThresholdSwitch);
    end
    if switchingInstant(2)~=0
        fprintf('Total time taken to reach the goal: %f s\n',switchingInstant(2)-tStop);
        fprintf('Experiment duration: %f s\n',switchingInstant(2));
    end

    fprintf('-------------------------------------------------------------\n');


    % Table with results
    if interval == 3 && or(switchingInstant(1)~=0,switchingInstant(2)~=0)
        Tab{seq} = build_table(switchingInstant,threshold,time,x_opt,y_opt,theta_opt,u1_opt,u2_opt,x1,y1);

        disp(Tab{seq});
    end

end

%% Data manipulation

[M_opt,T_opt] = access_data(Tab);

time = M_opt(:,1);
x_opt = M_opt(:,3);
y_opt = M_opt(:,4);
theta_opt = M_opt(:,5);
u1_opt = M_opt(:,7);
u2_opt = M_opt(:,8);
interval_opt = M_opt(:,2);

for i = 1:length(u1_opt)
    [vR(i),vL(i)] = wheels_velocities(u1_opt(i),u2_opt(i),d);
end

vR = vR';
vL = vL';

fprintf('-------------------------------------------------------------\n');
fprintf('OPTIMAL RESULT\n\n');

disp(T_opt);

disp('END')

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

