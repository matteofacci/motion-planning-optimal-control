
%% Draw/Render the Scenario
%figh = figure; % figure handle
%figure(3);
figh = figureFullScreen(100);
txt = "Motion Plan   \rightarrow   Task duration : " + string(time(end)) + " [s]";
%     "   -   Distance traveled [m]: " + num2str(distanceTraveled) + " ]";

fps = 25;
videoLength = ceil(time(end)); %[s]
totCameraPic = fps*videoLength;

robot = [time,x_opt,y_opt];
input = [u1_opt,u2_opt,vR,vL];

if mod(time(end),1) ~= 0
    robot(end+1,:) = robot(end,:);
    input(end+1,:) = input(end,:);
end

timeInterp = linspace(robot(1,1),robot(end,1),totCameraPic);

interp(:,1) = timeInterp';
interp(:,2) = interpolation(timeInterp,robot(:,1),robot(:,2));
interp(:,3) = interpolation(timeInterp,robot(:,1),robot(:,3));
interp(:,4) = interpolation(timeInterp,robot(:,1),input(:,1));
interp(:,5) = interpolation(timeInterp,robot(:,1),input(:,2));
interp(:,6) = interpolation(timeInterp,robot(:,1),input(:,3));
interp(:,7) = interpolation(timeInterp,robot(:,1),input(:,4));
%interp(:,8) = interpolation(timeInterp,robot(:,1),theta_opt);

% interp = interparc(totCameraPic,robot(:,1),robot(:,2),robot(:,3),...
%     input(:,1),input(:,2),input(:,3),input(:,4),'linear');

for i = 1 : length(interp)-1
    index_of_last  = find( robot(:,1) <= interp(i,1), 1, 'last');
    interp(i,8) = theta_opt(index_of_last);
    interp(i,9) = interval_opt(index_of_last);
end
interp(end,9) = interval_opt(end);
interp(end,8) = theta_opt(end);

i=1;
flag_threshold = 0;
count_threshold = 1;

while i<=length(interp)

    % Wipe the slate clean so we are plotting with a blank figure
    clf % clear figure

    sgtitle(txt)

    %% X coordinates
    
    subplot(5,2,1)

    plot(interp(:,1),interp(:,2),'Color','r');
    hold on
    xline(interp(i,1));
    hold on
    
    grid on

    infY = min(interp(:,2))-mean(interp(:,2))*0.5;
    supY = max(interp(:,2))+mean(interp(:,2))*0.5;
    xlim([interp(1:1) interp(end,1)])
    %ylim([infY supY])
    ylim('padded');

    xlabel('t [s]'), ylabel('X(t) [m]'), title('X coordinates')

    %% Y coordinates
    
    subplot(5,2,3)

    plot(interp(:,1),interp(:,3),'Color','r');
    hold on
    xline(interp(i,1));
    hold on
    
    grid on

    infY = min(interp(:,3))-mean(interp(:,3))*0.5;
    supY = max(interp(:,3))+mean(interp(:,3))*0.5;
    xlim([interp(1:1) interp(end,1)])
    %ylim([infY supY])
    ylim('padded');

    xlabel('t [s]'), ylabel('Y(t) [m]'), title('Y coordinates')

    %% Theta coordinates
    
    subplot(5,2,5)

    plot(interp(:,1),interp(:,8),'Color','r');
    hold on
    xline(interp(i,1));
    hold on
    
    grid on

    infY = min(interp(:,8))-mean(interp(:,8))*0.5;
    supY = max(interp(:,8))+mean(interp(:,8))*0.5;
    xlim([interp(1:1) interp(end,1)])
    %ylim([infY supY])
    ylim('padded');

    xlabel('t [s]'), ylabel('\theta(t) [rad]'), title('\theta coordinates')

    %% Linear velocity input
    
    subplot(5,2,7)

    plot(interp(:,1),interp(:,4),'Color','b');
    hold on
    xline(interp(i,1));
    hold on
    
    grid on

    infY = min(interp(:,4))-mean(interp(:,4))*0.5;
    supY = max(interp(:,4))+mean(interp(:,4))*0.5;
    xlim([interp(1:1) interp(end,1)])
    %ylim([infY supY])
    ylim('padded');

    xlabel('t [s]'), ylabel('u_{1}(t) [m/s]'), title('u_{1}(t) input')

    %% Angular velocity input
    
    subplot(5,2,9)

    plot(interp(:,1),interp(:,5),'Color','b');
    hold on
    xline(interp(i,1));
    hold on
    
    grid on

    infY = min(interp(:,5))-mean(interp(:,5))*0.5;
    supY = max(interp(:,5))+mean(interp(:,5))*0.5;
    xlim([interp(1:1) interp(end,1)])
    %ylim([infY supY])

    ylim('padded');

    xlabel('t [s]'), ylabel('u_{2}(t) [rad/s]'), title('u_{2}(t) input')

    %% Right wheel velocity
    
    subplot(5,2,2)

    plot(interp(:,1),interp(:,6),'Color','b');
    hold on
    xline(interp(i,1));
    hold on
    
    grid on

    infY = min(interp(:,6))-mean(interp(:,6))*0.5;
    supY = max(interp(:,6))+mean(interp(:,6))*0.5;
    xlim([interp(1:1) interp(end,1)])
    %ylim([infY supY])
    ylim('padded');

    xlabel('t [s]'), ylabel('v_{R}(t) [m/s]'), title('Right wheel velocity')

    %% Left wheel velocity
    
    subplot(5,2,4)

    plot(interp(:,1),interp(:,7),'Color','b');
    hold on
    xline(interp(i,1));
    hold on
    
    grid on

    infY = min(interp(:,7))-mean(interp(:,7))*0.5;
    supY = max(interp(:,7))+mean(interp(:,7))*0.5;
    xlim([interp(1:1) interp(end,1)])
    %ylim([infY supY])
    ylim('padded');

    xlabel('t [s]'), ylabel('v_{L}(t) [m/s]'), title('Left wheel velocity')


    %% Robot in motion

    subplot(5,2,[6,8,10])

    axis equal
    grid on
    abscissae = [min(viaPoint(:,1)-thresholdValue),max(viaPoint(:,1)+thresholdValue)];
    ordinates = [min(viaPoint(:,2)-thresholdValue),max(viaPoint(:,2)+thresholdValue)];
    xlim(abscissae)
    ylim(ordinates)
    hold on

    % Plot threshold
    if interp(i,9) >= 2 && flag_threshold == 0
        count_threshold = count_threshold+1;
        flag_threshold = 1;
        plotCircle(viaPoint(count_threshold,1),viaPoint(count_threshold,2),threshold(2),'r',1.5);
    elseif interp(i,9) >= 2 && flag_threshold == 1
        plotCircle(viaPoint(count_threshold,1),viaPoint(count_threshold,2),threshold(2),'r',1.5);
    else
        flag_threshold = 0;
    end

    %     for k = 2:maxPoses
    %         plotCircle(viaPoint(k,1),viaPoint(k,2),threshold(2),'r',2);
    %     end

    % Plot viapoints
    if returnToBase == 1
        limit = maxPoses-1;
    else
        limit = maxPoses;
    end
    for k = 1:limit
        scatter(viaPoint(k,1),viaPoint(k,2),25,'filled','MarkerEdgeColor','r','MarkerFaceColor','r');
        labels(k) = "Point" + " " + num2str(k);
        labelpoints(viaPoint(k,1),viaPoint(k,2),labels(k),'buffer',0.5);
    end

    % Plot robot's pose
    plot_unicycle(interp(i,2),interp(i,3),interp(i,8),wheelBase,wheelWidth,wheelDiam,bodyLength,bodyWidth);
    label = "Robot";
    labelpoints(interp(i,2),interp(i,3),label,'NE','buffer',0.5);

    xlabel('X [m]'), ylabel('Y [m]') %, title('Optimized trajectory')


    movieVector(i) = getframe(figh);

    i = i+1;
end


%% Save the movie

videoName = sprintf('segway_%s', datestr(now,'dd-mm-yyyy HH-MM'));
%myWriter = VideoWriter('curve'); % generate the file curve.avi
myWriter = VideoWriter(videoName, 'MPEG-4'); % generate the file curve.mp4

% If you want only one frame to appear per second, then you just need to
% set this rate in myWriter
%myWriter.FrameRate = 1/(totalTime/length(CP));
myWriter.FrameRate = fps;

% Open the VideoWriter object, write the movie, and close the file
open(myWriter);
writeVideo(myWriter, movieVector);
close(myWriter);