
%% Draw/Render the Scenario
%figh = figure; % figure handle
%figure(3);
figh = figureFullScreen(100);
% txt = "Rocker-Bogie Live Telemetry  \rightarrow   [ Task duration [hh:mm:ss.SSS]: " + string(taskDuration) +...
%     "   -   Distance traveled [m]: " + num2str(distanceTraveled) + " ]";

fps = 25;
videoLength = time(end); %[s]
totCameraPic = ceil(fps*videoLength);

robot = [time,x_opt,y_opt];
robotInterp = interparc(totCameraPic,robot(:,1),robot(:,2),robot(:,3),'linear');

for i = 1 : length(robotInterp)
    index_of_last  = find( robot(:,1) <= robotInterp(i,1), 1, 'last');
    robotInterp(i,4) = theta_opt(index_of_last);
    robotInterp(i,5) = interval_opt(index_of_last);
end
robotInterp(end,5) = interval_opt(end);
robotInterp(end,4) = theta_opt(end);

i=1;
flag_threshold = 0;
count_threshold = 1;

while i<=length(robotInterp)

    % Wipe the slate clean so we are plotting with a blank figure
    clf % clear figure

    %sgtitle(txt)

    axis square, axis equal, grid on, xlabel('x_{1}'), ylabel('x_{2}'),
    abscissae = [min(viaPoint(:,1)-thresholdValue),max(viaPoint(:,1)+thresholdValue)];
    ordinates = [min(viaPoint(:,2)-thresholdValue),max(viaPoint(:,2)+thresholdValue)];
    xlim(abscissae)
    ylim(ordinates)
    hold on

    % Plot threshold
    if robotInterp(i,5) >= 2 && flag_threshold == 0
        count_threshold = count_threshold+1;
        flag_threshold = 1;
        plotCircle(viaPoint(count_threshold,1),viaPoint(count_threshold,2),threshold(2),'r',2);
    elseif robotInterp(i,5) >= 2 && flag_threshold == 1
        plotCircle(viaPoint(count_threshold,1),viaPoint(count_threshold,2),threshold(2),'r',2);
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
        scatter(viaPoint(k,1),viaPoint(k,2),50,'filled','MarkerEdgeColor','r','MarkerFaceColor','r');
        labels(k) = "Point" + " " + num2str(k);
        labelpoints(viaPoint(k,1),viaPoint(k,2),labels(k),'buffer',0.5);
    end

    % Plot robot's pose
    plot_unicycle(robotInterp(i,2),robotInterp(i,3),robotInterp(i,4),wheelBase,wheelWidth,wheelDiam,bodyLength,bodyWidth);
    label = "Robot";
    labelpoints(robotInterp(i,2),robotInterp(i,3),label,'NE','buffer',0.5);


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