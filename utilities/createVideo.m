
%% Draw/Render the Scenario
%figh = figure; % figure handle
%figure(3);
figh = figureFullScreen(100);
% txt = "Rocker-Bogie Live Telemetry  \rightarrow   [ Task duration [hh:mm:ss.SSS]: " + string(taskDuration) +...
%     "   -   Distance traveled [m]: " + num2str(distanceTraveled) + " ]";

fps = 25;
videoLength = time(end); %[s]
totCameraPic = ceil(fps*videoLength);

% for j = 1:length(theta_opt)
%     k=floor(rad2deg(theta_opt(j))/360); % floor arrotonda il rapporto all'intero inferiore
%     degrees(j) = rad2deg(theta_opt(j))-k*360; % angolo equivalente in gradi
% end

% robot = [tempo',x_opt',y_opt',deg2rad(degrees')];
% robotInterp = interparc(totCameraPic,robot(:,1),robot(:,2),robot(:,3),robot(:,4),'linear');

robot = [time,x_opt,y_opt];
robotInterp = interparc(totCameraPic,robot(:,1),robot(:,2),robot(:,3),'linear');

for i = 1 : length(robotInterp)
    index_of_last  = find( robot(:,1) <= robotInterp(i,1), 1, 'last');
    robotInterp(i,4) = theta_opt(index_of_last);
end

robotInterp(end,4) = theta_opt(end);

i=1;
while i<=length(robotInterp)

    % Wipe the slate clean so we are plotting with a blank figure
    clf % clear figure

    %sgtitle(txt)

    axis square, axis equal, grid on, xlabel('x_{1}'), ylabel('x_{2}'),
    hold on
    for k = 2:maxPoses
        plotCircle(viaPoint(k,1),viaPoint(k,2),threshold(2),'r',2);
    end
    %     abscissae = [-20,20];
    %     ordinates = [-20,20];
    %     xlim(abscissae)
    %     ylim(ordinates)
    plot_unicycle(robotInterp(i,2),robotInterp(i,3),robotInterp(i,4),wheelBase,wheelWidth,wheelDiam,bodyLength,bodyWidth);
    % Plot della threshold
    %     for i = 2:length(viaPoint)
    %         n=0:0.01:2*pi;
    %         plot(viaPoint(i,1)+threshold(2)*cos(n), viaPoint(i,2) + threshold(2)*sin(n),'r -','linewidth',2)
    %     end
    %legend({'position error'},'Location', 'Best','Orientation','horizontal')

    % force Matlab to draw the image at this point (use drawnow or pause)
    % drawnow
    %pause(0.2)
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