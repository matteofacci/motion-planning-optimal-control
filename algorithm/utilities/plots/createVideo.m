
%% Draw/Render the Scenario
%figh = figure; % figure handle
%figure(3);
figh = figureFullScreen(100);
txt = "Motion Plan   \rightarrow   Task duration : " + string(max(time)) + " [s]";
%     "   -   Distance traveled [m]: " + num2str(distanceTraveled) + " ]";

fps = 15;
videoLength = ceil(time(end)); %[s]
totCameraPic = fps*videoLength;

%time = table2array(Tab(:,1));
robot = [table2array(Tab(:,1)),table2array(Tab(:,3)),table2array(Tab(:,4)),table2array(Tab(:,5))];

timeInterp = linspace(robot(1,1),robot(end,1),totCameraPic);

interp(:,1) = timeInterp';
interp(:,2) = interpolation(timeInterp,robot(:,1),robot(:,2));
interp(:,3) = interpolation(timeInterp,robot(:,1),robot(:,3));

for i = 1 : length(interp)-1
    index_of_last  = find( robot(:,1) <= interp(i,1), 1, 'last');
    interp(i,4) = robot(index_of_last,4);
end
interp(end,4) = robot(end,4);
    
i=1;

while i<=length(interp)

    % Wipe the slate clean so we are plotting with a blank figure
    clf % clear figure

    sgtitle(txt)

    axis equal
    grid on
    abscissae = [min(robot(1,2),x1)-threshold(2),max(robot(1,2),x1)+threshold(2)];
    ordinates = [min(robot(1,3),y1)-threshold(2),max(robot(1,3),y1)+threshold(2)];
    xlim(abscissae)
    ylim(ordinates)

    hold on

 
    % Plot threshold
    n=0:0.01:2*pi;
    plot(x1+threshold(2)*cos(n), y1 + threshold(2)*sin(n),'r -','linewidth',1.5)

    % Plot robot's pose
    plot_unicycle(interp(i,2),interp(i,3),interp(i,4),wheelBase,wheelWidth,wheelDiam,bodyLength,bodyWidth);
    label = "Robot";
    labelpoints(interp(i,2),interp(i,3),label,'NE','buffer',0.5);

    xlabel('X [m]'), ylabel('Y [m]'), %title('Optimized trajectory')

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