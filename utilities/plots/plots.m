%close all

color(1,:) = [0, 0.4470, 0.7410];
color(2,:) = [0.8500, 0.3250, 0.0980];
color(3,:) = [0.4660 0.6740 0.1880];

figure, plot(time,x_opt,'Color',color(2,:),'LineStyle','-','linewidth',2), grid on, xlim('tight'),ylim('padded'), xlabel('t [s]'), ylabel('X(t) [m]'), title('X coordinates')

if printFigures == 1
    print('figure_17','-depsc2','-r300'); % saves the file in eps format with name
end

figure, plot(time,y_opt,'Color',color(2,:),'LineStyle','-','linewidth',2), grid on, xlim('tight'),ylim('padded'), xlabel('t [s]'), ylabel('Y(t) [m]'), title('Y coordinates')

if printFigures == 1
    print('figure_18','-depsc2','-r300'); % saves the file in eps format with name
end

figure, plot(time,theta_opt,'Color',color(2,:),'LineStyle','-','linewidth',2), grid on, xlim('tight'),ylim('padded'), xlabel('t [s]'), ylabel('\theta(t) [rad]'), title('\theta coordinates')

if printFigures == 1
    print('figure_19','-depsc2','-r300'); % saves the file in eps format with name
end

figure, plot(time,u1_opt,'Color',color(1,:),'LineStyle','-','linewidth',2), grid on, xlim('tight'),ylim('padded'), xlabel('t [s]'), ylabel('u_{1}(t) [rad/s]'), title('u_{1}(t) input')

if printFigures == 1
    print('figure_15','-depsc2','-r300'); % saves the file in eps format with name
end

figure, plot(time,u2_opt,'Color',color(1,:),'LineStyle','-','linewidth',2), grid on, xlim('tight'),ylim('padded'), xlabel('t [s]'), ylabel('u_{2}(t) [rad/s]'), title('u_{2}(t) input')

if printFigures == 1
    print('figure_16','-depsc2','-r300'); % saves the file in eps format with name
end

% Trajectory
figure,plot(x_opt,y_opt,'k --','linewidth',1),...
    axis square, axis equal, grid on, xlim('padded'),ylim('padded'), xlabel('X [m]'), ylabel('Y [m]'), title('Optimized trajectory')
hold on
for i = 1:length(x_opt)
    plot_unicycle(x_opt(i),y_opt(i),theta_opt(i),wheelBase,wheelWidth,wheelDiam,bodyLength,bodyWidth)
end

for k = 2:maxPoses
    plotCircle(viaPoint(k,1),viaPoint(k,2),threshold(2),'r',1.5);
end

if printFigures == 1
    print('figure_14','-depsc2','-r300'); % saves the file in eps format with name
end
