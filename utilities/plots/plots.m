close all

figure, plot(time,x_opt,'r -','linewidth',2), grid on, xlim('tight'),ylim('padded'), xlabel('t [s]'), ylabel('X(t) [m]'), title('X coordinates')
figure, plot(time,y_opt,'r -','linewidth',2), grid on, xlim('tight'),ylim('padded'), xlabel('t [s]'), ylabel('Y(t) [m]'), title('Y coordinates')
figure, plot(time,theta_opt,'r -','linewidth',2), grid on, xlim('tight'),ylim('padded'), xlabel('t [s]'), ylabel('\theta(t) [rad]'), title('\theta coordinates')
figure, plot(time,u1_opt,'b -','linewidth',2), grid on, xlim('tight'),ylim('padded'), xlabel('t [s]'), ylabel('u_{1}(t) [rad/s]'), title('u_{1}(t) input')
figure, plot(time,u2_opt,'b -','linewidth',2), grid on, xlim('tight'),ylim('padded'), xlabel('t [s]'), ylabel('u_{2}(t) [rad/s]'), title('u_{2}(t) input')

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
