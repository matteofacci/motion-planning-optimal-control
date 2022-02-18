close all

% set(0, 'DefaultTextInterpreter', 'tex')
% set(0, 'DefaultLegendInterpreter', 'tex')
% set(gca, 'TickLabelInterpreter', 'tex')

for i = 1 : interval
    index_of_first(i)  = find( x_comp(i,:) ~= 0, 1, 'first'); 
end

color(1,:) = [0, 0.4470, 0.7410];
color(2,:) = [0.8500, 0.3250, 0.0980];
color(3,:) = [0.4660 0.6740 0.1880];

figure, plot(time,x_opt,'Color',color(2,:),'LineStyle','-','linewidth',2), grid on, xlim('tight'),ylim('padded'), xlabel('t [s]'), ylabel('X(t) [m]'), title('X coordinates')
figure, plot(time,y_opt,'Color',color(2,:),'LineStyle','-','linewidth',2), grid on, xlim('tight'),ylim('padded'), xlabel('t [s]'), ylabel('Y(t) [m]'), title('Y coordinates')
figure, plot(time,theta_opt,'Color',color(2,:),'LineStyle','-','linewidth',2), grid on, xlim('tight'),ylim('padded'), xlabel('t [s]'), ylabel('\theta(t) [rad]'), title('\theta coordinates')
figure, plot(time(1:n_samples_user),u1_opt,'Color',color(1,:),'LineStyle','-','linewidth',2), grid on, xlim('tight'),ylim('padded'), xlabel('t [s]'), ylabel('u_{1}(t) [rad/s]'), title('u_{1}(t) input')
figure, plot(time(1:n_samples_user),u2_opt,'Color',color(1,:),'LineStyle','-','linewidth',2), grid on, xlim('tight'),ylim('padded'), xlabel('t [s]'), ylabel('u_{2}(t) [rad/s]'), title('u_{2}(t) input')

figure, grid on, xlim('tight'),ylim('padded'),xlabel('t [s]'), ylabel('X(t) [m]'), title('X coordinates (comparison)')
hold on
plot(time,x_comp(1,:),'Color',color(1,:),'LineStyle','--','linewidth',1.5,'HandleVisibility','off')
plot(time,x_comp(2,:),'Color',color(2,:),'LineStyle','--','linewidth',1.5,'HandleVisibility','off')
plot(time,x_comp(3,:),'Color',color(3,:),'LineStyle','--','linewidth',1.5,'HandleVisibility','off')

plot(time(index_of_first(1):index_of_first(2)),x_comp(1,index_of_first(1):index_of_first(2)),...
    'Color',color(1,:),'LineStyle','-','linewidth',2)
plot(time(index_of_first(2):index_of_first(3)),x_comp(2,index_of_first(2):index_of_first(3)),...
    'Color',color(2,:),'LineStyle','-','linewidth',2)
plot(time(index_of_first(3):end),x_comp(3,index_of_first(3):end),...
    'Color',color(3,:),'LineStyle','-','linewidth',2)

legend ('Interval 1','Interval 2','Null input','best')
xline(time(index_of_first),'-',{'Interval 1','Interval 2', 'Null input'},...
    'LabelOrientation','aligned','HandleVisibility','off')

%%

figure, grid on, xlim('tight'),ylim('padded'),xlabel('t [s]'), ylabel('Y(t) [m]'), title('Y coordinates (comparison)')
hold on
plot(time,y_comp(1,:),'Color',color(1,:),'LineStyle','--','linewidth',1.5,'HandleVisibility','off')
plot(time,y_comp(2,:),'Color',color(2,:),'LineStyle','--','linewidth',1.5,'HandleVisibility','off')
plot(time,y_comp(3,:),'Color',color(3,:),'LineStyle','--','linewidth',1.5,'HandleVisibility','off')

plot(time(index_of_first(1):index_of_first(2)),y_comp(1,index_of_first(1):index_of_first(2)),...
    'Color',color(1,:),'LineStyle','-','linewidth',2)
plot(time(index_of_first(2):index_of_first(3)),y_comp(2,index_of_first(2):index_of_first(3)),...
    'Color',color(2,:),'LineStyle','-','linewidth',2)
plot(time(index_of_first(3):end),y_comp(3,index_of_first(3):end),...
    'Color',color(3,:),'LineStyle','-','linewidth',2)

legend ('Interval 1','Interval 2','Null input','best')
xline(time(index_of_first),'-',{'Interval 1','Interval 2', 'Null input'},...
    'LabelOrientation','aligned','HandleVisibility','off')

%%

figure, grid on, xlim('tight'),ylim('padded'),xlabel('t [s]'), ylabel('\theta(t) [rad]'), title('\theta coordinates (comparison)')
hold on
plot(time,theta_comp(1,:),'Color',color(1,:),'LineStyle','--','linewidth',1.5,'HandleVisibility','off')
plot(time,theta_comp(2,:),'Color',color(2,:),'LineStyle','--','linewidth',1.5,'HandleVisibility','off')
plot(time,theta_comp(3,:),'Color',color(3,:),'LineStyle','--','linewidth',1.5,'HandleVisibility','off')

plot(time(index_of_first(1):index_of_first(2)),theta_comp(1,index_of_first(1):index_of_first(2)),...
    'Color',color(1,:),'LineStyle','-','linewidth',2)
plot(time(index_of_first(2):index_of_first(3)),theta_comp(2,index_of_first(2):index_of_first(3)),...
    'Color',color(2,:),'LineStyle','-','linewidth',2)
plot(time(index_of_first(3):end),theta_comp(3,index_of_first(3):end),...
    'Color',color(3,:),'LineStyle','-','linewidth',2)

legend ('Interval 1','Interval 2','Null input','best')
xline(time(index_of_first),'-',{'Interval 1','Interval 2', 'Null input'},...
    'LabelOrientation','aligned','HandleVisibility','off')

%%

figure, grid on, xlim('tight'),ylim('padded'),xlabel('t [s]'), ylabel('u_{1}(t) [rad/s]'), title('u_{1}(t) input (comparison)')
hold on
plot(time,u1_comp(1,:),'Color',color(1,:),'LineStyle','--','linewidth',1.5,'HandleVisibility','off')
plot(time,u1_comp(2,:),'Color',color(2,:),'LineStyle','--','linewidth',1.5,'HandleVisibility','off')
plot(time,u1_comp(3,:),'Color',color(3,:),'LineStyle','--','linewidth',1.5,'HandleVisibility','off')

plot(time(index_of_first(1):index_of_first(2)),u1_comp(1,index_of_first(1):index_of_first(2)),...
    'Color',color(1,:),'LineStyle','-','linewidth',2)
plot(time(index_of_first(2):index_of_first(3)),u1_comp(2,index_of_first(2):index_of_first(3)),...
    'Color',color(2,:),'LineStyle','-','linewidth',2)
plot(time(index_of_first(3):end),u1_comp(3,index_of_first(3):end),...
    'Color',color(3,:),'LineStyle','-','linewidth',2)

legend ('Interval 1','Interval 2','Null input','best')
xline(time(index_of_first),'-',{'Interval 1','Interval 2', 'Null input'},...
    'LabelOrientation','aligned','HandleVisibility','off')

%%

figure, grid on, xlim('tight'),ylim('padded'),xlabel('t [s]'), ylabel('u_{2}(t) [rad/s]'), title('u_{2}(t) input (comparison)')
hold on
plot(time,u2_comp(1,:),'Color',color(1,:),'LineStyle','--','linewidth',1.5,'HandleVisibility','off')
plot(time,u2_comp(2,:),'Color',color(2,:),'LineStyle','--','linewidth',1.5,'HandleVisibility','off')
plot(time,u2_comp(3,:),'Color',color(3,:),'LineStyle','--','linewidth',1.5,'HandleVisibility','off')

plot(time(index_of_first(1):index_of_first(2)),u2_comp(1,index_of_first(1):index_of_first(2)),...
    'Color',color(1,:),'LineStyle','-','linewidth',2)
plot(time(index_of_first(2):index_of_first(3)),u2_comp(2,index_of_first(2):index_of_first(3)),...
    'Color',color(2,:),'LineStyle','-','linewidth',2)
plot(time(index_of_first(3):end),u2_comp(3,index_of_first(3):end),...
    'Color',color(3,:),'LineStyle','-','linewidth',2)

legend ('Interval 1','Interval 2','Null input','best')
xline(time(index_of_first),'-',{'Interval 1','Interval 2', 'Null input'},...
    'LabelOrientation','aligned','HandleVisibility','off')

% Plot trajectory
figure,plot(x_opt,y_opt,'k --','linewidth',1),...
    axis square, axis equal, grid on, xlim('padded'),ylim('padded'), xlabel('X [m]'), ylabel('Y [m]'), title('Optimized trajectory')
hold on
for i = 1:length(x_opt)
    plot_unicycle(x_opt(i),y_opt(i),theta_opt(i),wheelBase,wheelWidth,wheelDiam,bodyLength,bodyWidth)
end
% Plot threshold
n=0:0.01:2*pi; 
plot(x1 + threshold(2)*cos(n),y1 + threshold(2)*sin(n),'r -','linewidth',2) 
hold off

% Plot confronto traiettorie per ogni intervallo
figure,plot(x_comp(1,:),y_comp(1,:), '-- o','linewidth',1),...
    axis square, axis equal, grid on, xlim('padded'),ylim('padded'),xlabel('X [m]'), ylabel('Y [m]'), title('Optimized trajectory vs. Standard trajectory')
hold on
plot(x_opt,y_opt, 'k -- o','linewidth',1)
hold on
legend ('Standard trajectory','Optimized trajectory','Location', 'Best')

hold off