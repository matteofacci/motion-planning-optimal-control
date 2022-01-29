clear all
close all
clc

f = figure(1);
x0 = 0;
y0 = 0;
theta0 = deg2rad(0); %deg


wheelBase = 0.235; % wheelbase
wheelWidth = 0.02223; %m
wheelDiam = 0.072; %m
radCaster = 0.03; %m
bodyDiam = 0.3485; %m



plot_unicycle(x0,y0,theta0,wheelBase,wheelWidth,wheelDiam,bodyDiam,radCaster)
xlim([-3 3])
ylim([-3 3])
grid on
axis equal

viaPoint = pickPosition(f);
viaPoint = [viaPoint(:,:);x0,y0];

figure(2)
hold on
grid on
axis equal

for i = 1 : length(viaPoint)-1
    [viaPoint(i,3),R{i}] = computeTheta(viaPoint(i+1,1),viaPoint(i+1,2),viaPoint(i,1),viaPoint(i,2));
end

viaPoint(end,3)=theta0;

q=[x0,y0,theta0;viaPoint(:,:)];

for i = 1:length(q)
    plot_unicycle(q(i,1),q(i,2),q(i,3),wheelBase,wheelWidth,wheelDiam,bodyDiam,radCaster)
end







%
% plot_unicycle(xFinal,yFinal,thetaFinal,wheelBase,wheelWidth,wheelDiam,bodyDiam,radCaster)
%
% rad2deg(thetaFinal)

% qFinal = R*[xFinal;yFinal;1];
% qStart = R*[xStart;yStart;1];
%
% figure(2)
% plot_unicycle(qStart(1),qStart(2),thetaStart,wheelBase,wheelWidth,wheelDiam,bodyDiam,radCaster)
% plot_unicycle(qFinal(1),qFinal(2),thetaFinal,wheelBase,wheelWidth,wheelDiam,bodyDiam,radCaster)
% grid on
% axis equal

