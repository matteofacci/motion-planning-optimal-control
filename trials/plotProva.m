clear all
close all
clc

x = 0;
y = 0;


L = 0.235; % wheelbase
wheelWidth = 0.02223; %m
wheelDiam = 0.072; %m
radCaster = 0.03; %m
bodyDiam = 0.3485; %m

theta = deg2rad(30); %deg


% phi = deg2rad(-45); %deg
% 
% xR = 0;
% yR = 0;
% xF = xR+L*cos(theta);
% yF = yR+L*sin(theta);

figure(1)
%plot_bicycle(xR,xF,yR,yF,theta,phi,wheelWidth,wheelDiam,wheelRGB)
plot_unicycle(x,y,theta,L,wheelWidth,wheelDiam,bodyDiam,radCaster)
grid on
axis equal


