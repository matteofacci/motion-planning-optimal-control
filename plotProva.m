clear all
close all
clc

L = 1.55;
wheelWidth = 0.1778; %m
wheelDiam = 0.254; %m
wheelRGB = [0,0,0]; %black

theta = deg2rad(30); %deg
phi = deg2rad(-45); %deg

xR = 0;
yR = 0;
xF = xR+L*cos(theta);
yF = yR+L*sin(theta);

figure(1)
plot_bicycle(xR,xF,yR,yF,theta,phi,wheelWidth,wheelDiam,wheelRGB)
grid on
axis equal


