clear all
close all
clc


% Add that folder plus all subfolders to the path.
addpath(genpath('utilities'));


% x0 = 0;
% y0 = 0;
% theta0 = deg2rad(30); %deg

maxPoses = 2;
returnToBase = 1;

wheelBase = 0.65; % wheelbase
wheelWidth = 0.1; % m
wheelDiam = 0.483; % m
bodyLength = wheelBase-2*wheelWidth;
bodyWidth = 0.63;
bodyDiam = 0.3485; % m
theta0 = 0;

f = figure(1);
abscissae = [0,20];
ordinates = [0,20];
xlim(abscissae)
ylim(ordinates)
grid on
axis equal
hold on


viaPoint = pickPosition(f,maxPoses);

viaPoint(end+1,:) = viaPoint(1,:);
viaPoint(1,3)=theta0;
for i = 2 : length(viaPoint(:,1))-1
    viaPoint(i,3) = computeTheta(viaPoint(i+1,1),viaPoint(i+1,2),viaPoint(i,1),viaPoint(i,2));
end

if returnToBase == 0
    viaPoint(end,:) = [];
end

[numRows,numCols] = size(viaPoint);

grid on
axis equal
for i = 1:numRows
    plot_unicycle(viaPoint(i,1),viaPoint(i,2),viaPoint(i,3),wheelBase,wheelWidth,wheelDiam,bodyLength,bodyWidth)
end
