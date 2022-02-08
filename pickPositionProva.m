clear all
close all
clc

% Add that folder plus all subfolders to the path.
addpath(genpath('utilities'));


% x0 = 0;
% y0 = 0;
% theta0 = deg2rad(30); %deg

maxPoses = 2;
returnToBase = 0;

wheelBase = 0.65; % wheelbase
wheelWidth = 0.1; % m
wheelDiam = 0.483; % m
bodyLength = wheelBase-2*wheelWidth;
bodyWidth = 0.63;
bodyDiam = 0.3485; % m


f = figure(1);
%plot_unicycle(x0,y0,theta0,wheelBase,wheelWidth,wheelDiam,bodyLength,bodyWidth)
abscissae = [-30,30];
ordinates = [-30,30];
xlim(abscissae)
ylim(ordinates)
grid on
axis equal
hold on
position = pickPosition(f,maxPoses);
hold off
% R = [1,0,-position(2,1);0,1,-position(2,2);0,0,1];  % translation matrix
% 
% [numrows,numcols] = size(position);
% 
% orientation(1) = deg2rad(30);
% orientation(2) = deg2rad(90);
% 
% % figure
% % grid on
% % axis equal
% % for i = 1:numrows
% %     plot_unicycle(position(i,1),position(i,2),position(i,3),wheelBase,wheelWidth,wheelDiam,bodyLength,bodyWidth)
% % end
% 
% position(:,3) = 1;
% 
% for i = 1:numrows
%     position(i,:) = R*position(i,:)';
% end
% 
% for i = 1:numrows
%     pose(i,:) = [position(i,1:2),orientation(i)];
% end
% 
% figure
% grid on
% axis equal
% for i = 1:numrows
%     plot_unicycle(pose(i,1),pose(i,2),pose(i,3),wheelBase,wheelWidth,wheelDiam,bodyLength,bodyWidth)
% end
% 
% % viaPoint = pickPosition(f,maxPoses);
% % viaPoint = [viaPoint(:,:);x0,y0];
% % R{1} = eye(3);
% % 
% % for i = 1 : length(viaPoint)-1
% %     [viaPoint(i,3),R{i+1}] = computeTheta(viaPoint(i+1,1),viaPoint(i+1,2),viaPoint(i,1),viaPoint(i,2));
% % end
% % 
% % viaPoint(end,3)=theta0;
% % 
% % 
% % 
% % if returnToBase == 0
% %     viaPoint = viaPoint(1:end-1,:);
% % end
% % 
% % qPicked=[x0,y0,theta0;viaPoint(:,:)];
% % 
% % [numRows,numCols] = size(qPicked);
% % 
% % % figure(2)
% % % grid on
% % % axis equal
% % % for i = 1:numRows
% % %     plot_unicycle(qPicked(i,1),qPicked(i,2),qPicked(i,3),wheelBase,wheelWidth,wheelDiam,bodyDiam,radCaster)
% % % end
% % 
% % [numElems,numMat] = size(R);
% % 
% % for k = 1:numMat
% % for i = 1:numRows
% %     qTemp(i,:) = R{k}*[qPicked(i,1:2)';1];
% %     qTemp(i,3) = qPicked(i,3);
% %     qActual{k} = qTemp;
% %     
% % end
% % end
% % 
% % if returnToBase ~= 0
% % R{end+1} = [1,0,qActual{1}(end,1);0,1,qActual{1}(end,2);0,0,1];  % translation matrix
% % 
% % [numElems,numMat] = size(R);
% % 
% % for k = 1:numMat
% % for i = 1:numRows
% %     qTemp(i,:) = R{k}*[qPicked(i,1:2)';1];
% %     qTemp(i,3) = qPicked(i,3);
% %     qActual{k} = qTemp;
% %     
% % end
% % end
% % end
% % 
% % for k=1:numMat
% % 
% %    figure(k+1)
% %         hold on
% %         grid on
% %         axis equal
% %         for i = 1:numRows
% %             plot_unicycle(qActual{k}(i,1),qActual{k}(i,2),qActual{k}(i,3),wheelBase,wheelWidth,wheelDiam,bodyDiam,radCaster)
% %         end
% % end
