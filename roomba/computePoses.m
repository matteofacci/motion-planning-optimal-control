function [qPicked,qActual,R,totR,totPoses] = computePoses(x0,y0,theta0,f,maxPoses,returnToBase,wheelBase,wheelWidth,wheelDiam,bodyDiam,radCaster)

viaPoint = pickPosition(f,maxPoses);
viaPoint = [viaPoint(:,:);x0,y0];
R{1} = eye(3);

for i = 1 : length(viaPoint)-1
    [viaPoint(i,3),R{i+1}] = computeTheta(viaPoint(i+1,1),viaPoint(i+1,2),viaPoint(i,1),viaPoint(i,2));
end

viaPoint(end,3)=theta0;



if returnToBase == 0
    viaPoint = viaPoint(1:end-1,:);
end

qPicked=[x0,y0,theta0;viaPoint(:,:)];

[totPoses,numCols] = size(qPicked);

% figure(2)
% grid on
% axis equal
% for i = 1:numRows
%     plot_unicycle(qPicked(i,1),qPicked(i,2),qPicked(i,3),wheelBase,wheelWidth,wheelDiam,bodyDiam,radCaster)
% end

[numElems,totR] = size(R);

for k = 1:totR
for i = 1:totPoses
    qTemp(i,:) = R{k}*[qPicked(i,1:2)';1];
    qTemp(i,3) = qPicked(i,3);
    qActual{k} = qTemp;
    
end
end

if returnToBase ~= 0
R{end+1} = [1,0,qActual{1}(end,1);0,1,qActual{1}(end,2);0,0,1];  % translation matrix

[numElems,totR] = size(R);

for k = 1:totR
for i = 1:totPoses
    qTemp(i,:) = R{k}*[qPicked(i,1:2)';1];
    qTemp(i,3) = qPicked(i,3);
    qActual{k} = qTemp;
    
end
end
end

for k=1:totR

   figure(k+1)
        hold on
        grid on
        axis equal
        for i = 1:totPoses
            plot_unicycle(qActual{k}(i,1),qActual{k}(i,2),qActual{k}(i,3),wheelBase,wheelWidth,wheelDiam,bodyDiam,radCaster)
        end
end

end