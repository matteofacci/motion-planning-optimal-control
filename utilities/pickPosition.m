function viaPoint = pickPosition(f,maxPoses,color,label)

i=1;

while true && i <= maxPoses 
    enableDefaultInteractivity(gca);
    [xVP,yVP] = ginput(1);
    viaPoint(i,1) = xVP;
    viaPoint(i,2) = yVP;

    if f.CurrentCharacter > 0
        %viaPoint = viaPoint(1:length(viaPoint)-1,:); 
         viaPoint(end,:) = []; 
        break;
    end

    scatter(viaPoint(i,1),viaPoint(i,2),50,'filled','MarkerEdgeColor',color,'MarkerFaceColor',color);
    labels(i) = label + " " + num2str(i);
    labelpoints(viaPoint(i,1),viaPoint(i,2),labels(i),'buffer',0.5);
    hold on
    i=i+1;

end
end