function viaPoint = pickPosition(f)

i=1;

while true

    [xVP,yVP] = ginput(1);
    viaPoint(i,1) = xVP;
    viaPoint(i,2) = yVP;

    if f.CurrentCharacter > 0
        viaPoint = viaPoint(1:length(viaPoint)-1,:); 
        break;
    end

    scatter(viaPoint(i,1),viaPoint(i,2),50,'filled','MarkerEdgeColor','r','MarkerFaceColor','r');
    labels(i) = "viaPoint " + num2str(i);
    labelpoints(viaPoint(i,1),viaPoint(i,2),labels(i))
    hold on
    i=i+1;

end
end