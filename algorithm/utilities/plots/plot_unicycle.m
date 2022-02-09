function plot_unicycle(x,y,theta,L,wheelWidth,wheelDiam,bodyLength,bodyWidth)

xR = x + L/2*cos(theta-pi/2);
yR = y + L/2*sin(theta-pi/2);
xL = x - L/2*cos(theta-pi/2);
yL = y - L/2*sin(theta-pi/2);
% xCaster = x + 0.127*cos(theta);
% yCaster = y + 0.127*sin(theta);

wheelRGB = [0,0,0]; %black
bodyRGB = 'r';
draw_rectangle([x,y],bodyWidth,bodyLength,rad2deg(theta),bodyRGB);
draw_rectangle([xR,yR],wheelWidth,wheelDiam,rad2deg(theta),wheelRGB)
hold on
draw_rectangle([xL,yL],wheelWidth,wheelDiam,rad2deg(theta),wheelRGB)
%plot([xR xL], [yR yL],'LineWidth',2,'Color','r')
%scatter([xR xL], [yR yL],50,'filled','MarkerEdgeColor','r','MarkerFaceColor','r')
%plotCircle(x,y,bodyDiam/2,bodyRGB,1);
%scatter(xCaster, yCaster,1000,'filled','MarkerEdgeColor',wheelRGB,'MarkerFaceColor',wheelRGB)
%[xCircle,yCircle] = plotCircle(xCaster,yCaster,radCaster,wheelRGB,1);
%fill(xCircle,yCircle, wheelRGB)



arrowLength = bodyLength; % diminuire o aumentare per regolare la lunghezza delle frecce nel grafico

angle = theta;
rho=arrowLength*ones(1,length(angle));
[a,b] = pol2cart(angle,rho);
quiver(x + bodyLength/2*cos(theta), y + bodyLength/2*sin(theta),a, b,0,'b','linewidth',1.5)

end