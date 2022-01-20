function plot_bicycle(xR,xF,yR,yF,theta,phi,wheelWidth,wheelDiam,wheelRGB)

draw_rectangle([xR,yR],wheelWidth,wheelDiam,rad2deg(theta),wheelRGB)
hold on
draw_rectangle([xF,yF],wheelWidth,wheelDiam,rad2deg(theta+phi),wheelRGB)
plot([xR xF], [yR yF],'LineWidth',5,'Color','r')
scatter([xR xF], [yR yF],100,'filled','MarkerEdgeColor','r','MarkerFaceColor','r')

arrowLength = 0.5; % diminuire o aumentare per regolare la lunghezza delle frecce nel grafico

angle = theta+phi;
rho=arrowLength*ones(1,length(angle));
[a,b] = pol2cart(angle,rho);
quiver(xF,yF,a, b,0,'b','linewidth',3)

end