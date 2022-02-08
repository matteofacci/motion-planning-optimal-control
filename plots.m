

figure(1), plot(time,x_opt,'k -','linewidth',1), grid on, xlabel('t'), ylabel('x_{1}(t)'), title('x_{1}(t) finale')
figure(2), plot(time,y_opt,'k -','linewidth',1), grid on, xlabel('t'), ylabel('x_{2}(t)'), title('x_{2}(t) finale')
figure(3), plot(time,theta_opt,'k -','linewidth',1), grid on, xlabel('t'), ylabel('x_{3}(t)'), title('x_{3}(t) finale')
figure(4), plot(time(1:Ntot),u1_opt,'k -','linewidth',1), grid on, xlabel('t'), ylabel('u_{1}(t)'), title('u_{1}(t) finale')
figure(5), plot(time(1:Ntot),u2_opt,'k -','linewidth',1), grid on, xlabel('t'), ylabel('u_{2}(t)'), title('u_{2}(t) finale')

figure(6), plot(time,x_comp,'linewidth',1), grid on, xlabel('t'), ylabel('x_{1}(t)'), title('componenti x_{1}(t)')
legend ('intervallo 1','intervallo 2','ingresso nullo')
figure(7), plot(time,y_comp,'linewidth',1), grid on, xlabel('t'), ylabel('x_{2}(t)'), title('componenti x_{2}(t)')
legend ('intervallo 1','intervallo 2','ingresso nullo')
figure(8), plot(time,theta_comp,'linewidth',1), grid on, xlabel('t'), ylabel('x_{3}(t)'), title('componenti x_{3}(t)')
legend ('intervallo 1','intervallo 2','ingresso nullo')
figure(9), plot(time(1:Ntot),u1_comp,'linewidth',1), grid on,  xlabel('t'), ylabel('u_{1}(t)'), title('componenti u_{1}(t)')
legend ('intervallo 1','intervallo 2','ingresso nullo')
figure(10), plot(time(1:Ntot),u2_comp,'linewidth',1), grid on, xlabel('t'), ylabel('u_{2}(t)'), title('componenti u_{2}(t)')
legend ('intervallo 1','intervallo 2','ingresso nullo')

% Plot della traiettoria
figure(11),plot(x_opt,y_opt,'k --','linewidth',1),...
    axis square, axis equal, grid on, xlabel('x_{1}'), ylabel('x_{2}'), title('traiettoria ottimizzata')
hold on
for i = 1:length(x_opt)
    plot_unicycle(x_opt(i),y_opt(i),theta_opt(i),wheelBase,wheelWidth,wheelDiam,bodyLength,bodyWidth)
end
% Plot della threshold
n=0:0.01:2*pi; 
plot(threshold(2)*cos(n), threshold(2)*sin(n),'r -','linewidth',2) 
% hold on
% % Plot dell'orientazione per ogni campione
% lunghezza = input('Lunghezza frecce orientazione (default 1 m): ');
% if isempty(lunghezza)
%     lunghezza = 1; % diminuire o aumentare per regolare la lunghezza delle frecce nel grafico
% end
% rho=lunghezza*ones(1,length(theta_opt));
% [a,b] = pol2cart(theta_opt,rho);
% quiver(x_opt,y_opt,a, b,0,'r','linewidth',1)
% hold on
% legend ('traiettoria','threshold','orientazione')
hold off

% Plot confronto traiettorie per ogni intervallo
figure(12),plot(x_comp(1,:),y_comp(1,:), '-- o','linewidth',1),...
    axis square, axis equal, grid on, xlabel('x_{1}'), ylabel('x_{2}'), title('confronto traiettorie')
hold on
plot(x_opt,y_opt, 'k -- o','linewidth',1)
hold on
legend ('traiettoria non ottimizzata','traiettoria ottimizzata')

hold off