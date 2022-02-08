%PROGRAMMA PRINCIPALE PER DETERMINARE IL CONTROLLO OTTIMO IN UN MODELLO
%UNICICLO
%--------------------------------------------------------------------------
clear all
close all
clc

nFigure = 1;

global vmax d intervallo % parametri del modello
global K1 K2 K3 P1 P2 % pesi del funzionale
global N Tc % parametri della simulazione
global xStart yStart thetaStart theta0GradStart thetaobiettivoGradStart % condizioni iniziali
global x y theta u qActual

% Add that folder plus all subfolders to the path.
addpath(genpath('utilities'));

robotParams;

% Condizioni iniziali
% input ascissa e ordinata dispositivo (in metri)
xStart = 0;
yStart = 0;
thetaStart = deg2rad(0); %deg

maxPoses = 1;
returnToBase = 0;

%fprintf('\nPosizione iniziale Segway in metri: (%f,%f)\n',x0,y0);
%fprintf('Orientazione iniziale in gradi: %f\n',theta0Grad);
fprintf('Orientazione iniziale in radianti: %f\n\n',thetaStart);

f = figure(nFigure);
plot_unicycle(xStart,yStart,thetaStart,wheelBase,wheelWidth,wheelDiam,bodyDiam,radCaster)
xlim([-3 3])
ylim([-3 3])
grid on
axis equal
nFigure = nFigure+1;

[qPicked,qActual,R,totR,totPoses] = computePoses(xStart,yStart,thetaStart,f,maxPoses,returnToBase,wheelBase,wheelWidth,wheelDiam,bodyDiam,radCaster);

for k=1:totR

    figure(nFigure)
    hold on
    grid on
    axis equal
    for i = 1:totPoses
        plot_unicycle(qActual{k}(i,1),qActual{k}(i,2),qActual{k}(i,3),wheelBase,wheelWidth,wheelDiam,bodyDiam,radCaster)
    end
    nFigure = nFigure+1;
end


% Soglia
% Calcolo la soglia base corrispondente alla distanza iniziale che separa il
% dispositivo dall'obiettivo
distanza0=(qActual{end}(1)^2+qActual{end}(2)^2)^(1/2);
fprintf('Distanza dall''obiettivo: %f m\n', distanza0);
valoreSoglia=1; % valore in metri di default della soglia di commutazione 2
soglia(1)=1000*distanza0; % per semplicità aumento a piacere la soglia dell'intervallo I1
% soglie prestabilite in base al problema
soglia(2)=valoreSoglia*1.5; % switch del funzionale
soglia(3)=0.05; % valore in metri della soglia di commutazione 3 (per u nullo)
soglia(4)=0; % soglia aggiuntiva

% Parametri della simulazione di default
Ttotdef=30; % durata esperimento in secondi

% Parametri della simulazione cambiati dall'utente
Ttot=input('Durata esperimento in secondi (default 20 s): ');
if isempty(Ttot)
    Ttot = Ttotdef;
end

Ntotdef = Ttot*2; % Almeno due campioni al secondo
fprintf('Numero totale Ntot di istanti da campionare');
Ntot=input([' (Massimo ',num2str(Ntotdef),' campioni per un esperimento di ',num2str(Ttot),' s): ']);
if (isempty(Ntot) || Ntot>Ntotdef) % Per la regolarita' della traiettoria Tc >= 0.5 s
    Ntot = Ntotdef;
end

N=Ntot;
Tc=Ttot/Ntot; % tempo di campionamento
% attempts = 0;
% maxPossibleSolutions = 30;

% Tc = 0.5;
% Ttot = ceil(3 * distanza0/vmax);
% Ntot = Ttot/Tc;
% N = Ntot;

[xtot,ytot,thetatot,u1tot,u2tot,xfull,yfull,thetafull,u1full,u2full,tempo,istantecomm] = optimalControl(qActual{end}(1,1),qActual{end}(1,2),qActual{end}(1,3),Tc,Ttot,N,Ntot,vmax,d,soglia);

if istantecomm(2)~=0

    %close all
    figure(nFigure), plot(tempo,xtot,'k -','linewidth',1), grid on, xlabel('t'), ylabel('x_{1}(t)'), title('x_{1}(t) finale')
    nFigure = nFigure+1;
    figure(nFigure), plot(tempo,ytot,'k -','linewidth',1), grid on, xlabel('t'), ylabel('x_{2}(t)'), title('x_{2}(t) finale')
    nFigure = nFigure+1;
    figure(nFigure), plot(tempo,thetatot,'k -','linewidth',1), grid on, xlabel('t'), ylabel('x_{3}(t)'), title('x_{3}(t) finale')
    nFigure = nFigure+1;
    figure(nFigure), plot(tempo(1:Ntot),u1tot,'k -','linewidth',1), grid on, xlabel('t'), ylabel('u_{1}(t)'), title('u_{1}(t) finale')
    nFigure = nFigure+1;
    figure(nFigure), plot(tempo(1:Ntot),u2tot,'k -','linewidth',1), grid on, xlabel('t'), ylabel('u_{2}(t)'), title('u_{2}(t) finale')

    figure(nFigure), plot(tempo,xfull,'linewidth',1), grid on, xlabel('t'), ylabel('x_{1}(t)'), title('componenti x_{1}(t)')
    legend ('intervallo 1','intervallo 2','ingresso nullo')
    nFigure = nFigure+1;
    figure(nFigure), plot(tempo,yfull,'linewidth',1), grid on, xlabel('t'), ylabel('x_{2}(t)'), title('componenti x_{2}(t)')
    legend ('intervallo 1','intervallo 2','ingresso nullo')
    nFigure = nFigure+1;
    figure(nFigure), plot(tempo,thetafull,'linewidth',1), grid on, xlabel('t'), ylabel('x_{3}(t)'), title('componenti x_{3}(t)')
    legend ('intervallo 1','intervallo 2','ingresso nullo')
    nFigure = nFigure+1;
    figure(nFigure), plot(tempo(1:Ntot),u1full,'linewidth',1), grid on,  xlabel('t'), ylabel('u_{1}(t)'), title('componenti u_{1}(t)')
    legend ('intervallo 1','intervallo 2','ingresso nullo')
    nFigure = nFigure+1;
    figure(nFigure), plot(tempo(1:Ntot),u2full,'linewidth',1), grid on, xlabel('t'), ylabel('u_{2}(t)'), title('componenti u_{2}(t)')
    legend ('intervallo 1','intervallo 2','ingresso nullo')
    nFigure = nFigure+1;

    % Plot della traiettoria
    figure(nFigure),plot(xtot,ytot,'k --','linewidth',1),...
        axis square, axis equal, grid on, xlabel('x_{1}'), ylabel('x_{2}'), title('traiettoria ottimizzata')
    hold on
    % Plot della soglia
    n=0:0.01:2*pi;
    plot(soglia(2)*cos(n), soglia(2)*sin(n),'r -','linewidth',2)
    hold on

    for i = 1:length(xtot)
        plot_unicycle(xtot(i),ytot(i),thetatot(i),wheelBase,wheelWidth,wheelDiam,bodyDiam,radCaster)
    end
    hold on
    legend ('traiettoria','soglia')
    hold off
    nFigure = nFigure+1;

    % Plot confronto traiettorie per ogni intervallo
    figure(nFigure),plot(xfull(1,:),yfull(1,:), '-- o','linewidth',1),...
        axis square, axis equal, grid on, xlabel('x_{1}'), ylabel('x_{2}'), title('confronto traiettorie')
    hold on
    plot(xtot,ytot, 'k -- o','linewidth',1)
    hold on
    legend ('traiettoria non ottimizzata','traiettoria ottimizzata')

    hold off
    nFigure = nFigure+1;

    disp('FINE.')
else
    fprintf('Tempo insufficiente per raggiungere l''obiettivo.\nAumentare durata esperimento.\nFINE.');
end


