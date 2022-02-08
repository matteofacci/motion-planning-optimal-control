%PROGRAMMA PRINCIPALE PER DETERMINARE IL CONTROLLO OTTIMO IN UN MODELLO
%UNICICLO
%--------------------------------------------------------------------------
clear all
close all
clc

global vmax d intervallo % parametri del modello
global K1 K2 K3 P1 P2 % pesi del funzionale
global N Tc % parametri della simulazione
global x0 y0 theta0 theta0Grad x1 y1 theta1 theta1Grad % condizioni iniziali
global x y theta u

robotParams;
maxPoses = 2;

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

x0 = position(1,1);
y0 = position(1,2);
x1 = position(2,1);
y1 = position(2,2);

% x1 = 20;
% y1 = -10;
% theta1 = 90;
% 
% theta1 = deg2rad(theta1);
% 
% % Condizioni iniziali
% % input ascissa e ordinata dispositivo (in metri)
% x0 = input('Ascissa iniziale in metri (default 0 m): '); 
% if isempty(x0)
%     x0 = 0;
% end
% 
% y0 = input('Ordinata iniziale in metri (default 0 m): ');
% if isempty(y0)
%     y0 = 0;
% end

% input orientazione iniziale dispositivo (in gradi)
theta0Grad = input('Orientazione iniziale in gradi (default 0°): ');
if isempty(theta0Grad)
    theta0Grad = 0;
end
theta0 = deg2rad(theta0Grad); % conversione in radianti

% input orientazione finale dispositivo (in gradi)
theta1Grad = input('Orientazione finale in gradi (default 0°): ');
if isempty(theta1Grad)
    theta1Grad = 0;
end
theta1 = deg2rad(theta1Grad); % conversione in radianti

%fprintf('\nPosizione iniziale Segway in metri: (%f,%f)\n',x0,y0);
%fprintf('Orientazione iniziale in gradi: %f\n',theta0Grad);
fprintf('Orientazione iniziale in radianti: %f\n\n',theta0);

% Vincoli sul controllo (limiti sulle velocita' linerare e angolare)
U1inf = 0;
%U1inf = -vmax;
%U1inf(1) = 0; % nell'intervallo 1 si muove solo con velocita' di avanzamento positiva
%U1inf(2) = -vmax; % nell'intervallo 2 per posizionarsi si aggiunge la possibilità di fare retromarcia
U1sup = vmax;
U2inf = -vmax/d;
U2sup = vmax/d;

%fprintf('Velocità lineare in m/s (min,max): (%f,%f)\n',U1inf,U1sup);
%fprintf('Velocità angolare in m/s (min,max): (%f,%f)\n\n',U2inf,U2sup);

% Soglia
% Calcolo la soglia base corrispondente alla distanza iniziale che separa il
% dispositivo dall'obiettivo
distanza0=sqrt((x1-x0)^2+(y1-y0)^2);
fprintf('Distanza dall''obiettivo: %f m\n', distanza0);
valoreSoglia=4; % valore in metri di default della soglia di commutazione 2
soglia(1)=1000*distanza0; % per semplicità aumento a piacere la soglia dell'intervallo I1
% soglie prestabilite in base al problema
soglia(2)=valoreSoglia*1.5; % switch del funzionale
soglia(3)=0.5; % valore in metri della soglia di commutazione 3 (per u nullo)
soglia(4)=0; % soglia aggiuntiva

% Zona di appartenenza iniziale (dominio normale rispetto asse x)
% se il dispositivo è nell'intervallo 2 (tra le due circonferenze - vicino all'obiettivo)
if (x1-x0)^2+(y1-y0)^2<soglia(2)^2 && (x1-x0)^2+(y1-y0)^2>=soglia(3)^2
    intervallo=2;
% se è nell'intervallo3 (posizione obiettivo)
elseif (x1-x0)^2+(y1-y0)^2<soglia(3)^2
    intervallo=3;
else 
    intervallo=1;
end

if intervallo==1
    fprintf('Il dispositivo si trova inizialmente in I1.\n\n');
elseif intervallo==2
    fprintf('Il dispositivo si trova inizialmente in I2.\n\n');
else
    fprintf('Il dispositivo si trova già nella posizione obiettivo.\n');
    fprintf('Ottimizzazione non necessaria.\n\n');
end

% Pesi della parte variabile del funzionale, termini u1^2 e u2^2
PP1(1) = 1;
PP1(2) = 1000;
PP1(3) = 0; % non utilizzato perché relativo all'intervallo sotto soglia di attivazione (intervallo(3))
PP1(4) = 0;
PP2(1) = 1000;
PP2(2) = 1;
PP2(3) = 0; % come PP1(3)
PP2(4) = 0;

% Pesi fissi del funzionale
K1 = 10000; % Peso termine x
K2 = 10000; % Peso termine y
K3 = 10000; % Peso termine theta

% Parametri della simulazione di default
Ttotdef=20; % durata esperimento in secondi

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

% Generazione dell'asse dei tempi
tempo=0:Tc:Ttot; % N+1 elementi

% Input non necessari di fmincon
A=[];
B=[];
Aeq=[];
Beq=[];

% Inizializzazione dei vettori di stato e ingresso
xtot=x0;
ytot=y0;
thetatot=theta0;
u1tot=[];
u2tot=[];

finito=false; % flag

% Preparazione dei vettori x y theta u1 e u2 riferiti all'intero
% intervallo di tempo per confronto
vettzeri=[];
xfull=[];
yfull=[];
thetafull=[];
u1full=[];
u2full=[];

fprintf('\n\n');

% INIZIO CICLO
segmento=0;
istantecomm=[0,0]; % vettore contenente gli istanti di switch definitivi

while N>0
    
    disp('-------------------------------------------------------------')
    segmento=segmento+1;
    t_in=(Ntot-N)*Tc; % Corrisponde a tempo(Ntot-N+1);
    indice_in=Ntot-N+1;
    disp(['Inizio analisi segmento di tempo ',num2str(segmento),' da t_in = ',num2str(t_in),' a t_fin = ',num2str(Ntot*Tc)])
    disp(['N = ',num2str(N)])
    disp(['dispositivo nell''intervallo I',num2str(intervallo)])
    
    % Inizializzazione delle grandezze dipendenti da N
    % Definizione dimensione e vincoli su ingresso per fmincon
    vettore=ones(1,N); % crea vettore di valori unitari
    U1min=U1inf*vettore;
    %if intervallo==1
    %    U1min=U1inf(1)*vettore; 
    %else
    %    U1min=U1inf(2)*vettore;
    %end
    U1max=U1sup*vettore;
    U2min=U2inf*vettore;
    U2max=U2sup*vettore;
    LB=horzcat(U1min,U2min); % vettore di dimensione 2N
    UB=horzcat(U1max,U2max);
    U0=0.2*UB;
    
    % Peso attuale termine discontinuo
    P1=PP1(intervallo);
    P2=PP2(intervallo);

    disp(['Peso P1 variabile pari a ',num2str(P1)])
    disp(['Peso P2 variabile pari a ',num2str(P2)])
    
    % Operazione diversa per I1 e I2 e quello sotto soglia minima I3.
    % Per tutti occorre trovare l'ingresso (fmincon), tranne per I3
    % intervallo con l'ingresso posto a zero
    if intervallo<3 % Calcolo controllo ottimo; ottengo x,y,theta,u global
        % Chiamata fmincon
        [ingresso,costo,EF,uscita,moltiplicatori]...
            = fmincon('funzionale',U0,A,B,Aeq,Beq,LB,UB);
    end
    
    if intervallo == 3 % Calcolo l'evoluzione per ingresso nullo; ottengo x,y,theta,u global
        unullo=zeros(1,2*N);
        costozero=funzionale(unullo);
        finito=true;
    end
    
    % Per il segmento attuale, trovo, se esiste, l'istante di commutazione 
    % corrispondente al raggiungimento di uno dei valori di soglia.
    % N campioni di ingresso, N+1 di stato, con il primo elemento pari allo
    % stato iniziale, quindi inutile da verificare
    L=2;
    % x(1)=x0, inutile riconsiderarlo. Stesso per y e theta
    
    % finchè il dispositivo si trova nell'intervallo ...
    while and((x(L)-x1)^2+(y(L)-y1)^2>=soglia(intervallo+1)^2 ...
            && (x(L)-x1)^2+(y(L)-y1)^2<soglia(intervallo)^2, L<=N) %length(x)=N+1
        disp(['L = ',num2str(L),'; dispositivo ancora nell''intervallo ',num2str(intervallo)])
        L=L+1; % itero fino ad arrivare alla fine dell'intervallo
    end
    
    if L==N+1 % x(N+1)=x(N Tc)
        disp('Nessuna (ulteriore) commutazione effettuata entro il tempo')
        finito=true;
    end
    
    if and((x(L)-x1)^2+(y(L)-y1)^2<soglia(intervallo+1)^2, not(finito))
        if intervallo==1
            istantecomm(intervallo)=(L-1)*Tc+t_in;
            % Calcolo tempo di reazione tra soglia e switch
            dSwitch1=((x(L)-x1)^2+(y(L)-y1)^2)^(1/2); % posizione primo campione dopo lo switch
            dSogliaSwitch=soglia(2)-dSwitch1; % distanza tra il campione e la soglia
            tReazione=dSogliaSwitch/abs(u(L-1)); % tempo di reazione per lo switch
        end
        if intervallo==2
            istantecomm(intervallo)=(L-1)*Tc+t_in;
            % Calcolo tempo totale di arrivo tra soglia e switch
            dSwitch2=((x(L)-x1)^2+(y(L)-y1)^2)^(1/2); % posizione primo campione dopo lo switch
            dSogliaSwitch2=soglia(3)-dSwitch2; % distanza tra il campione e la soglia finale
            tStop=dSogliaSwitch2/abs(u(L-1)); % tempo di reazione per ingresso nullo
        end
        intervallo=intervallo+1;
        disp(['Commutazione all''istante ',num2str((L-1)*Tc+t_in),'; passaggio all''intervallo (superiore) ',num2str(intervallo)])
    end
    
    if and((x(L)-x1)^2+(y(L)-y1)^2>=soglia(intervallo)^2, not(finito))
        intervallo=intervallo-1;
        disp(['L = ',num2str(L),'; passaggio all''intervallo (inferiore) ',num2str(intervallo)])
    end
    
    t_fin=(Ntot-N+(L-1))*Tc;
    indice_fin=(Ntot-N+(L-1)+1);
    disp(['Segmento considerato da t_in = ',num2str(t_in),' a t_fin = ',num2str(t_fin)])
    
    % Aggiorno i vettori di stato e dell'ingresso
    
    % segmento dello stato riferito all'intervallo, oltre lo stato iniziale
    % già contenuto nel precedente segmento come stato finale
    int_x=x(2:L);
    int_y=y(2:L);
    int_theta=theta(2:L);
    
    % concatenazione dallo stato iniziale
    nuovo_xtot=[xtot,int_x];
    nuovo_ytot=[ytot,int_y];
    nuovo_thetatot=[thetatot,int_theta];
    
    % segmenti degli ingressi riferiti all'intervallo
    int_u1=u(1:L-1);
    int_u2=u(N+1:N+L-1);
    
    % cancatenazione dell'ingresso dall'inizio
    nuovo_u1tot=[u1tot,int_u1];
    nuovo_u2tot=[u2tot,int_u2];
    
    % memorizzazione evoluzione fino alla fine, per confronto successivo.
    xfull(segmento,:)=[vettzeri,x]; % estrae la riga segmento dalla matrice
    yfull(segmento,:)=[vettzeri,y];
    thetafull(segmento,:)=[vettzeri,theta];
        
    % ingresso esteso a tutto l'intervallo
    u1full(segmento,:)=[vettzeri,u(1:N)];
    u2full(segmento,:)=[vettzeri,u(N+1:2*N)];
    
    % aggiorno tutto per ricominciare il ciclo
    % stato
    xtot=nuovo_xtot;
    ytot=nuovo_ytot;
    thetatot=nuovo_thetatot;
    % ingresso
    u1tot=nuovo_u1tot;
    u2tot=nuovo_u2tot;
    
    % condizione iniziale
    x0=xtot(length(xtot));
    y0=ytot(length(ytot));
    theta0=thetatot(length(thetatot));
    
    % campioni rimanenti
    N=N-(L-1);
    % Zeri iniziali per le variabili *full al prossimo passo, se serve (!)
    % Devo finire sempre con vettzeri con Ntot+1 componenti
    vettzeri=[vettzeri,zeros(1,L-1)];
    
end

disp('-------------------------------------------------------------')

% Tabella riportante i risultati della simulazione
if istantecomm(1)~=0 && istantecomm(2)~=0
    j=1;
    while tempo(j)<=istantecomm(2)
        if (xtot(j)-x1)^2+(ytot(j)-y1)^2 >= soglia(2)^2
            Intervallo(j,1) = 1;
        else
            Intervallo(j,1) = 2;
        end
        Istante(j,1) = tempo(j);
        Ascissa(j,1) = xtot(j);
        Ordinata(j,1) = ytot(j);
        Radianti(j,1) = thetatot(j);
        k=floor(rad2deg(thetatot(j))/360); % floor arrotonda il rapporto all'intero inferiore
        Gradi(j,1) = rad2deg(thetatot(j))-k*360; % angolo equivalente in gradi
        v_lineare(j,1) = u1tot(j);
        v_angolare(j,1) = u2tot(j);
        j=j+1;
    end
    
    table(Istante,Intervallo,Ascissa,Ordinata,Radianti,Gradi,v_lineare,v_angolare)
end

disp('-------------------------------------------------------------')
if istantecomm(1)~=0
    fprintf('Istante di commutazione I1-->I2: %f s\n',istantecomm(1));
    fprintf('Istante di superamento della soglia: %f s\n', istantecomm(1)-tReazione);
    fprintf('Tempo di reazione: %f s\n',tReazione);
    fprintf('Distanza percorsa tra soglia e switch: %f m\n', dSogliaSwitch);
end
if istantecomm(2)~=0
    fprintf('Tempo totale impiegato per raggiungere l''obiettivo: %f s\n',istantecomm(2)-tStop);
    fprintf('Durata esperimento: %f s\n',istantecomm(2));
    fprintf('-------------------------------------------------------------\n');

close all
figure(1), plot(tempo,xtot,'k -','linewidth',1), grid on, xlabel('t'), ylabel('x_{1}(t)'), title('x_{1}(t) finale')
figure(2), plot(tempo,ytot,'k -','linewidth',1), grid on, xlabel('t'), ylabel('x_{2}(t)'), title('x_{2}(t) finale')
figure(3), plot(tempo,thetatot,'k -','linewidth',1), grid on, xlabel('t'), ylabel('x_{3}(t)'), title('x_{3}(t) finale')
figure(4), plot(tempo(1:Ntot),u1tot,'k -','linewidth',1), grid on, xlabel('t'), ylabel('u_{1}(t)'), title('u_{1}(t) finale')
figure(5), plot(tempo(1:Ntot),u2tot,'k -','linewidth',1), grid on, xlabel('t'), ylabel('u_{2}(t)'), title('u_{2}(t) finale')

figure(6), plot(tempo,xfull,'linewidth',1), grid on, xlabel('t'), ylabel('x_{1}(t)'), title('componenti x_{1}(t)')
legend ('intervallo 1','intervallo 2','ingresso nullo')
figure(7), plot(tempo,yfull,'linewidth',1), grid on, xlabel('t'), ylabel('x_{2}(t)'), title('componenti x_{2}(t)')
legend ('intervallo 1','intervallo 2','ingresso nullo')
figure(8), plot(tempo,thetafull,'linewidth',1), grid on, xlabel('t'), ylabel('x_{3}(t)'), title('componenti x_{3}(t)')
legend ('intervallo 1','intervallo 2','ingresso nullo')
figure(9), plot(tempo(1:Ntot),u1full,'linewidth',1), grid on,  xlabel('t'), ylabel('u_{1}(t)'), title('componenti u_{1}(t)')
legend ('intervallo 1','intervallo 2','ingresso nullo')
figure(10), plot(tempo(1:Ntot),u2full,'linewidth',1), grid on, xlabel('t'), ylabel('u_{2}(t)'), title('componenti u_{2}(t)')
legend ('intervallo 1','intervallo 2','ingresso nullo')

% Plot della traiettoria
figure(11),plot(xtot,ytot,'k --','linewidth',1),...
hold on
for i = 1 : length(xtot)
    plot_unicycle(xtot(i),ytot(i),thetatot(i),wheelBase,wheelWidth,wheelDiam,bodyLength,bodyWidth)
end
axis equal, grid on, xlabel('x_{1}'), ylabel('x_{2}'), title('traiettoria ottimizzata')

% Plot della soglia
n=0:0.01:2*pi; 
plot(x1 + soglia(2)*cos(n), y1 + soglia(2)*sin(n),'r -','linewidth',2) 
hold on
% % Plot dell'orientazione per ogni campione
% lunghezza = input('Lunghezza frecce orientazione (default 1 m): ');
% if isempty(lunghezza)
%     lunghezza = 1; % diminuire o aumentare per regolare la lunghezza delle frecce nel grafico
% end
% rho=lunghezza*ones(1,length(thetatot));
% [a,b] = pol2cart(thetatot,rho);
% quiver(xtot,ytot,a, b,0,'r','linewidth',1)
% hold on
legend ('traiettoria','soglia','orientazione')
hold off

% Plot confronto traiettorie per ogni intervallo
figure(12),plot(xfull(1,:),yfull(1,:), '-- o','linewidth',1),...
    axis square, axis equal, grid on, xlabel('x_{1}'), ylabel('x_{2}'), title('confronto traiettorie')
hold on
plot(xtot,ytot, 'k -- o','linewidth',1)
hold on
legend ('traiettoria non ottimizzata','traiettoria ottimizzata')

hold off

disp('FINE.')
else
    fprintf('Tempo insufficiente per raggiungere l''obiettivo.\nAumentare durata esperimento.\nFINE.');
end


