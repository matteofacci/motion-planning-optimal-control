%FMINCON APPOGGIO
%--------------------------------------------------------------------------
function [finito,intervallo]=calculus(Ttot)

global vmax d % parametri del modello
global K1 K2 K3 P1 P2 % pesi del funzionale
global N Tc % parametri della simulazione
global x0 y0 theta0 theta0Grad % condizioni iniziali
global x y theta u% Generazione dell'asse dei tempi

engaged=false;

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
distanza0=(x0^2+y0^2)^(1/2);
fprintf('Distanza dall''obiettivo: %f m\n', distanza0);
valoreSoglia=4; % valore in metri di default della soglia di commutazione 2
soglia(1)=1000*distanza0; % per semplicità aumento a piacere la soglia dell'intervallo I1
% soglie prestabilite in base al problema
soglia(2)=valoreSoglia*1.5; % switch del funzionale
soglia(3)=0.5; % valore in metri della soglia di commutazione 3 (per u nullo)
soglia(4)=0; % soglia aggiuntiva

% Zona di appartenenza iniziale (dominio normale rispetto asse x)
% se il dispositivo è nell'intervallo 2 (tra le due circonferenze - vicino all'obiettivo)
if x0^2+y0^2<soglia(2)^2 && x0^2+y0^2>=soglia(3)^2
    intervallo=2;
% se è nell'intervallo3 (posizione obiettivo)
elseif x0^2+y0^2<soglia(3)^2
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

Ntot = Ttot/Tc;
N=Ntot;

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
            = fmincon('c_functional',U0,A,B,Aeq,Beq,LB,UB);
    end
    
    if intervallo == 3 % Calcolo l'evoluzione per ingresso nullo; ottengo x,y,theta,u global
        unullo=zeros(1,2*N);
        costozero=c_functional(unullo);
        finito=true;
    end
    
    % Per il segmento attuale, trovo, se esiste, l'istante di commutazione 
    % corrispondente al raggiungimento di uno dei valori di soglia.
    % N campioni di ingresso, N+1 di stato, con il primo elemento pari allo
    % stato iniziale, quindi inutile da verificare
    L=2;
    % x(1)=x0, inutile riconsiderarlo. Stesso per y e theta
    
    % finchè il dispositivo si trova nell'intervallo ...
    while and(x(L)^2+y(L)^2>=soglia(intervallo+1)^2 ...
            && x(L)^2+y(L)^2<soglia(intervallo)^2, L<=N) %length(x)=N+1
        disp(['L = ',num2str(L),'; dispositivo ancora nell''intervallo ',num2str(intervallo)])
        L=L+1; % itero fino ad arrivare alla fine dell'intervallo
    end
    
    if L==N+1 % x(N+1)=x(N Tc)
        disp('Nessuna (ulteriore) commutazione effettuata entro il tempo')
        finito=true;
    end
    
    if and(x(L)^2+y(L)^2<soglia(intervallo+1)^2, not(finito))
        if intervallo==1
            istantecomm(intervallo)=(L-1)*Tc+t_in;
            % Calcolo tempo di reazione tra soglia e switch
            dSwitch1=(x(L)^2+y(L)^2)^(1/2); % posizione primo campione dopo lo switch
            dSogliaSwitch=soglia(2)-dSwitch1; % distanza tra il campione e la soglia
            tReazione=dSogliaSwitch/abs(u(L-1)); % tempo di reazione per lo switch
        end
        if intervallo==2
            istantecomm(intervallo)=(L-1)*Tc+t_in;
            % Calcolo tempo totale di arrivo tra soglia e switch
            dSwitch2=(x(L)^2+y(L)^2)^(1/2); % posizione primo campione dopo lo switch
            dSogliaSwitch2=soglia(3)-dSwitch2; % distanza tra il campione e la soglia finale
            tStop=dSogliaSwitch2/abs(u(L-1)); % tempo di reazione per ingresso nullo
        end
        intervallo=intervallo+1;
        disp(['Commutazione all''istante ',num2str((L-1)*Tc+t_in),'; passaggio all''intervallo (superiore) ',num2str(intervallo)])
    end
    if and(x(L)^2+y(L)^2>=soglia(intervallo)^2, not(finito))
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
