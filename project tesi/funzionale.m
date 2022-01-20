% PROGRAMMA CHE IMPLEMENTA IL SISTEMA CINEMATICO UNICICLO CON IL CONTROLLO; 
% MINIMIZZA L'INDICE DI COSTO "Costo"
%-------------------------------------------------------------------------
function Costo=funzionale(ingresso)

% parametri
global K1 K2 K3 P1 P2 % pesi del funzionale
global N Tc % parametri della simulazione
global x0 y0 theta0 % condizioni iniziali
global x y theta u u1vett u2vett
global termine1 termine2 termine3 termine4 termine5

% Inizializzazione
u=ingresso;
x=zeros(1,N+1); % vettore di zeri
y=zeros(1,N+1);
theta=zeros(1,N+1);

% Condizioni iniziali
x(1)=x0;
y(1)=y0;
theta(1)=theta0;

% modello base: ad ogni componente va sommata la componente precedente
for i=1:N
    x(i+1)=x(i) + Tc*( u(i)*cos(theta(i)) );
    y(i+1)=y(i) + Tc*( u(i)*sin(theta(i)) );
    theta(i+1)=theta(i) + Tc*( u(N+i) );
end

% Estrazione vettori u1 e u2 da matrice u
u1vett=u(1:N);
u2vett=u(N+1:2*N);

% Funzionale discretizzato
termine1=K1*sum(x.^2); % ogni componente di x viene elevata a potenza e sommata
termine2=K2*sum(y.^2);
thetaobiettivoGrad = 0; % cambiare valore (gradi) per variare orientazione finale
thetaobiettivo = deg2rad(thetaobiettivoGrad); % conversione in radianti
termine3=K3*sum((theta-(thetaobiettivo)).^2); 
termine4=P1*sum(u1vett.^2);
termine5=P2*sum(u2vett.^2);

Costo=Tc*(termine1+termine2+termine3+termine4+termine5);
