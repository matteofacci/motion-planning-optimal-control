function Ttot=search(tmin)

global vmax d intervallo % parametri del modello
global K1 K2 K3 P1 P2 % pesi del funzionale
global N Tc % parametri della simulazione
global x0 y0 theta0 theta0Grad % condizioni iniziali
global x y theta u

i=0;
finito=false;

while not(intervallo==3)
    Ttot=tmin+(i*Tc);
    [finito,intervallo]=calculus(Ttot);
    i=i+1;
end

Ttot=Ttot+1

    