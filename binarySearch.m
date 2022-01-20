%SEARCH MIN TIME
%--------------------------------------------------------------------------
function indice=binarySearch(Tempo, inizio,fine)

global vmax d intervallo % parametri del modello
global K1 K2 K3 P1 P2 % pesi del funzionale
global N Tc % parametri della simulazione
global x0 y0 theta0 theta0Grad % condizioni iniziali
global x y theta u

%     if (inizio>fine)
%         m=0;
%         indice=m;
%     else
%         m=floor((inizio+fine)/2);
%         val1=calculus(m);
%         val2=calculus(m-1);
% 
%         if and(val1==1, val2==0)
%             indice=m;
%         else
%             if (val1==0)
%                 indice=binarySearch(Tempo,m+1,fine);
%             elseif (val2==1)
%                 indice=binarySearch(Tempo,inizio,m-1);
%             end
%         end
%     end

