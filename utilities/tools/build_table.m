function T = build_table(switchingInstant,threshold,time,x_opt,y_opt,theta_opt,u1_opt,u2_opt,x1,y1)

if switchingInstant(1)~=0 || switchingInstant(2)~=0
    j=1;
    while time(j)<=switchingInstant(2)
        if (x_opt(j)-x1)^2+(y_opt(j)-y1)^2 >= threshold(2)^2
            Interval(j,1) = 1;
        elseif (x_opt(j)-x1)^2+(y_opt(j)-y1)^2 <= threshold(3)^2
            Interval(j,1) = 3;
        else
            Interval(j,1) = 2;
        end
        Time(j,1) = time(j);
        X(j,1) = x_opt(j);
        Y(j,1) = y_opt(j);
        Theta_Radians(j,1) = theta_opt(j);
        k=floor(rad2deg(theta_opt(j))/360); % floor arrotonda il rapporto all'intero inferiore
        Theta_Degrees(j,1) = rad2deg(theta_opt(j))-k*360; % angolo equivalente in gradi
        V_Linear(j,1) = u1_opt(j);
        V_Angular(j,1) = u2_opt(j);
        j=j+1;
    end
    
    T = table(Time,Interval,X,Y,Theta_Radians,Theta_Degrees,V_Linear,V_Angular);
end

end