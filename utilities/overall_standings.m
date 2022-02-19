function [standing_matrix,switchingInstant,time,x_opt,y_opt,theta_opt,u1_opt,u2_opt,tReaction,dThresholdSwitch,tStop] = overall_standings(evaluation_matrix)

[maxEvals,maxElems] = size(evaluation_matrix);

for i = 1:maxEvals
    err_x = evaluation_matrix{i,2}(end) - evaluation_matrix{i,11}(end);
    err_y = evaluation_matrix{i,3}(end) - evaluation_matrix{i,12}(end);
    err_theta = evaluation_matrix{i,4}(end) - evaluation_matrix{i,13}(end);
    err_tot(i) = sqrt(err_x^2+err_y^2+err_theta^2);  
    time_tot(i) = evaluation_matrix{i,7}(2) - evaluation_matrix{i,10}(end);
    index(i) = i;
end

standing_matrix = [index;err_tot;time_tot]';
standing_matrix = sortrows(standing_matrix,2);

for i=1:maxEvals
    standing_matrix(i,4) = i;
end

standing_matrix = sortrows(standing_matrix,3);

for i=1:maxEvals
    standing_matrix(i,5) = i;
    standing_matrix(i,6) = standing_matrix(i,4)+standing_matrix(i,5);
end

standing_matrix = sortrows(standing_matrix,6);

finalIndex = standing_matrix(1,1); 

switchingInstant = evaluation_matrix{finalIndex,7}(:);
time = evaluation_matrix{finalIndex,1}(:);
x_opt = evaluation_matrix{finalIndex,2}(:);
y_opt = evaluation_matrix{finalIndex,3}(:);
theta_opt = evaluation_matrix{finalIndex,4}(:);
u1_opt = evaluation_matrix{finalIndex,5}(:);
u2_opt = evaluation_matrix{finalIndex,6}(:);
tReaction = evaluation_matrix{finalIndex,8}(:);
dThresholdSwitch = evaluation_matrix{finalIndex,9}(:);
tStop = evaluation_matrix{finalIndex,10}(:);


