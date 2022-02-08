function [x,y,theta,u,x0,y0,theta0,x_opt,y_opt,theta_opt,u1_opt,u2_opt,x_comp,y_comp,theta_comp,u1_comp,u2_comp] = update_variables(timeInterval,N,L,x,y,theta,u,x_opt,y_opt,theta_opt,u1_opt,u2_opt,zero_vec,x_comp,y_comp,theta_comp,u1_comp,u2_comp);

% Subarray of the state referred to the interval, with the initial state
% already contained in the previous subarray as the final state
x_temp=x(2:L);
y_temp=y(2:L);
theta_temp=theta(2:L);

% Concatenation from the initial state
x_temp=[x_opt,x_temp];
y_temp=[y_opt,y_temp];
theta_temp=[theta_opt,theta_temp];

% Subarray of the inputs referred to the interval
u1_temp=u(1:L-1);
u2_temp=u(N+1:N+L-1);

% Concatenation of the input from the start
u1_temp=[u1_opt,u1_temp];
u2_temp=[u2_opt,u2_temp];

% Evolution until the end, for comparison
x_comp(timeInterval,:)=[zero_vec,x]; % extracts the time interval row from the matrix
y_comp(timeInterval,:)=[zero_vec,y];
theta_comp(timeInterval,:)=[zero_vec,theta];

% Input extended to the whole interval
u1_comp(timeInterval,:)=[zero_vec,u(1:N)];
u2_comp(timeInterval,:)=[zero_vec,u(N+1:2*N)];

% Update the variables to repeat the loop

x_opt=x_temp;
y_opt=y_temp;
theta_opt=theta_temp;

u1_opt=u1_temp;
u2_opt=u2_temp;

% New starting conditions
x0=x_opt(length(x_opt));
y0=y_opt(length(y_opt));
theta0=theta_opt(length(theta_opt));

end