function [x,y,theta,u,x0,y0,theta0,x_opt,y_opt,theta_opt,u1_opt,u2_opt] = update_variables(N,L,x,y,theta,u,x_opt,y_opt,theta_opt,u1_opt,u2_opt)

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