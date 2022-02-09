function [LB,UB,U0] = input_bounds(N,u1_lb,u1_ub,u2_lb,u2_ub)

% Definition of dimension and constraints on input for fmincon
one_vec=ones(1,N); % creates vector of unit values
u1_min=u1_lb*one_vec;
%if intervallo==1
%    U1min=U1inf(1)*vettore;
%else
%    U1min=U1inf(2)*vettore;
%end
u1_max=u1_ub*one_vec;
u2_min=u2_lb*one_vec;
u2_max=u2_ub*one_vec;

LB=horzcat(u1_min,u2_min); % vector of size 2N
UB=horzcat(u1_max,u2_max);
U0=0.2*UB; % initial input not null but low enough

end