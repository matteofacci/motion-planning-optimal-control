function Cost = functional(input)

global K1 K2 K3 W1 W2 % weights of the functional
global N St % simulation parameters
global x0 y0 theta0 x1 y1 theta1 % initial conditions and final orientation
global x y theta u % state and control variables


% Initialization
u=input;
x=zeros(1,N+1); % array of zeros
y=zeros(1,N+1);
theta=zeros(1,N+1);

% Initial conditions
x(1)=x0;
y(1)=y0;
theta(1)=theta0;

% Discrete kinematic model
for i=1:N
    x(i+1)=x(i) + St*( u(i)*cos(theta(i)) );
    y(i+1)=y(i) + St*( u(i)*sin(theta(i)) );
    theta(i+1)=theta(i) + St*( u(N+i) );
end

% Extraction of vectors u1 and u2 from matrix u
u1vett=u(1:N);
u2vett=u(N+1:2*N);

% Discrete functional
term1=K1*sum(x.^2); % each component of x is raised to a power and added
term2=K2*sum(y.^2);
term3=K3*sum((theta-theta1).^2); % error between current and final orientation
term4=W1*sum(u1vett.^2);
term5=W2*sum(u2vett.^2);

Cost=St*(term1+term2+term3+term4+term5);
