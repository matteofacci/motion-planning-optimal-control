global vmax d % model parameters
global K1 K2 K3 % weights of the functional

%% Robot physical parameters

wheelBase = 0.65; % [m]
wheelWidth = 0.1; % [m]
wheelDiam = 0.483; % [m]
bodyLength = wheelBase-wheelWidth; % [m]
bodyWidth = 0.63; % [m]
bodyDiam = 0.3485; % [m]

vmax = 20/3.6; % maximum linear velocity [m/s]
d = wheelBase/2; % wheel drive shaft length [m]

%% Control constraints (limits on linear and angular velocities)

u1_lb = 0; % minimum linear velocity [m/s]
%u1_lb = -vmax; % minimum linear velocity [m/s]
%U1inf(1) = 0; % in interval 1 the robot moves only with positive speed of advancement
%U1inf(2) = -vmax; % in the interval 2 to position itself, the possibility of reversing is added
u1_ub = vmax;
u2_lb = -vmax/d; % minimum angular velocity [rad/s]
u2_ub = vmax/d; % maximum angular velocity [rad/s]

%% Threshold

% Calculate the threshold corresponding to the distance that separates the
% device from the goal
distance0=sqrt((x1-x0)^2+(y1-y0)^2);
fprintf('Distance from the goal: %f m\n', distance0);

thresholdValue=6; % default value of the switching threshold 2 [m]

threshold(1)=1000*distance0; % for simplicity, increase the threshold of the I1 interval as desired
% Predetermined thresholds based on the problem
threshold(2)=thresholdValue; % functional switch
threshold(3)=0.5; % value of the switching threshold 3 (for u_null) [m]
threshold(4)=0; % additional threshold

%% Weights

% Weights of the variable part of the functional (terms u1^2 and u2^2)
W1_var(1) = 1;
W1_var(2) = 1000;
W1_var(3) = 0; % not used because it relates to the interval below the activation threshold (interval 3)
W1_var(4) = 0;
W2_var(1) = 1000;
W2_var(2) = 1;
W2_var(3) = 0; % same as varW1(3)
W2_var(4) = 0;

% Fixed weights of the functional
K1 = 10000; % weight of term x
K2 = 10000; % weight of term y
K3 = 10000; % weight of term theta