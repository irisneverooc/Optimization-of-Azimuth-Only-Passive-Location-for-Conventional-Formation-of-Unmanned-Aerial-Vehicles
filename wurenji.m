clear;clc;close all
% M is the standard template square
M=[0,0
80,80
-80,80
-80,-80
80,-80];

% C is the initial position of the square
C=[0,0
81,80
-80,79
-79.89,-80.15
80.17,-79.66
];

% Number of drones
[n,~] = size(C);

% Position table, P(i,:) is the Cartesian coordinate of i
P = M;
% Fix two drones without adjustment
n1 = 2;
n2 = 3;
% Plot the standard template M
figure
plot(M(:,1),M(:,2),'bo');
title("The Standard Template")
hold on
% Initial positions of drones
plot(P(:,1),P(:,2),'r.');
axis equal
axis([-100,100,-100,100])
grid on

% K is the number of adjustment rounds
K = 15;
DR = zeros(1,K);
k = 1;
while k<=K
disp(k)
% Calculate the standard angles A(i,j) for P(n1,:),P(i,:),P(j,:) based on current position P
A = IntersectAngle(P,n1);
% Calculate the standard angles B(i,j) for P(n2,:),P(i,:),P(j,:) based on current position P
B = IntersectAngle(P,n2);

% The following loop simulates drone i using the angle information from the transmitter
% A, B to determine position adjustment dP(i,:)
dP = zeros(n,2);
% For drones other than n1 and n2, calculate adjustment information
for i = [1:n1-1,n1+1:n2-1,n2+1:n]
% Based on angle information A, B received by drone i in round k, reverse the formation coordinates
[Q,fval,exitflag,output] = FindCoordinate(A(i,:),B(i,:),i,M,n1,n2);
% Use the reverse coordinates Q(i,:) to compare with the standard template M(i,:) to determine position adjustment dP(i,:)
dP(i,:) = M(i,:)-Q(i,:);
end
% Update current coordinates with position adjustment for round k
P = P+dP;
% Plot the current position of drones
plot(P(:,1),P(:,2),'k.');
title("The Current Position of Drones")
axis equal
axis([-100,100,-100,100])
% Calculate and record the standard deviation of drones for round k
[T,R] = cart2pol(P(:,1),P(:,2));
T = T/pi*180;
% Transform angles to 0~360
T(T<0) = T(T<0)+360;
T(T>360) = T(T>360)-360;
% Calculate and record the standard deviation of the radius R of drones for round k
DR(k) = std(R(2:n));
k = k+1;
end

% Plot final positions
plot(P(:,1),P(:,2),'mx')
% Plot standard deviation graph
figure;
plot(1:K, DR);
xlabel('Iteration');
ylabel('Standard Deviation of difference');
title('Standard Deviation of difference vs Iteration');
grid on;

function [Q,fval,exitflag,output] = FindCoordinate(A,B,i,M,n1,n2)
% Reverse the coordinates of drone i based on angle information A, B received
% M is the standard template, n1 and n2 are fixed
% M first column R/m, second column theta/°
[n,~] = size(M);

% Generate optimization
prob = optimproblem('ObjectiveSense','minimize');
% Generate optimization variable Q (n*2 matrix)
% Column labels for drones, row labels for Cartesian coordinates
Q = optimvar('Q',n,2);

AQ(1,n) = Q(1,1)*0;
% Calculate the standard angles AQ(i,j) for the current position table Q
% When j=i, the angle is 0
for j = [1:n1-1,n1+1:n]
if j~=i
a = Q(n1,:)-Q(i,:);
b = Q(j,:)-Q(i,:);
AQ(1,j) = acos(dot(a,b)/norm(a)/norm(b)*0.99);
end
end

BQ(1,n) = Q(1,1)*0;
% Calculate the standard angles BQ(i,j) for the current position table Q
% When j=i, the angle is 0
for j = [1:n2-1,n2+1:n]
if j~=i
a = Q(n2,:)-Q(i,:);
b = Q(j,:)-Q(i,:);
BQ(1,j) = acos(dot(a,b)/norm(a)/norm(b)*0.99);
end
end

% Objective function
prob.Objective = sum((AQ-A).^2)+sum((BQ-B).^2);
% Constraints
prob.Constraints.cons1 = Q(n1,1)==M(n1,1);
prob.Constraints.cons2 = Q(n1,2)==M(n1,2);
prob.Constraints.cons3 = Q(n2,1)==M(n2,1);
prob.Constraints.cons4 = Q(n2,2)==M(n2,2);

% Solve the optimization problem
% Use the formation standard template M as the initial solution
Q0.Q = M;
[q,fval,exitflag,output] = solve(prob,Q0);
Q = q.Q;
end

function A = IntersectAngle(P,k)

[n,~] = size(P);
A = zeros(n);
for i = [1:k-1,k+1:n]
for j = [1:k-1,k+1:n]
if j ~= i
a = P(k,:)-P(i,:);
b = P(j,:)-P(i,:);
A(i,j) = acos(dot(a,b)/norm(a)/norm(b));
end
end
end
end