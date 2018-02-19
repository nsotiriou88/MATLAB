%% Nicholas.m

clear all; close all; clc; beep off

%% Initialize

Anchors = 8;
Lin     = 1;

% XY      = 10*rand(Anchors,2);
% no theta2 here, just random anchor points
XY      = 5*rand(Anchors,2);
T       = rand(1,2);
D_Mat   = zeros(Anchors,1);
theta   = linspace(0,2*pi,100)';

%% Determine distances, without error included

for i = 1:Anchors
    D_Mat(i) = sqrt((XY(i,1)-T(1))^2+(XY(i,2)-T(2))^2); % + 1*randn(1);
end


%% Determine distance from target to linearizer
Dr2     = ((T(1,1)-XY(Lin,1))^2+(T(1,2)-XY(Lin,2))^2);

%% Determine Di2, Dir2
Dir2    = zeros(Anchors,1);
Di2     = D_Mat.^2;


for i = 2:Anchors              
        Dir2(i) = ((XY(i,1)-XY(Lin,1))^2+(XY(i,2)-XY(Lin,2))^2);
end

%% Set up solution matrix

Sol_Mat = zeros(Anchors-1,2);
Sol_Vec = zeros(Anchors-1,1);

for i = 2:Anchors              
        
    Rowi = [XY(Lin,1) - XY(i,1),XY(Lin,2) - XY(i,2)];
    Sol_Mat(i-1,:)  = Rowi;
    Sol_Vec(i-1)    = 0.5*(Dr2 + Dir2(i) - Di2(i));    
end

%% Solve system

X = (inv((Sol_Mat')*(Sol_Mat)))*(Sol_Mat')*(Sol_Vec);

% Add the reference point, XY(Lin,:)
Target = X' + XY(Lin,:)
disp(T)

%% Plot Stuff
hold on
plot(XY(:,1),XY(:,2),'r.','MarkerSize',15)
plot(T(:,1),T(:,2),'b.','MarkerSize',15)
plot(Target(:,1),Target(:,2),'g.','MarkerSize',15)
for j = 1:Anchors
   plot(D_Mat(j)*cos(theta)+XY(j,1),D_Mat(j)*sin(theta)+XY(j,2),'r--') 
end
axis equal
box on