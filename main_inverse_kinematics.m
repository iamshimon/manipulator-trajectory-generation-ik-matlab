clc
clear all
close all

%%% This example shows how to do the inverse kinematics for a 4 link
%%% manipulator

% DH parameters
global d1 a1 alpha1 
global d2 a2 alpha2
global d3 a3 alpha3
global d4 a4 alpha4 
global x_des y_des z_des %where you want the end-effector


%D-H for links. Theta's are not set, because we need to find them. 
%Link lengths are assumed as below values for a
d1=0; a1=0.3;  alpha1=0; % theta1
d2=0; a2=0.3; alpha2=0; % theta2
d3=0; a3=0.25; alpha3=0; %theta3
d4=0; a4=0.2; alpha4=0; %theta4



%define the circle using theta as parametric
r=0.15; %radius of the circle
x_center= 0.25
y_center= 0.25
theta = linspace(0,2*pi,50);
n = length(theta);
x_des_all= x_center+r*cos(theta);
y_des_all= y_center+r*sin(theta);
z_des_all= zeros(1,n);


% initial guess values for theta
theta1=pi/2; theta2=0; theta3=pi/4; theta4=0;

X0 = [theta1, theta2, theta3, theta4];


%% Solve for the values of theta that give the required end-effector pose.

theta1_all=zeros(1,n);
theta2_all=zeros(1,n);
theta3_all=zeros(1,n);
theta4_all=zeros(1,n);

%fsolve solves for the roots for the equation X-XDES
for i=1:n
    x_des=x_des_all(1,i); %assign destination values for each point on the circle
    y_des=y_des_all(1,i);
    z_des=z_des_all(1,i);
    [X,FVAL,EXITFLAG] = fsolve('find_joint_angles',X0);
    theta1_all(1,i) = X(1); %store solved angle values at each point on the circle
    theta2_all(1,i) = X(2);
    theta3_all(1,i) = X(3);
    theta4_all(1,i) = X(4);
    
    %assign new values as guesses for next iteration
    X0 = X;
    disp(['Exitflag after running fsolve = ', num2str(EXITFLAG) ]) %Tells if fsolve converged or not
               %1 means converged else not converged             
              
    X;
    FVAL;
%Visualise the manipulator with the generated theta values
end

for i=1:n
    X_des = [x_des_all(1,i) y_des_all(1,i),z_des_all(1,i)];
    X = [theta1_all(1,i), theta2_all(1,i), theta3_all(1,i), theta4_all(1,i)];
    disp('final position');
    figure(2)
    plot_manipulator(X,X_des)
end