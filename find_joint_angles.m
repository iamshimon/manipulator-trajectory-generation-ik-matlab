%This function finds the values for theta, given the end effector positions
function X = find_joint_angles(X0)

global d1 a1 alpha1 
global d2 a2 alpha2
global d3 a3 alpha3
global d4 a4 alpha4 

global x_des y_des z_des

% Get individual theta's from input
theta1=X0(1); theta2=X0(2); theta3=X0(3); theta4=X0(4);

%Calculate DH matrices

H01 = DH(a1,alpha1,d1,theta1); 
H12 = DH(a2,alpha2,d2,theta2); 
H23 = DH(a3,alpha3,d3,theta3); 
H34 = DH(a4,alpha4,d4,theta4);

H02 = H01*H12;
H03 = H02*H23;
H04 = H03*H34;

endOfLink4 = H04(1:3,4);

%Forming the equation to be solved. This will be then solved using FSolve
X = [endOfLink4(1)-x_des; endOfLink4(2)-y_des; endOfLink4(3)-z_des]; 