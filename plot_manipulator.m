%%%This function is used to animate/plot the manipulator

function plot_manipulator(X0,X_des)

theta1=X0(1); theta2=X0(2); theta3=X0(3); theta4=X0(4);

global a1 d1 alpha1 
global a2 d2 alpha2
global a3 d3  alpha3 
global a4 d4 alpha4 

% Now let us plot the results.
H01 = DH(a1,alpha1,d1,theta1); 
H12 = DH(a2,alpha2,d2,theta2); 
H23 = DH(a3,alpha3,d3,theta3); 
H34 = DH(a4,alpha4,d4,theta4);


%Location of joint 1
endOfLink1 = H01(1:3,4);

%Location of joint 2
H02 = H01*H12;
endOfLink2 = H02(1:3,4);

%Location of joint 3
H03 = H02*H23;
endOfLink3 = H03(1:3,4);

%Location of joint 4
H04 = H03*H34;
endOfLink4 = H04(1:3,4);


%End effector is plotted seperately and the links will be plotted
%seperately. We can see that in the end the end effector position aligns
%with the final/last link to which end effector is attached

%Consider plotting end-effector only if two arguments are passed
if (nargin>1) 
    x_des = X_des(1);
    y_des = X_des(2);
    %z_des = X_des(3);
    %Plot the point where we want the end-effector
    plot(x_des,y_des,'o','MarkerSize',1.5,'MarkerFaceColor','black');
    hold on; %Ensures that dot on screen does not dissappear
end


%Draw line from origin to end of link 1
h1=line([0 endOfLink1(1)],[0 endOfLink1(2)],[0 endOfLink1(3)],....
      'LineWidth',5,'Color','red');

%Draw line from end of link 1 to end of link 2
h2=line([endOfLink1(1) endOfLink2(1)],...
     [endOfLink1(2) endOfLink2(2)],...
     [endOfLink1(3) endOfLink2(3)],...
     'LineWidth',5,'Color','blue');
 
%Draw line from end of link 2 to end of link 3
h3=line([endOfLink2(1) endOfLink3(1)],...
     [endOfLink2(2) endOfLink3(2)],...
     [endOfLink2(3) endOfLink3(3)],...
     'LineWidth',5,'Color','green');
 
%Draw line from end of link 3 to end of link 4
h4=line([endOfLink3(1) endOfLink4(1)],...
     [endOfLink3(2) endOfLink4(2)],...
     [endOfLink3(3) endOfLink4(3)],...
     'LineWidth',5,'Color','yellow');
 

 
xlabel('x');
ylabel('y');
grid on; %if you want the grid to show up.
axis('equal'); %make the axis equal, to avoid scaling effect



%These set the x and y limits for the axis (will need adjustment)
xlim([-1 1]); 
ylim([-1 1]);

pause(0.2);
delete(h1);
delete(h2);
delete(h3);
delete(h4);



