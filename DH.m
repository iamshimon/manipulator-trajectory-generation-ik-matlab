%This is the implementation of DH convention
function H = DH(a,alpha,d,theta)

cth = cos(theta);
sth = sin(theta);
cal = cos(alpha);
sal = sin(alpha); 

H_z_theta = [cth -sth 0 0; 
               sth  cth 0 0;
                0     0 1 0;
                0     0 0 1];
            
H_z_d = [1 0 0 0;
             0 1 0 0;
             0 0 1 d;
             0 0 0 1];
       
H_x_a = [1 0 0 a;
                 0 1 0 0;
                 0 0 1 0;
                 0 0 0 1];
          
H_x_alpha = [1   0   0   0;
               0  cal -sal 0;
               0  sal  cal 0;
               0   0    0  1];
           
H = H_z_theta*H_z_d*H_x_a*H_x_alpha;


