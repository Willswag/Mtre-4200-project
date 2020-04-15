% Spencer Waguespack
% 4/8/2020
% 
% mtre 4200 project first part
% 
% calculates the h matrix for a robot using DH params
%thanks to robotiq with their website post
%https://blog.robotiq.com/how-to-calculate-a-robots-forward-kinematics-in-5-easy-steps

%control comands for the arm
coms = [0, 0, 0, 0, 0, 0]; 

%the distance between the previous x-axis and the current x-axis, along the previous z-axis.
d = {0,1+coms(2),.5,0,2+coms(5),0,0}; 

%the angle around the z-axis between the previous x-axis and the current x-axis.
thet = {coms(1), 0, coms(3), coms(4), 0, coms(6),-pi/2}; 


%the length of the common normal, which is the distance between the previous z-axis and the current z-axis
a = {.5, 0, 0, 2, 0, 2,.5 };

%the angle around the common normal to between the previous z-axis and current z-axis.
alph = {-pi/2,0, pi/2, pi/2, 0, -pi/2,0};

%storage cell for each H matrix of the robot
h = cell(6,1);

%loop through the joints anc calculate kinematics
for i = 1:7
    
    %pull the proper params from the param cells
    dc = cell2mat(d(i));
    thc = cell2mat(thet(i));
    ac = cell2mat(a(i));
    alc =  cell2mat(alph(i));

    %calc the kinematics of the robotic arm
    t = [cos(thc), -sin(thc)*cos(alc),  sin(thc)*sin(alc), ac*cos(thc);
         sin(thc),  cos(thc)*cos(alc), -cos(thc)*sin(alc), ac*sin(thc);
         0,         sin(alc),           cos(alc),          dc;
         0,         0,                  0,                 1];
        
    %store kinematic matrixes in cell     
    h(i) = mat2cell(t,4,4);
        
end

%init a variable to multiply the multiplication matrixes by
r = eye(4:4);

%loop through the h matrix and calc overall model
for i = 1:7
    
    r = r*cell2mat(h(i))
end