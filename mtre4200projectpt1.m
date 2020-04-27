% Spencer Waguespack
% 4/8/2020
% 
% mtre 4200 project first part
% 
% calculates the h matrix for a robot using DH params
%thanks to robotiq with their website post
%https://blog.robotiq.com/how-to-calculate-a-robots-forward-kinematics-in-5-easy-steps
types = [,0,0,0, 1, 0, 0, 1, 0]
%control comands for the arm
coms = [0, 0, 0, 0, 0, 0]; 

%the distance between the previous x-axis and the current x-axis, along the previous z-axis.
d = [0,1.5+coms(2),.5,0,4+coms(5),2,0]; 

%the angle around the z-axis between the previous x-axis and the current x-axis.
thet = [coms(1), 0, coms(3), coms(4)+pi/2, 0, coms(6),0]; 


%the length of the common normal, which is the distance between the previous z-axis and the current z-axis
a = [0, 0, 0, 0, 0, 0,-.5 ];

%the angle around the common normal to between the previous z-axis and current z-axis.
alph = [-pi/2,0, pi/2, pi/2, 0, -pi/2,0];

%storage cell for each H matrix of the robot
h = cell(6,1);

%loop through the joints anc calculate kinematics

[mod H o z] = for_kin(d,thet,a,alph)
% for i = 1:7
%     
%     %pull the proper params from the param cells
%     dc = cell2mat(d(i));
%     thc = cell2mat(thet(i));
%     ac = cell2mat(a(i));
%     alc =  cell2mat(alph(i));
% 
%     %calc the kinematics of the robotic arm
%     t = [cos(thc), -sin(thc)*cos(alc),  sin(thc)*sin(alc), ac*cos(thc);
%          sin(thc),  cos(thc)*cos(alc), -cos(thc)*sin(alc), ac*sin(thc);
%          0,         sin(alc),           cos(alc),          dc;
%          0,         0,                  0,                 1];
%         
%     %store kinematic matrixes in cell
%     h(i) = mat2cell(t,4,4);
%         
% end
th1  = deg2rad([180; 0;273; 323;283]);
d2  = [.3; .2;.5;.7;1];
th3 = deg2rad([180;280;390;166;0]);
th4 = deg2rad([180;65;49;88;44]);
d5 = [1.5;.8;1;.6;.1];
th6 = deg2rad([180;210;288;233;258;]);

joints = [th1 d2, th3 th4 d5 th6];

eepose=zeros(3,length(joints(:,1)));
for i = 1:length(joints(:,1))

%control comands for the arm
coms = joints(i,:); 

%the distance between the previous x-axis and the current x-axis, along the previous z-axis.
d = [0,1.5+coms(2),.5,0,4+coms(5),2,0]; 

%the angle around the z-axis between the previous x-axis and the current x-axis.
thet = [coms(1), 0, coms(3), coms(4)+pi/2, 0, coms(6),0]; 


%the length of the common normal, which is the distance between the previous z-axis and the current z-axis
a = [0, 0, 0, 0, 0, 0,-.5 ];

%the angle around the common normal to between the previous z-axis and current z-axis.
alph = [-pi/2,0, pi/2, pi/2, 0, -pi/2,0];

%storage cell for each H matrix of the robot
h = cell(6,1);

%loop through the joints anc calculate kinematics

[mod H o z] = for_kin(d,thet,a,alph);
eepose(:,i) = mod(1:3,4,7);
end

jac = jac6(mod,types);


