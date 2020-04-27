%robotics tool box

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

body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');

setFixedTransform(jnt1,dhparams(1,:),'dh');
body1.Joint = jnt1;

addBody(robot,body1,'base')