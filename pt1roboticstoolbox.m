%robotics tool box

%control comands for the arm
coms = [0, 0, 0, 0, 0, 0]; 

%the distance between the previous x-axis and the current x-axis, along the previous z-axis.
d = [0;1.5+coms(2);.5;0;4+coms(5);2;0]; 

%the angle around the z-axis between the previous x-axis and the current x-axis.
thet = [coms(1); 0; coms(3); coms(4)+pi/2; 0; coms(6);0]; 


%the length of the common normal, which is the distance between the previous z-axis and the current z-axis
a = [0; 0; 0; 0; 0; 0;-.5 ];

%the angle around the common normal to between the previous z-axis and current z-axis.
alph = [-pi/2;0; pi/2; pi/2; 0; -pi/2;0];

%storage cell for each H matrix of the robot
h = cell(6,1);

%loop through the joints anc calculate kinematics

[mod H o z] = for_kin(d,thet,a,alph)

body1 =rigidBody('body1');

jnt1 = rigidBodyJoint('jnt1','revolute');
jnt1.HomePosition = 0;
tform = H(:,:,1); % User defined
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;

robot = rigidBodyTree('DataFormat','column','MaxNumbodies',7);

addBody(robot,body1,'base')

body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','prismatic');
jnt2.HomePosition = 0; % User defined
tform2 = H(:,:,2); % User defined
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2;
addBody(robot,body2,'body1'); % Add body2 to body1

body3 = rigidBody('body3');
body4 = rigidBody('body4');
jnt3 = rigidBodyJoint('jnt3','revolute');
jnt4 = rigidBodyJoint('jnt4','revolute');
tform3 = H(:,:,3);
tform4 = H(:,:,4);
setFixedTransform(jnt3,tform3);
setFixedTransform(jnt4,tform4);
jnt3.HomePosition = 0; % User defined
body3.Joint = jnt3;
body4.Joint = jnt4;
addBody(robot,body3,'body2'); % Add body3 to body2
addBody(robot,body4,'body3'); % Add body4 to body3


body5 = rigidBody('body5');
body6 = rigidBody('body6');
jnt5 = rigidBodyJoint('jnt5','prismatic');
jnt6 = rigidBodyJoint('jnt6','revolute');
tform5 = H(:,:,5);
tform6 = H(:,:,6); % User defined
setFixedTransform(jnt5,tform5);
setFixedTransform(jnt6,tform6);
jnt5.HomePosition = 0; % User defined
jnt6.HomePosition = 0; % User defined
body5.Joint = jnt5;
body6.Joint = jnt6;
addBody(robot,body5,'body4'); % Add body3 to body2
addBody(robot,body6,'body5'); % Add body4 to body3


bodyEndEffector = rigidBody('endeffector');
tform7 = H(:,:,7);
setFixedTransform(bodyEndEffector.Joint,tform7);
addBody(robot,bodyEndEffector,'body6');

t = (0:0.2:10)'; % Time
count = length(t);
center = [4 2 0];
radius = 1;
theta = t*(2*pi/t(end));
points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];

q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);

ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'endeffector';


qInitial = q0; % Use home configuration as the initial guess
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end

figure(1)
show(robot,qs(1,:)');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2),'k')
axis([-7 7 -7 7])

framesPerSecond = 15;
r = rateControl(framesPerSecond);
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end
