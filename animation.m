%robotics toolbox animation

%mtre 4200 project part 2

pb = [2;.5;0];
thetf = [pi/2; pi/2];
atragains = [1; 1 ];
po = 1;
repgains = [1; 1];
%the distance between the previous x-axis and the current x-axis, along the previous z-axis.
d = [ 0 0 ];

%the angle around the z-axis between the previous x-axis and the current x-axis.
thet =  [ 0;  0];

%the length of the common normal, which is the distance between the previous z-axis and the current z-axis
a = [1; 1];

%the angle around the common normal to between the previous z-axis and current z-axis.
alph = [ 0 0 ];

%current location of joints
robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);

body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');

body = rigidBody('link2');
joint = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint, trvec2tform([a(1),0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link1');

body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([a(2), 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link2');


t = (0:0.2:10)'; % Time

points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];

q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'tool';

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


figure
show(robot,qs(1,:)');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2),'k')
axis([-5 5 -5 5])


framesPerSecond = 15;
r = rateControl(framesPerSecond);
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end
