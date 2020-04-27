%robotics toolbox animation
%
%mtre 4200 project part 2
clear
%parmaters for the simulation
%point to avoid
pb = [2;.5;0];
%final joint angle
thetf = [pi/2 pi/2];
%atraction gainz
atragains = [10; 100 ];
%size of point to avoid
po = 1;
%repulsion gains
repgains = [0; 10];
%rotational gains
kgains = [.001; .1 ];

%the distance between the previous x-axis and the current x-axis, along the previous z-axis.
d = [ 0 0 ];

%the angle around the z-axis between the previous x-axis and the current x-axis.
thet =  [ 0; 0];

%the length of the common normal, which is the distance between the previous z-axis and the current z-axis
a = [1; 1];

%the angle around the common normal to between the previous z-axis and current z-axis.
alph = [ 0 0 ];

%current location of joints and set up the value for the robot
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

count =200
q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'tool';
o2(:,1) =[0;1;0];
o2(:,2) =[0;1;0] ;
qInitial = q0; % Use home configuration as the initial guess
error = 0
for j = 1:count
[mod1 H1 o1 z1]= for_kin(d,thet,a,alph);
fa = zeros(3,2);
frep = zeros(3,2);
fsum = zeros(3,2);
% calculate repulsion and atraction
for i = 1:length(thet)
   fa(:,i) = atragains(i)*(o2(:,i)-o1(:,i));
   odist = sqrt((o1(1,:,i)-pb(1))^2+(o1(2,i)-pb(2))^2);
   frep(:,i) = (repgains(i)*(1/odist-1/po)*1/odist^2)*(o1(:,:,i)-pb)/norm((o1(:,:,i)-pb));
   fsum(:,i) = fa(:,i)+frep(:,i);
end

j1 = transpose(calc_jacob(H1));
j2 = transpose(calc_jacob(mod1));
tor = j1*fsum(:,1)+j2*fsum(:,2);
thet = tor.*kgains;
rad2deg(thet)
    
    % Store the configuration
    qs(j,:) = thet;
    % Start from prior solution
 data1(:,j) = mod1(1:2,4,2)
end
figure(1)
x = 1:count
gox = -1*ones(size(x));
goy = ones(size(x));
plot(x,data1,x,gox,x,goy)

% figure(2)
% show(robot,qs(1,:)');
% view(2)
% ax = gca;
% ax.Projection = 'orthographic';
% hold on
% axis([-5 5 -5 5])
% 
% 
% framesPerSecond = 60;
% r = rateControl(framesPerSecond);
% for i = 1:count
%     show(robot,qs(i,:)','PreservePlot',false);
%     drawnow
%     waitfor(r);
% end
