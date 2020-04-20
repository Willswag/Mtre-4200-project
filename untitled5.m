d = [ 0 0 ];

%the angle around the z-axis between the previous x-axis and the current x-axis.
thet =  [ 0;  0];

%the length of the common normal, which is the distance between the previous z-axis and the current z-axis
a = [1 1];

%the angle around the common normal to between the previous z-axis and current z-axis.
alph = [ 0 0 ];

%location of joints
o = zeros(2,1,2);


for_kin(d,thet,a,alph)