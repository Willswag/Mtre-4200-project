pb = [2;.5;0];
thetf = [pi/2; pi/2];
atragains = [5; 5 ];
po = 1;
repgains = [.1; .1];
kgains = [.1; .1 ];


%the distance between the previous x-axis and the current x-axis, along the previous z-axis.
d = [ 0 0 ];

%the angle around the z-axis between the previous x-axis and the current x-axis.
thet =  [ 0;  0];

%the length of the common normal, which is the distance between the previous z-axis and the current z-axis
a = [1 1];

%the angle around the common normal to between the previous z-axis and current z-axis.
alph = [ 0 0 ];

%current location of joints
o1 = zeros(3,1,2);

%calculate robot model in forkin func
[mod1 H1 o1 z1]= for_kin(d,thet,a,alph);
[mod2 H2 o2 z2]= for_kin(d,thetf,a,alph);
j1 = calc_jacob(mod1)