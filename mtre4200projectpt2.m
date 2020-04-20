%mtre 4200 project part 2

pb = [2;.5];
thetf = [0; 0];
%the distance between the previous x-axis and the current x-axis, along the previous z-axis.
d = [ 0 0 ];

%the angle around the z-axis between the previous x-axis and the current x-axis.
thet =  [ pi/2;  pi/2];

%the length of the common normal, which is the distance between the previous z-axis and the current z-axis
a = [1 1];

%the angle around the common normal to between the previous z-axis and current z-axis.
alph = [ 0 0 ];

%location of joints
o = zeros(2);
of = [];


%empty matrix to hold model
H = zeros(4,4,2);

for i = 1:2
    dc = d(i);
    thc = thet(i);
    ac = a(i);
    alc = alph(i);

    t = [cos(thc), -sin(thc)*cos(alc),  sin(thc)*sin(alc), ac*cos(thc);
         sin(thc),  cos(thc)*cos(alc), -cos(thc)*sin(alc), ac*sin(thc);
         0,         sin(alc),           cos(alc),          dc;
         0,         0,                  0,                 1];
    
    H(:,:,i) = t
end

m =  eye(4);
for i = 1:2
    m = m*H(:,:,i)
end

%trajectory with no consideration of point

