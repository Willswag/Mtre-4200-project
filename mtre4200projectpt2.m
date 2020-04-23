%mtre 4200 project part 2

pb = [2;.5;0];
thetf = [pi/2; pi/2];
atragains = [1; 1 ];
po = 1;
repgains = [.1; .1];
kgains = [.5; .5 ];
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

%run the simulation until both points are at the 
while thet(1) ~= thetf(1) && thet(2) ~= thetf(2)
[mod1 H1 o1 z1]= for_kin(d,thet,a,alph);
fa = zeros(3,1,2);
frep = zeros(3,1,2);
fsum = zeros(3,1,2);

for i = 1:length(thet)
   fa(:,:,i) = atragains(i)*(o2(:,:,i)-o1(:,:,i));
   odist = sqrt((o1(1,:,i)-pb(1))^2+(o1(2,:,i)-pb(2))^2);
   frep(:,:,i) = (repgains(i)*(1/odist-1/po)*1/odist^2)*(o1(:,:,i)-pb)/norm((o1(:,:,i)-pb));
   fsum(:,:,i) = fa(:,:,i)+frep(:,:,i);
end
z1(:,:,1) = [0,0,0];
z1(:,:,2) = H1([1:3],3,1);
z1(:,:,3) = mod1([1:3],3);

op(:,:,1) = [0,0,0];
op(:,:,2) = H1([1:3],4,1);
op(:,:,3) = mod1([1:3],4,1);


for i = 1:length(thet)
    jv(:,i) = cross(z1(:,:,i+1),op(:,:,i+1)-op(:,:,i));
    jw(:,i) = z1(:,:,i+1);
end
tor = transpose(jv)*fsum(:,1) + transpose(jv) * fsum(:,2);
thet = kgains.*tor
end
