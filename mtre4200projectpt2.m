%mtre 4200 project part 2

pb = [2;.5;0];
thetf = [pi/2; pi/2];
atragains = [1; 2 ];
po = 1;
repgains = [0; 1];
kgains = [1; .5 ];


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
iterat = 0
%run the simulation 200 times
for j = 1:200
%while thet(1) ~= thetf(1) && thet(2) ~= thetf(2)
iterat = iterat+1
[mod1 H1 o1 z1]= for_kin(d,thet,a,alph);
fa = zeros(3,1,2);
frep = zeros(3,1,2);
fsum = zeros(3,2);
% calculate repulsion and atraction
for i = 1:length(thet)
   fa(:,:,i) = atragains(i)*(o2(:,:,i)-o1(:,:,i));
   odist = sqrt((o1(1,:,i)-pb(1))^2+(o1(2,:,i)-pb(2))^2);
   frep(:,:,i) = (repgains(i)*(1/odist-1/po)*1/odist^2)*(o1(:,:,i)-pb)/norm((o1(:,:,i)-pb));
   fsum(:,i) = fa(:,:,i)+frep(:,:,i);
end

jv = calc_jacob(mod1);
for i =1:length(thet)
    tor(i) = dot(transpose(jv(:,i)),fsum(:,i));
end
thet = transpose(tor).*kgains;
rad2deg(thet)
% for i = 1:length(thet)
%     if thet(i) > 2*pi
%         thet(i) = thet(i)-2*pi
%     end
%     if thet(i)<-2*pi
%         thet(i) = thet(i)+2*pi
%     end
% end
end
