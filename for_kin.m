function [model H o z] = for_kin(d,thet,a,alph)
    H = zeros(4,4,length(thet));
    o = zeros(3,1,length(thet));
    for i = 1:length(thet)
        dc = d(i);
        thc = thet(i);
        ac = a(i);
        alc = alph(i);

        t = [cos(thc), -sin(thc)*cos(alc),  sin(thc)*sin(alc), ac*cos(thc);
         sin(thc),  cos(thc)*cos(alc), -cos(thc)*sin(alc), ac*sin(thc);
         0,         sin(alc),           cos(alc),          dc;
         0,         0,                  0,                 1];
        
        H(:,:,i) = t;
    end

    model =  eye(4);
for i = 1:length(thet);
    model = model*H(:,:,i);
    o(:,:,i) = [model(1,4);model(2,4);model(3,4)];
    z(:,:,i) = [model(1,3);model(2,3);model(3,3)];

end