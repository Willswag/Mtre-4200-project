ac = 2.5
dc =0
thc=pi/2
alc =pi/6

    %calc the kinematics of the robotic arm
    t = [cos(thc), -sin(thc)*cos(alc),  sin(thc)*sin(alc), ac*cos(thc);
         sin(thc),  cos(thc)*cos(alc), -cos(thc)*sin(alc), ac*sin(thc);
         0,         sin(alc),           cos(alc),          dc;
         0,         0,                  0,                 1]
     
        
    %store kinematic matrixes in cell     
    
    
    
    h1 = [-.2588 0 .9659 0;
          0.9659 0 .2588 0;
          0 1 0 1.5;
          0 0 0 1]
    h2= [-.2588 -.3304 .9077 -.7765;
         .9659 -.0885 0.2432 2.8980;
         0 .9397 .3420 1.5;
         0 0 0 1]
     
    h3 = [-.2588 -.3304 .9077 -1.1910;
        .9659 -.0885 .2432 4.443;
        0 .9397 .3420 1.5;
        0 0 0 1]
    