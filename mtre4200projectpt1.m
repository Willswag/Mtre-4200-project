coms = [0, 0, 0, 0, 0, 0];
d = {.5,1.5+coms(2),2,4+coms(5),.5,0}; 
thet = {coms(1), 0, coms(3), coms(4), 0, coms(6)};
a = {0, 0, 0, 0, 0, 0, 0, };
alph = {pi/2, 0, pi/2, pi/2, 0, -pi/2};
h = cell(6,1);

for i = 1:6
    dc = cell2mat(d(i));
    thc = cell2mat(thet(i));
    ac = cell2mat(a(i));
    alc =  cell2mat(alph(i));

    t = [cos(thc), -sin(thc)*cos(alc), sin(thc)*sin(alc), ac*cos(thc);
            sin(thc), cos(thc)*sin(alc), -cos(thc)*sin(alc), ac*cos(thc);
            0, sin(alc), cos(alc), dc;
            0, 0, 0, 1];
    h(i) = mat2cell(t,4,4)
        
end

r = eye(4:4)
for i = 1:6
    r = r*cell2mat(h(i))
end