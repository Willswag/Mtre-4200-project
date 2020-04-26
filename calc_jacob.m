function jv = calc_jacob(mod)
    dof = length(mod(1,1,:));
    
    %get z and o params
    z = mod([1:3],3,:);
    o = mod([1:3],4,:);
    
    %calc jacob
    jv(:,:,1) = cross([0;0;1],(o(:,:,dof)-[0;0;0]))
    if dof > 1
    for i = 2:dof
        jv(:,i) = cross(z(:,:,i-1),(o(:,:,dof)-o(:,:,i-1)));
    end
    end
end