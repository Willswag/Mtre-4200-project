function jac = jac6(model,types)
z(:,:,1) = [0;0;0];
o(:,:,1) = [0;0;0];
%def relevent vectors
dof = length(model(1,1,:))
z(:,:,2:8) = model(1:3,3,:);
o(:,:,2:8) = model(1:3,4,:);
jac=zeros(6,dof);
%check for revolute or prismatic
for i = 2:dof+1
    if types(i) == 0
        jac(1:3,i) = cross(z(:,:,i-1),model(1:3,4,dof)-o(:,:,i-1));
        jac(4:6,i) = z(:,:,i-1);
    else
        jac(1:3,i) = z(:,:,i-1); 
        jac(4:6,i) = [0;0;0];
    end
end
jac = jac(:,3:8)
end
