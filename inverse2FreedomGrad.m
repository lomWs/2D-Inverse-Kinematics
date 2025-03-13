% This function gives angle link of 2 freedom grads,2d inverse kinematics
% Input:
%L1 first link length
%L2 second link length
%px, x-axis coordinates respect end effector frame
%py, y-axis coordinates respect end effector frame
%pz, suppose pz  given
%theta = atan(py,px), rotation of end effector frame
% Output:
% robotAngles =[q1 , q2], angles of links


function [robotAngles] = inverse2FreedomGrad(L1,L2,px, py, pz,theta)
    %1) check for the condition of existence for the solution:
    if theta ~= atan2(py,px)
        disp("theta must be equals atan2(py,px) ")
        return
    end
    if sqrt(px^2+py^2) > (L1+L2)
        disp("(py,px) is not reachable, out of range")
        return
    end
    if L2<L1 && (sqrt(px^2+py^2) < (L1-L2))
        disp("(py,px) is not reachable, L2<L1 inner cirle")
        return
    end
    %2) Inizialize the system of equation
    c2=(px^2+py^2-L2^2-L1^2)/(2*L1*L2);
    s2=sqrt(1-c2^2); % use s2=-sqrt(1-c2^2) for down elbow solution

    c1s1=(1/(L1^2+L2^2+2*L1*L2*c2))*[L1+L1*c2 , L2*s2; -L2*s2, L1+L2*c2]*[px;py];
    
    c1= c1s1(1:1);
    s1=c1s1(2:2);

    q1=atan2(s1,c1);
    q2=atan2(s2,c2);

    robotAngles=[q1,q2];

end


