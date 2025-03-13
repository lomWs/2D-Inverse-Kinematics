% Iterative version of inverse2FreedomGrad function.
% Suppose we want to move the links, from P1 -> P2 . 
% We can exploit the line parameterization: p(lambda)= P1+labda*(P2-P1)
%
% Input:
%P1, 2D starting point
%P2, 2D arrival point
%L1 first link length
%L2 second link length
% Output:
% anglesPoint, matrix of (length(lambda) x 2)

function [anglesPoint]= moovingLink(P1,P2, L1,L2)
    %create discrete lambda values with 0<lambda<1
    startValue = 0;
    endValue = 1;
    stepSize= 0.1;
    lambda = startValue:stepSize:endValue

    %instanziate output vector 
    anglesPoint= zeros(length(lambda), 2);

    %for each lambda value, obtain the angle of the links
    for i = 1:length(lambda)
        p=P1+lambda(i)*(P2-P1)
        anglesPoint(i,:) = inverse2FreedomGrad(L1,L2,p(1,1), p(1,2),0,atan2(p(1,2),p(1,1)));
    end



end

