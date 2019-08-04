
% 正运动学

%   
function T=D_H(theta,d,a,alpth)

T= [cos(theta) -sin(theta) 0 a;...
    sin(theta)*cos(alpth) cos(theta)*cos(alpth) -sin(alpth) -sin(alpth)*d;...
    sin(theta)*sin(alpth) cos(theta)*sin(alpth) cos(alpth) cos(alpth)*d; ...
    0 0 0 1];
end

