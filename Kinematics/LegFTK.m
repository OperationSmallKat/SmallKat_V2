function FTK = LegFTK(p,T)

%Forward Torque Kinematics
%DH Table for leg

% _________________________________________
% | Link |  a   | alpha |   d   | theta   |
% | Base |  0   |  -90  |   0   |   0     |
% |   1  | .161 |   90  |   0   | q1-pi/2 |
% |   2  | 1.5  |   0   | -.161 | q2      |
% |   3  | 1.5  |   0   |   0   | q3      |
% _________________________________________

%q1 = deg2rad(p(1));
%q2 = deg2rad(p(2));
%q3 = deg2rad(p(3));
q1 = deg2rad(0);
q2 = deg2rad(0);
q3 = deg2rad(0);

T = [0;0;-1];

A1 = -deg2rad(90);
A2 =  deg2rad(90);
d1 = -.161;
a1 = .161;
a2 = 1.5;
a3 = 1.5;

%Tranpose of Jacobian Solved in Leg Kinematics Setup
Jt = [ a3*sin(q3)*(cos(q1)*sin(q2) + cos(A2)*cos(q2)*sin(q1)) - a2*cos(q1)*cos(q2) - a3*cos(q3)*(cos(q1)*cos(q2) - cos(A2)*sin(q1)*sin(q2)) - a1*cos(q1) + a2*cos(A2)*sin(q1)*sin(q2),                                        a3*sin(q3)*(cos(A1)*sin(q1)*sin(q2) - cos(A1)*cos(A2)*cos(q1)*cos(q2)) - a3*cos(q3)*(cos(A1)*cos(q2)*sin(q1) + cos(A1)*cos(A2)*cos(q1)*sin(q2)) - a1*cos(A1)*sin(q1) - a2*cos(A1)*cos(q2)*sin(q1) - a2*cos(A1)*cos(A2)*cos(q1)*sin(q2),                                        a3*sin(q3)*(sin(A1)*sin(q1)*sin(q2) - cos(A2)*sin(A1)*cos(q1)*cos(q2)) - a3*cos(q3)*(sin(A1)*cos(q2)*sin(q1) + cos(A2)*sin(A1)*cos(q1)*sin(q2)) - a1*sin(A1)*sin(q1) - a2*sin(A1)*cos(q2)*sin(q1) - a2*cos(A2)*sin(A1)*cos(q1)*sin(q2);...
             a2*sin(q1)*sin(q2) + a3*cos(q3)*(sin(q1)*sin(q2) - cos(A2)*cos(q1)*cos(q2)) + a3*sin(q3)*(cos(q2)*sin(q1) + cos(A2)*cos(q1)*sin(q2)) - a2*cos(A2)*cos(q1)*cos(q2), a3*sin(q3)*(sin(q2)*(sin(A1)*sin(A2) + cos(A1)*cos(A2)*sin(q1)) - cos(A1)*cos(q1)*cos(q2)) - a3*cos(q3)*(cos(q2)*(sin(A1)*sin(A2) + cos(A1)*cos(A2)*sin(q1)) + cos(A1)*cos(q1)*sin(q2)) - a2*cos(q2)*(sin(A1)*sin(A2) + cos(A1)*cos(A2)*sin(q1)) - a2*cos(A1)*cos(q1)*sin(q2), a2*cos(q2)*(cos(A1)*sin(A2) - cos(A2)*sin(A1)*sin(q1)) + a3*cos(q3)*(cos(q2)*(cos(A1)*sin(A2) - cos(A2)*sin(A1)*sin(q1)) - sin(A1)*cos(q1)*sin(q2)) - a3*sin(q3)*(sin(q2)*(cos(A1)*sin(A2) - cos(A2)*sin(A1)*sin(q1)) + sin(A1)*cos(q1)*cos(q2)) - a2*sin(A1)*cos(q1)*sin(q2);...
                                                               a3*cos(q3)*(sin(q1)*sin(q2) - cos(A2)*cos(q1)*cos(q2)) + a3*sin(q3)*(cos(q2)*sin(q1) + cos(A2)*cos(q1)*sin(q2)),                                                                                       a3*sin(q3)*(sin(q2)*(sin(A1)*sin(A2) + cos(A1)*cos(A2)*sin(q1)) - cos(A1)*cos(q1)*cos(q2)) - a3*cos(q3)*(cos(q2)*(sin(A1)*sin(A2) + cos(A1)*cos(A2)*sin(q1)) + cos(A1)*cos(q1)*sin(q2)),                                                                                       a3*cos(q3)*(cos(q2)*(cos(A1)*sin(A2) - cos(A2)*sin(A1)*sin(q1)) - sin(A1)*cos(q1)*sin(q2)) - a3*sin(q3)*(sin(q2)*(cos(A1)*sin(A2) - cos(A2)*sin(A1)*sin(q1)) + sin(A1)*cos(q1)*cos(q2))];
 
FTK = Jt*T


end