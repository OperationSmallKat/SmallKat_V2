% Torque Graphs

%DH Table for leg

% _________________________________________
% | Link |  a   | alpha |   d   | theta   |
% | Base |  0   |  -90  |   0   |   0     |
% |   1  | .161 |   90  |   0   | q1-pi/2 |
% |   2  | 1.5  |   0   | -.161 | q2      |
% |   3  | 1.5  |   0   |   0   | q3      |
% _________________________________________

%Create DH Transformations

syms a1 a2 a3 A1 A2 d1 q1 q2 q3 
TB0 = simplify(MatrixTransform(0,A1,0,0));
T00 = simplify(MatrixTransform(a1,A2,0,q1+deg2rad(90)));
T01 = simplify(MatrixTransform(a2,0,d1,q2));
T12 = simplify(MatrixTransform(a3,0,0,q3));

%Forward Position Kinematics
T03 = simplify(TB0*T00*T01*T12);
x = simplify(T03(1,4))
y = simplify(T03(2,4))
z = simplify(T03(3,4))


J = jacobian([x, y, z], [q1, q2, q3]);
Jt = transpose(J);
%Transpose(J)*F = torques on joints

var = [a1, a2, a3, A1, A2, d1, q1];
val  = [.161, 1.5, 1.5, -deg2rad(90), deg2rad(90), -.161, deg2rad(0)];

Jt = subs(Jt,var, val);
T = [0;0;-.5];
FTK = Jt*T

% q2 = deg2rad(90);
% q3 = deg2rad(10);
% F = [-161/1000;...
%      - (3*sin(q2))/2 - (3*cos(q2)*sin(q3))/2 - (3*cos(q3)*sin(q2))/2;...
%      - (3*cos(q2)*sin(q3))/2 - (3*cos(q3)*sin(q2))/2]

U = -45: 1: 45; 
V =   0: 1: 90;
[q2,q3] = meshgrid(U,V);
F2 =  - (3*sin(deg2rad(q2)))/4 - (3*cos(deg2rad(q2)).*sin(deg2rad(q3)))/4 - (3*cos(deg2rad(q3)).*sin(deg2rad(q2)))/4;
%F3 =  - (3*cos(deg2rad(q2)).*sin(deg2rad(q3)))/4 - (3*cos(deg2rad(q3)).*sin(deg2rad(q2)))/4;
surf(q2,q3,F2);

