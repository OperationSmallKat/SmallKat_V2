%Quadruped Leg Kinematics

%% Forward Kinematics

%DH Table for leg

% _________________________________________
% | Link |  a   | alpha |   d   | theta   |
% | Base |  0   |  -90  |   0   |   0     |
% |   1  | .161 |   90  |   0   | q1-pi/2 |
% |   2  | 1.5  |   0   |   0   | q2      |
% |   3  | 1.5  |   0   |   0   | q3      |
% _________________________________________

%Create DH Transformations

syms a1 a2 a3 A1 A2 d1 q1 q2 q3 
TB0 = simplify(MatrixTransform(0,A1,0,0));
T00 = simplify(MatrixTransform(a1,A2,0,q1+deg2rad(90)));
T01 = simplify(MatrixTransform(a2,0,0,q2));
T12 = simplify(MatrixTransform(a3,0,0,q3));

%Forward Position Kinematics
T03 = simplify(TB0*T00*T01*T12);
x = simplify(T03(1,4))
y = simplify(T03(2,4))
z = simplify(T03(3,4))

%% Force Propagation 

J = jacobian([x, y, z], [q1, q2, q3]);
Jt = transpose(J)
%Transpose(J)*F = torques on joints


