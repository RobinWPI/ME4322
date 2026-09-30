% six bar linkage
clc;
clear;

% define the joints
A = [7 4 0];
B = [5 16 0];
C = [25 25 0];
D = [23 10 0];
E = [18 35 0];
F = [43 32 0];
G = [45 17 0];

% define the lengths of the links
lAB = norm(B - A);
lBC = norm(C - B);
lCD = norm(D - C);
lBE = norm(E - B);
lEF = norm(F - E);
lFG = norm(G - F);

% weight of each link
WAB = [0 -1 0];
WBEC = [0 -1 0];
WCD = [0 -1 0];
WEF = [0 -1 0];
WFG = [0 -1 0];

% center of mass of each link
S1 = (A+B)/2;
S2 = (B+C+E)/3;%not accurate
S3 = (C+D)/2;
S4 = (E+F)/2;
S5 = (F+G)/2;

syms FAx FAy FBx FBy FCx FCy FDx FDy FEx FEy FFx FFy FGx FGy Tin

ForceA = [FAx FAy 0];
ForceB = [FBx FBy 0];
ForceC = [FCx FCy 0];
ForceD = [FDx FDy 0];
ForceE = [FEx FEy 0];
ForceF = [FFx FFy 0];
ForceG = [FGx FGy 0];
InputTorque = [0 0 Tin];

% applied force
AppliedForce = [50 0 0];

% static equilibrium conditions for link AB
% Sum of forces = 0
% Fa + Fb + WeightAB = 0
eqn1 = ForceA + ForceB + WAB == 0;

% sum of moments = 0 with respect of CoM of link AB
% S1A x FA + S1B x FB + InputTorque = 0
eqn2 = cross(A-S1, ForceA) + cross(B-S1,ForceB) + InputTorque == 0;

% static equilibrium conditions for link BEC
% sum of forces = 0
% -Fb + Fc + Fe + WBEC = 0
eqn3 = -ForceB + ForceC + ForceE +WBEC == 0;


% sum of moments = 0 with respect of CoM of link BEC
% S2B x -Fb + S2C x Fc + S2E x Fe = 0
eqn4 = cross(B-S2, -ForceB) + cross(C-S2,ForceC) + cross(E-S2, ForceE) == 0;

% static equilibrium conditions for link CD
% Sum of forces = 0
% -Fc + Fd + WCD = 0
eqn5 = -ForceC + ForceD + WCD == 0;

% sum of moments = 0 with respect of CoM of link CD
% S3C x -Fc + S3D x Fd = 0
eqn6 = cross(C-S3, -ForceC) + cross(D-S3, ForceD) == 0;

% static equilibrium conditions for link EF
% sum of forces = 0
% -Fe + Ff + WEF = 0
eqn7 = -ForceE + ForceF + WEF == 0;

% sum of moments = 0 with respect of CoM of link EF
% S4E x -Fe + S4F x Ff = 0
eqn8 = cross(E-S4, -ForceE) + cross(F-S4, ForceF) == 0;

% static equilibrium conditions for link FG
% sum of forces = 0
% -Ff + Fg + WFG + AppliedForce = 0
eqn9 = -ForceF + ForceG + WFG + AppliedForce == 0;
 
% sum of moments = 0 with respect of CoM of link FG
% S5F x - Ff + S5G x Fg = 0
eqn10 = cross(F-S5, -ForceF) + cross(G-S5, ForceG) == 0;

% Solve the system of equations
eqnMatrix = [eqn1; eqn2; eqn3; eqn4; eqn5; eqn6; eqn7; eqn8; eqn9; eqn10];
StaticSolution = solve(eqnMatrix, [FAx, FAy, FBx, FBy, FCx, FCy, FDx, FDy, FEx, FEy, FFx, FFy, FGx, FGy, Tin]);
Force_Ax = double(StaticSolution.FAx);
Force_Ay = double(StaticSolution.FAy);
Force_Cx = double(StaticSolution.FCx);
Force_Cy = double(StaticSolution.FCy);
Force_Dx = double(StaticSolution.FDx);
Force_Dy = double(StaticSolution.FDy);
Force_Ex = double(StaticSolution.FEx);
Force_Ey = double(StaticSolution.FEy);
Force_Fx = double(StaticSolution.FFx);
Force_Fy = double(StaticSolution.FFy);
Force_Gx = double(StaticSolution.FGx);
Force_Gy = double(StaticSolution.FGy);
Input_Torque = double(StaticSolution.Tin);

% Display the calculated forces and input torque
disp('Calculated Forces and Input Torque:');
disp(['Force Ax: ', num2str(Force_Ax)]);
disp(['Force Ay: ', num2str(Force_Ay)]);
disp(['Force Cx: ', num2str(Force_Cx)]);
disp(['Force Cy: ', num2str(Force_Cy)]);
disp(['Force Dx: ', num2str(Force_Dx)]);
disp(['Force Dy: ', num2str(Force_Dy)]);
disp(['Force Ex: ', num2str(Force_Ex)]);
disp(['Force Ey: ', num2str(Force_Ey)]);
disp(['Force Fx: ', num2str(Force_Fx)]);
disp(['Force Fy: ', num2str(Force_Fy)]);
disp(['Force Gx: ', num2str(Force_Gx)]);
disp(['Force Gy: ', num2str(Force_Gy)]);
disp(['Input Torque: ', num2str(Input_Torque)]);

% Angular Velocity Calculation
% Loop ABCDA

syms wBEC wCD
omega_AB = [0 0 1];
omega_BEC = [0 0 wBEC];
omega_CD = [0 0 wCD];

eqn11 = cross(omega_AB, B-A) + cross(omega_BEC, C-B) + cross(omega_CD, D-C) == 0;

loopsolution = solve(eqn11,[wBEC, wCD]);

% Calculate angular velocities from the loop solution
angularVelocity_BEC = double(loopsolution.wBEC);
angularVelocity_CD = double(loopsolution.wCD);

% Calculate angular velocities for links CD and BEC
disp('Calculated Angular Velocities:');
disp(['Angular Velocity BEC: ', num2str(angularVelocity_BEC)]);
disp(['Angular Velocity CD: ', num2str(angularVelocity_CD)]);


% Angular Velocity Calculation
% Loop DCEFGD

omegaCD = [0 0 angularVelocity_CD];
omegaBEC = [0 0 angularVelocity_BEC];

syms wEF wFG
omega_EF = [0 0 wEF];
omega_FG = [0 0 wFG];

eqn12 = cross(omegaCD,C-D) + cross(omegaBEC,E-C) + cross(omega_EF, F-E) + cross(omega_FG, G-F) == 0;

loopsolution = solve(eqn12,[wEF, wFG]);

% Calculate angular velocities from the loop solution
angularVelocity_EF = double(loopsolution.wEF);
angularVelocity_FG = double(loopsolution.wFG);

% Calculate angular velocities for links CD and BEC
disp('Calculated Angular Velocities:');
disp(['Angular Velocity EF: ', num2str(angularVelocity_EF)]);
disp(['Angular Velocity FG: ', num2str(angularVelocity_FG)]);


% Angular Acceleration Calculation
% ABCDA

syms aBEC aCD
alpha_AB = [0 0 0];
alpha_BEC = [0 0 aBEC];
alpha_CD = [0 0 aCD];

a_B_A = cross(alpha_AB, B-A) + cross(omega_AB, cross(omega_AB, B-A));
a_C_B = cross(alpha_BEC, C-B) + cross(omegaBEC, cross(omegaBEC, C-B));
a_D_C = cross(alpha_CD, D-C) + cross(omegaCD, cross(omegaCD, D-C));

eqn13 = a_B_A + a_C_B + a_D_C == 0;

loop1AccSolution = solve(eqn13, [aBEC aCD]);

% Calculate angular accelerations for links CD and BEC
angularAcceleration_BEC = double(loop1AccSolution.aBEC);
angularAcceleration_CD = double(loop1AccSolution.aCD);

disp('Calculated Angular Acceleration:');
disp(['Angular Acceleration BEC: ', num2str(angularAcceleration_BEC)]);
disp(['Angular Acceleration CD: ', num2str(angularAcceleration_CD)]);


% Angular Acceleration Calculation
% DCEFGD

alphaCD_vector = [0 0 angularAcceleration_CD];
alphaBEC_vector = [0 0 angularAcceleration_BEC];

syms aEF aFG

alpha_EF = [0 0 aEF];
alpha_FG = [0 0 aFG];

a_C_D = cross(alphaCD_vector, C-D) + cross(omegaCD, cross(omegaCD, C-D));
a_E_C = cross(alphaBEC_vector, E-C) + cross(omegaBEC, cross(omegaBEC, E-C));

angvel_EF = [0 0 angularVelocity_EF];
angvel_FG = [0 0 angularVelocity_FG];

a_F_E = cross(alpha_EF, F-E) + cross(angvel_EF, cross(angvel_EF, F-E));

a_E_G = cross(alpha_FG, G-F) + cross(angvel_FG, cross(angvel_FG, G-F));


eqn14 = a_C_D + a_E_C + a_F_E + a_E_G == 0;

loop2AccSolution = solve(eqn14, [aEF, aFG]);

% Calculate angular accelerations for links EF and FG
angularAcceleration_EF = double(loop2AccSolution.aEF);
angularAcceleration_FG = double(loop2AccSolution.aFG);


disp('Calculated Angular Acceleration:');
disp(['Angular Acceleration EF: ', num2str(angularAcceleration_EF)]);
disp(['Angular Acceleration FG: ', num2str(angularAcceleration_FG)]);



% Velocity at joint 

v_B_A = cross(omega_AB, B-A);

%VE_A = V_E_B + V_B_A

v_E_B = cross(omegaBEC, E-B);

vE_A = v_E_B + v_B_A;

% Velocity at joint E

disp('Calculated Velocity at Joint E:');
disp(['Velocity E_A: ', num2str(vE_A)]);

% Velocity at joint G

V_S4_F = cross(angvel_EF,S4-F);

V_F_G = cross(angvel_FG, F-G);

vS4_G = V_S4_F + V_F_G;

disp('Calculated Velocity at JointS4:');
disp(['Velocity S4_G: ', num2str(vS4_G)]);

% Complete Kinematic Results

disp('Input Link Angular Motion:');
disp(['Angular Velocity AB: ', num2str(omega_AB(3))]);
disp(['Angular Acceleration AB: ', num2str(alpha_AB(3))]);


% Velocities at Joints Relative to Reference Joints

v_A_ground = [0 0 0];
v_C_D = cross(omegaCD, C-D);
v_D_ground = [0 0 0];
v_E_A = vE_A;
v_F_G = V_F_G;
v_G_ground = [0 0 0];

disp('Calculated Velocities at Joints:');
disp(['Velocity A_ground: ', num2str(v_A_ground)]);
disp(['Velocity B_A: ', num2str(v_B_A)]);
disp(['Velocity C_D: ', num2str(v_C_D)]);
disp(['Velocity D_ground: ', num2str(v_D_ground)]);
disp(['Velocity E_A: ', num2str(v_E_A)]);
disp(['Velocity F_G: ', num2str(v_F_G)]);
disp(['Velocity G_ground: ', num2str(v_G_ground)]);


% Velocities at Centers of Mass Relative to Closest Ground Joints

vS1_A = cross(omega_AB, S1-A);

v_S2_C = cross(omegaBEC, S2-C);
vS2_D = v_C_D + v_S2_C;

vS3_D = cross(omegaCD, S3-D);

vS5_G = cross(angvel_FG, S5-G);


disp('Calculated Velocities at Centers of Mass:');
disp(['Velocity S1_A: ', num2str(vS1_A)]);
disp(['Velocity S2_D: ', num2str(vS2_D)]);
disp(['Velocity S3_D: ', num2str(vS3_D)]);
disp(['Velocity S4_G: ', num2str(vS4_G)]);
disp(['Velocity S5_G: ', num2str(vS5_G)]);


% Accelerations at Joints Relative to Reference Joints

alphaEF_vector = [0 0 angularAcceleration_EF];
alphaFG_vector = [0 0 angularAcceleration_FG];

a_A_ground = [0 0 0];
a_D_ground = [0 0 0];
a_G_ground = [0 0 0];

a_E_B = cross(alphaBEC_vector, E-B) + cross(omegaBEC, cross(omegaBEC, E-B));
a_E_A = a_B_A + a_E_B;

a_F_G = cross(alphaFG_vector, F-G) + cross(angvel_FG, cross(angvel_FG, F-G));

disp('Calculated Accelerations at Joints:');
disp(['Acceleration A_ground: ', num2str(a_A_ground)]);
disp(['Acceleration B_A: ', num2str(a_B_A)]);
disp(['Acceleration C_D: ', num2str(a_C_D)]);
disp(['Acceleration D_ground: ', num2str(a_D_ground)]);
disp(['Acceleration E_A: ', num2str(a_E_A)]);
disp(['Acceleration F_G: ', num2str(a_F_G)]);
disp(['Acceleration G_ground: ', num2str(a_G_ground)]);


% Accelerations at Centers of Mass Relative to Closest Ground Joints

aS1_A = cross(alpha_AB, S1-A) + cross(omega_AB, cross(omega_AB, S1-A));

a_S2_C = cross(alphaBEC_vector, S2-C) + cross(omegaBEC, cross(omegaBEC, S2-C));
aS2_D = a_C_D + a_S2_C;

aS3_D = cross(alphaCD_vector, S3-D) + cross(omegaCD, cross(omegaCD, S3-D));

a_S4_F = cross(alphaEF_vector, S4-F) + cross(angvel_EF, cross(angvel_EF, S4-F));
aS4_G = a_F_G + a_S4_F;

aS5_G = cross(alphaFG_vector, S5-G) + cross(angvel_FG, cross(angvel_FG, S5-G));

disp('Calculated Accelerations at Centers of Mass:');
disp(['Acceleration S1_A: ', num2str(aS1_A)]);
disp(['Acceleration S2_D: ', num2str(aS2_D)]);
disp(['Acceleration S3_D: ', num2str(aS3_D)]);
disp(['Acceleration S4_G: ', num2str(aS4_G)]);
disp(['Acceleration S5_G: ', num2str(aS5_G)]);


%Newton Second Law Implementation

MassAB = 1;
MassBEC = 1;
MassCD = 1;
MassEF = 1;
MassFG = 1;

%Mass of moment inertia
J_AB = 1;
J_BEC = 1;
J_CD = 1;
J_EF = 1;
J_FG = 1;

syms NFAx NFAy NFBx NFBy NFCx NFCy NFDx NFDy NFEx NFEy NFFx NFFy NFGx NFGy NTin


%Define Forces
NForceA = [NFAx NFAy 0];
NForceB = [NFBx NFBy 0];
NForceC = [NFCx NFCy 0];
NForceD = [NFDx NFDy 0];
NForceE = [NFEx NFEy 0];
NForceF = [NFFx NFFy 0];
NForceG = [NFGx NFGy 0];
NInputTorque = [0 0 NTin];

%Equation for link AB
%sum of forces
eqn15 = NForceA + NForceB +WAB == MassAB * aS1_A;
%sum of moment
eqn16 = cross(A-S1,NForceA)+cross(B-S1,NForceB)+NInputTorque == J_AB * alpha_AB;

%Equation for link BEC
%sum of forces
eqn17 = -NForceB + NForceE + NForceC + WBEC == MassBEC * aS2_D;

%sum of moments
eqn18 = cross(B-S2, -NForceB)+ cross(C-S2, NForceC) + cross(E-S2, NForceE) == J_BEC * alphaBEC_vector;

%Equations for link CD
%sum of forces
eqn19 = -NForceC + NForceD + WCD == MassCD * aS3_D;

%sum of moment
eqn20 = cross(C -S3, -NForceC) + cross(D-S3, NForceD) == J_CD * alphaCD_vector;


%Equation of link EF
%sum of forces
eqn21 = -NForceE + NForceF + WEF == MassEF * aS4_G;

%sum of moment
eqn22 = cross(E -S4, -NForceE) + cross(F-S4, NForceF) == J_EF * alphaEF_vector;


%Equation of link FG
%sum of forces
eqn23 = -NForceF + NForceG + WFG == MassFG * aS5_G;

%sum of moment
eqn24 = cross(F-S5, -NForceF) + cross(G-S5, NForceG) == J_FG * alphaFG_vector;


%Solving equations
NeqnMatrix = [eqn15, eqn16 , eqn17 , eqn18, eqn19, eqn20, eqn21, eqn22, eqn23, eqn24];
DynamicSolution = solve(NeqnMatrix, [NFAx, NFAy, NFBx, NFBy, NFCx, NFCy, NFDx, NFDy, NFEx, NFEy, NFFx, NFFy, NFGx, NFGy, NTin])


%Extract forces from the dynamic solution
NForce_Ax = double(DynamicSolution.NFAx);
NForce_Ay = double(DynamicSolution.NFAy);
NForce_Bx = double(DynamicSolution.NFBx);
NForce_By = double(DynamicSolution.NFBy);
NForce_Cx = double(DynamicSolution.NFCx);
NForce_Cy = double(DynamicSolution.NFCy);
NForce_Dx = double(DynamicSolution.NFDx);
NForce_Dy = double(DynamicSolution.NFDy);
NForce_Ex = double(DynamicSolution.NFEx);
NForce_Ey = double(DynamicSolution.NFEy);
NForce_Fx = double(DynamicSolution.NFFx);
NForce_Fy = double(DynamicSolution.NFFy);
NForce_Gx = double(DynamicSolution.NFGx);
NForce_Gy = double(DynamicSolution.NFGy);
NTorqueInput1 = double(DynamicSolution.NTin);

%Display the result
disp(['NForce_Ax:',num2str(NForce_Ax)]);
disp(['NForce_Ay:',num2str(NForce_Ay)]);

disp(['NForce_Bx:',num2str(NForce_Bx)]);
disp(['NForce_By:',num2str(NForce_By)]);

disp(['NForce_Cx:',num2str(NForce_Cx)]);
disp(['NForce_Cy:',num2str(NForce_Cy)]);

disp(['NForce_Dx:',num2str(NForce_Dx)]);
disp(['NForce_Dy:',num2str(NForce_Dy)]);

disp(['NForce_Ex:',num2str(NForce_Ex)]);
disp(['NForce_Ey:',num2str(NForce_Ey)]);

disp(['NForce_Fx:',num2str(NForce_Fx)]);
disp(['NForce_Fy:',num2str(NForce_Fy)]);

disp(['NForce_Gx:',num2str(NForce_Gx)]);
disp(['NForce_Gy:',num2str(NForce_Gy)]);


disp(['NTorqueInput:',num2str(NTorqueInput1)]);


%Position Analysis
%Loop equation technique --> solving non-linear equations
%Circle intersection --> geometric
%-->Two circles
%   -->(1)new position of joint B
%   -->Current orientation of link AB
%   -->theta = 99.46 deg \_ ccw 
%   -->increment theta by 1 deg 
%   -->new position of joint B
%   -->With B' as center
%   -->BC as radius, draw a circle
%   -->With D as center
%   -->DC as radius, draw a circle
%   -->intersection of the circle gives new position of joint C
%   -->only works when in a 4 bar loop


%joint coordinates have been defined 
%length of links also defined

%compute initial angle of input link AB

% Position Analysis for the new configuration
initial_theta = atan2(B(2)-A(2), B(1)-A(1)); 

if(initial_theta<0)
    inputAngle= 2*pi + initial_theta;
else
    inputAngle = initial_theta;
end


for theta = 1:1:360
    
    %new position of joint B

    B_new = A + [lAB*cos(inputAngle+deg2rad(theta)) lAB*sin(inputAngle+deg2rad(theta)) 0];


[Cx, Cy] = circcirc(B_new(1),B_new(2),lBC,D(1),D(2),lCD);

end