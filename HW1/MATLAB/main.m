% Six-bar linkage analysis
clc;
clear;

%% Input speed
parts_required = 12500;
operating_time = 9*3600;
parts_per_revolution = 1;
T_cycle = operating_time*parts_per_revolution/parts_required;
n_AB = 60/T_cycle;
omega_AB = [0 0 2*pi/T_cycle];
alpha_AB = [0 0 0];

%% Settings
material = 'Aluminum 6061-T6';
rho = 2700;
g = 9.81;
step_deg = 0.5;
show_plots = true;
script_folder = fileparts(mfilename('fullpath'));
output_folder = fullfile(script_folder,'results_matlab');
if ~exist(output_folder,'dir'), mkdir(output_folder); end
assert(license('test','Symbolic_Toolbox'),'Symbolic Math Toolbox is required.');

%% Joint coordinates
A = [1.400 0.485 0];
B = [1.670 0.990 0];
C = [0.255 1.035 0];
D = [0.285 0.055 0];
E = [0.195 2.540 0];
F = [-0.980 2.570 0];
G = [0.050 0.200 0];
lFH = 1.843;
H = F+lFH*(F-G)/norm(F-G);

%% Link lengths
lAB = norm(B-A);
lBC = norm(C-B);
lDC = norm(C-D);
lDE = norm(E-D);
lCE = norm(E-C);
lEF = norm(F-E);
lGF = norm(F-G);
lGH = norm(H-G);

%% CAD mass and centroidal inertia
m_AB = 8.03;
m_BC = 19.41;
m_DCE = 33.48;
m_EF = 16.16;
m_GFH = 59.68;
I_S1_z = 0.26;
I_S2_z = 3.39;
I_S3_z = 17.81;
I_S4_z = 1.97;
I_S5_z = 99.17;

%% Weights and payload
W_AB = [0 -m_AB*g 0];
W_BC = [0 -m_BC*g 0];
W_DCE = [0 -m_DCE*g 0];
W_EF = [0 -m_EF*g 0];
W_GFH = [0 -m_GFH*g 0];
Q_static = [0 -200 0];
m_payload = 200/g;

%% Mass centers
% Rounded local CAD coordinates, mapped to the linkage plane.
S1 = A+0.29*(B-A)/lAB;
S2 = C+0.71*(B-C)/lBC;
S3 = D+1.25*(E-D)/lDE;
S4 = F+0.59*(E-F)/lEF;
S5 = G+2.21*(H-G)/lGH;

%% Relative-position vectors
r_B_A = B-A;
r_C_B = C-B;
r_C_D = C-D;
r_E_D = E-D;
r_F_E = F-E;
r_F_G = F-G;
r_H_G = H-G;
r_S1_A = S1-A;
r_S2_B = S2-B;
r_S3_D = S3-D;
r_S4_E = S4-E;
r_S5_G = S5-G;

%% Static equilibrium
syms FAx FAy FBx FBy FCx FCy FDx FDy FEx FEy FFx FFy FGx FGy Tin real
F_A = [FAx FAy 0];
F_B = [FBx FBy 0];
F_C = [FCx FCy 0];
F_D = [FDx FDy 0];
F_E = [FEx FEy 0];
F_F = [FFx FFy 0];
F_G = [FGx FGy 0];
T = [0 0 Tin];
force_variables = [FAx FAy FBx FBy FCx FCy FDx FDy FEx FEy FFx FFy FGx FGy Tin];

% Link AB
eqn1 = F_A+F_B+W_AB == 0;
eqn2 = cross(A-S1,F_A)+cross(B-S1,F_B)+T == 0;
% Link BC
eqn3 = -F_B+F_C+W_BC == 0;
eqn4 = cross(B-S2,-F_B)+cross(C-S2,F_C) == 0;
% Link DCE
eqn5 = F_D-F_C+F_E+W_DCE == 0;
eqn6 = cross(D-S3,F_D)+cross(C-S3,-F_C)+cross(E-S3,F_E) == 0;
% Link EF
eqn7 = -F_E+F_F+W_EF == 0;
eqn8 = cross(E-S4,-F_E)+cross(F-S4,F_F) == 0;
% Link GFH
eqn9 = F_G-F_F+W_GFH+Q_static == 0;
eqn10 = cross(G-S5,F_G)+cross(F-S5,-F_F)+cross(H-S5,Q_static) == 0;

% Use x,y force components and z moments.
eqnMatrix = [eqn1(1:2),eqn2(3),eqn3(1:2),eqn4(3),eqn5(1:2),eqn6(3), ...
    eqn7(1:2),eqn8(3),eqn9(1:2),eqn10(3)];
StaticSolution = solve(eqnMatrix,force_variables);
static_initial = solution_vector(StaticSolution,force_variables);
StaticForces = named_forces(static_initial);

%% Angular velocities
% Loop ABCD
syms wBC wDCE real
omega_BC = [0 0 wBC];
omega_DCE = [0 0 wDCE];
eqn11 = cross(omega_AB,r_B_A)+cross(omega_BC,r_C_B) ...
    -cross(omega_DCE,r_C_D) == 0;
Loop1Velocity = solve(eqn11(1:2),[wBC wDCE]);
omega_BC = [0 0 double(Loop1Velocity.wBC)];
omega_DCE = [0 0 double(Loop1Velocity.wDCE)];

% Loop DEFG
syms wEF wGFH real
omega_EF = [0 0 wEF];
omega_GFH = [0 0 wGFH];
eqn12 = cross(omega_DCE,r_E_D)+cross(omega_EF,r_F_E) ...
    -cross(omega_GFH,r_F_G) == 0;
Loop2Velocity = solve(eqn12(1:2),[wEF wGFH]);
omega_EF = [0 0 double(Loop2Velocity.wEF)];
omega_GFH = [0 0 double(Loop2Velocity.wGFH)];

%% Angular accelerations
% Loop ABCD
syms alphaBC alphaDCE real
alpha_BC = [0 0 alphaBC];
alpha_DCE = [0 0 alphaDCE];
a_B_A = cross(alpha_AB,r_B_A)+cross(omega_AB,cross(omega_AB,r_B_A));
a_C_B = cross(alpha_BC,r_C_B)+cross(omega_BC,cross(omega_BC,r_C_B));
a_C_D = cross(alpha_DCE,r_C_D)+cross(omega_DCE,cross(omega_DCE,r_C_D));
eqn13 = a_B_A+a_C_B-a_C_D == 0;
Loop1Acceleration = solve(eqn13(1:2),[alphaBC alphaDCE]);
alpha_BC = [0 0 double(Loop1Acceleration.alphaBC)];
alpha_DCE = [0 0 double(Loop1Acceleration.alphaDCE)];

% Loop DEFG
syms alphaEF alphaGFH real
alpha_EF = [0 0 alphaEF];
alpha_GFH = [0 0 alphaGFH];
a_E_D = cross(alpha_DCE,r_E_D)+cross(omega_DCE,cross(omega_DCE,r_E_D));
a_F_E = cross(alpha_EF,r_F_E)+cross(omega_EF,cross(omega_EF,r_F_E));
a_F_G = cross(alpha_GFH,r_F_G)+cross(omega_GFH,cross(omega_GFH,r_F_G));
eqn14 = a_E_D+a_F_E-a_F_G == 0;
Loop2Acceleration = solve(eqn14(1:2),[alphaEF alphaGFH]);
alpha_EF = [0 0 double(Loop2Acceleration.alphaEF)];
alpha_GFH = [0 0 double(Loop2Acceleration.alphaGFH)];

%% Joint velocities
v_A = [0 0 0]; v_D = [0 0 0]; v_G = [0 0 0];
v_B = cross(omega_AB,r_B_A);
v_C = cross(omega_DCE,r_C_D);
v_E = cross(omega_DCE,r_E_D);
v_F = cross(omega_GFH,r_F_G);
v_H = cross(omega_GFH,r_H_G);
v_C_check = v_B+cross(omega_BC,r_C_B);
v_F_check = v_E+cross(omega_EF,r_F_E);
assert(norm(v_C-v_C_check)<1e-10 && norm(v_F-v_F_check)<1e-10);

%% Joint accelerations
a_A = [0 0 0]; a_D = [0 0 0]; a_G = [0 0 0];
a_B = cross(alpha_AB,r_B_A)+cross(omega_AB,cross(omega_AB,r_B_A));
a_C = cross(alpha_DCE,r_C_D)+cross(omega_DCE,cross(omega_DCE,r_C_D));
a_E = cross(alpha_DCE,r_E_D)+cross(omega_DCE,cross(omega_DCE,r_E_D));
a_F = cross(alpha_GFH,r_F_G)+cross(omega_GFH,cross(omega_GFH,r_F_G));
a_H = cross(alpha_GFH,r_H_G)+cross(omega_GFH,cross(omega_GFH,r_H_G));
a_C_check = a_B+cross(alpha_BC,r_C_B)+cross(omega_BC,cross(omega_BC,r_C_B));
a_F_check = a_E+cross(alpha_EF,r_F_E)+cross(omega_EF,cross(omega_EF,r_F_E));
assert(norm(a_C-a_C_check)<1e-10 && norm(a_F-a_F_check)<1e-10);

%% Mass-center velocities
v_S1 = cross(omega_AB,r_S1_A);
v_S2 = v_B+cross(omega_BC,r_S2_B);
v_S3 = cross(omega_DCE,r_S3_D);
v_S4 = v_E+cross(omega_EF,r_S4_E);
v_S5 = cross(omega_GFH,r_S5_G);

%% Mass-center accelerations
a_S1 = cross(alpha_AB,r_S1_A)+cross(omega_AB,cross(omega_AB,r_S1_A));
a_S2 = a_B+cross(alpha_BC,r_S2_B)+cross(omega_BC,cross(omega_BC,r_S2_B));
a_S3 = cross(alpha_DCE,r_S3_D)+cross(omega_DCE,cross(omega_DCE,r_S3_D));
a_S4 = a_E+cross(alpha_EF,r_S4_E)+cross(omega_EF,cross(omega_EF,r_S4_E));
a_S5 = cross(alpha_GFH,r_S5_G)+cross(omega_GFH,cross(omega_GFH,r_S5_G));

%% Newton's second law
% Point-mass artifact force on link GFH.
Q_dynamic = m_payload*([0 -g 0]-a_H);
% Link AB
eqn15 = F_A+F_B+W_AB == m_AB*a_S1;
eqn16 = cross(A-S1,F_A)+cross(B-S1,F_B)+T == I_S1_z*alpha_AB;
% Link BC
eqn17 = -F_B+F_C+W_BC == m_BC*a_S2;
eqn18 = cross(B-S2,-F_B)+cross(C-S2,F_C) == I_S2_z*alpha_BC;
% Link DCE
eqn19 = F_D-F_C+F_E+W_DCE == m_DCE*a_S3;
eqn20 = cross(D-S3,F_D)+cross(C-S3,-F_C)+cross(E-S3,F_E) == I_S3_z*alpha_DCE;
% Link EF
eqn21 = -F_E+F_F+W_EF == m_EF*a_S4;
eqn22 = cross(E-S4,-F_E)+cross(F-S4,F_F) == I_S4_z*alpha_EF;
% Link GFH
eqn23 = F_G-F_F+W_GFH+Q_dynamic == m_GFH*a_S5;
eqn24 = cross(G-S5,F_G)+cross(F-S5,-F_F)+cross(H-S5,Q_dynamic) == I_S5_z*alpha_GFH;

NeqnMatrix = [eqn15(1:2),eqn16(3),eqn17(1:2),eqn18(3),eqn19(1:2),eqn20(3), ...
    eqn21(1:2),eqn22(3),eqn23(1:2),eqn24(3)];
DynamicSolution = solve(NeqnMatrix,force_variables);
dynamic_initial = solution_vector(DynamicSolution,force_variables);
DynamicForces = named_forces(dynamic_initial);

%% First-position data
point_names = {'A','B','C','D','E','F','G','H'};
link_names = {'AB','BC','DCE','EF','GFH'};
FirstPosition.Joints = [A;B;C;D;E;F;G;H];
FirstPosition.COM = [S1;S2;S3;S4;S5];
FirstPosition.omega = [omega_AB;omega_BC;omega_DCE;omega_EF;omega_GFH];
FirstPosition.alpha = [alpha_AB;alpha_BC;alpha_DCE;alpha_EF;alpha_GFH];
FirstPosition.v = [v_A;v_B;v_C;v_D;v_E;v_F;v_G;v_H];
FirstPosition.a = [a_A;a_B;a_C;a_D;a_E;a_F;a_G;a_H];
FirstPosition.v_COM = [v_S1;v_S2;v_S3;v_S4;v_S5];
FirstPosition.a_COM = [a_S1;a_S2;a_S3;a_S4;a_S5];
FirstPosition.StaticForces = StaticForces;
FirstPosition.DynamicForces = DynamicForces;

%% Position analysis over one cycle
P.link_names = link_names;
P.mass = [m_AB m_BC m_DCE m_EF m_GFH];
P.I_centroid_z = [I_S1_z I_S2_z I_S3_z I_S4_z I_S5_z];
P.g_vector = [0;-g;0]; P.k_hat = [0;0;1];
P.omega_AB = omega_AB'; P.alpha_AB = alpha_AB';
P.Q_static = Q_static'; P.payload_mass = m_payload;
for j=1:8, ref.(['r_' point_names{j}])=FirstPosition.Joints(j,:)'; end
for j=1:5, ref.(sprintf('r_S%d',j))=FirstPosition.COM(j,:)'; end
initial_theta = atan2(B(2)-A(2),B(1)-A(1));
initial_DCE = atan2(C(2)-D(2),C(1)-D(1));
angle_deg = linspace(0,360,ceil(360/step_deg)+1)';
time = deg2rad(angle_deg)/omega_AB(3);
N = numel(time);
last_C = C; last_F = F;
static_history = zeros(N,15);
dynamic_history = zeros(N,15);
Q_history = zeros(N,3);
loop_error = zeros(N,6);
force_error = zeros(N,2);
power_error = zeros(N,2);

for n=1:N
    theta = initial_theta+deg2rad(angle_deg(n));
    B_new = A+[lAB*cos(theta) lAB*sin(theta) 0];
    C_new = circle_intersection(B_new',lBC,D',lDC,last_C')';
    delta_DCE = atan2(C_new(2)-D(2),C_new(1)-D(1))-initial_DCE;
    E_new = D+(rotate_z(delta_DCE)*(E-D)')';
    F_new = circle_intersection(E_new',lEF,G',lGF,last_F')';
    H_new = F_new+lFH*(F_new-G)/lGF;
    last_C = C_new; last_F = F_new;
    s = ref;
    s.r_B = B_new'; s.r_C = C_new'; s.r_E = E_new';
    s.r_F = F_new'; s.r_H = H_new';

    % Recalculate all motion and mass centers.
    [s,checks] = kinematics_at_position(s,ref,P);
    % Recalculate static and dynamic forces.
    [M,rhs,Q_cases] = force_system(s,P);
    assert(rcond(M)>1e-12,'Singular force matrix.');
    X = M\rhs(:,1:2);
    static_history(n,:) = X(:,1)';
    dynamic_history(n,:) = X(:,2)';
    Q_history(n,:) = Q_cases(:,2)';
    if n==1
        motion_names = fieldnames(s);
        for j=1:numel(motion_names), Motion.(motion_names{j})=zeros(N,3); end
        assert(norm(X(:,1)-static_initial,inf)<1e-8);
        assert(norm(X(:,2)-dynamic_initial,inf)<1e-8);
        for j=1:5
            assert(norm(s.(['omega_' link_names{j}])'-FirstPosition.omega(j,:))<1e-10);
            assert(norm(s.(['alpha_' link_names{j}])'-FirstPosition.alpha(j,:))<1e-10);
            assert(norm(s.(sprintf('a_S%d',j))'-FirstPosition.a_COM(j,:))<1e-10);
        end
    end
    for j=1:numel(motion_names), Motion.(motion_names{j})(n,:)=s.(motion_names{j})'; end
    loop_error(n,:) = checks.residual;
    force_error(n,:) = max(abs(M*X-rhs(:,1:2)),[],1);
    for c=1:2
        power = X(15,c)*omega_AB(3)+dot(Q_cases(:,c),s.v_H);
        for j=1:5
            vs = s.(sprintf('v_S%d',j)); as = s.(sprintf('a_S%d',j));
            power = power+dot(P.mass(j)*P.g_vector,vs);
            if c==2
                power = power-P.mass(j)*dot(as,vs) ...
                    -P.I_centroid_z(j)*dot(s.(['alpha_' link_names{j}]),s.(['omega_' link_names{j}]));
            end
        end
        power_error(n,c) = power;
    end
end

%% Verification
Diagnostics.max_position_residual_m = max(loop_error(:,1:2),[],'all');
Diagnostics.max_velocity_residual_m_s = max(loop_error(:,3:4),[],'all');
Diagnostics.max_acceleration_residual_m_s2 = max(loop_error(:,5:6),[],'all');
Diagnostics.max_force_residual = max(force_error,[],'all');
Diagnostics.max_power_residual_W = max(abs(power_error),[],'all');
FDinput = struct('time_s',time,'motion',Motion,'parameters',P);
Diagnostics.finite_difference = finite_difference_check(FDinput);
assert(max(loop_error,[],'all')<1e-9);
assert(Diagnostics.max_force_residual<1e-7);
assert(Diagnostics.max_power_residual_W<1e-6);

%% First-position tables
Tables = first_position_tables(FirstPosition,static_initial,dynamic_initial,output_folder);
disp('Input speed (rpm), angular velocity (rad/s), cycle time (s):');
disp([n_AB omega_AB(3) T_cycle]);
disp('Static equilibrium:'); disp(Tables.Static);
disp('Newton second law:'); disp(Tables.Dynamic);
disp('Angular velocities:'); disp(Tables.AngularVelocity);
disp('Angular accelerations:'); disp(Tables.AngularAcceleration);
disp('Joint velocities:'); disp(Tables.JointVelocity);
disp('Joint accelerations:'); disp(Tables.JointAcceleration);
disp('Mass-center accelerations:'); disp(Tables.COMAcceleration);

%% Full-cycle exports and figures
export_cycle(Motion,time,angle_deg,static_history,dynamic_history,Q_history,output_folder);
plot_classroom(Motion,angle_deg,static_history,dynamic_history,show_plots,output_folder);
save(fullfile(output_folder,'classroom_results.mat'),'FirstPosition','Tables','Motion', ...
    'static_history','dynamic_history','Q_history','time','angle_deg','P', ...
    'Diagnostics','material','rho','T_cycle');
disp('Validation:'); disp(Diagnostics);
fprintf('Results saved to %s\n',output_folder);

%% Local functions
function Tables=first_position_tables(first,xstatic,xdynamic,out)
quantity=strings(22,1); unit=repmat("N",22,1);
stat=zeros(22,1); dyn=stat;
for j=1:7
    nm=char('A'+j-1); rows=3*j-2:3*j; cols=2*j-1:2*j;
    quantity(rows)=["F_"+nm+"x";"F_"+nm+"y";"|F_"+nm+"|"];
    stat(rows)=[xstatic(cols);norm(xstatic(cols))];
    dyn(rows)=[xdynamic(cols);norm(xdynamic(cols))];
end
quantity(end)="T_z"; unit(end)="N m"; stat(end)=xstatic(15); dyn(end)=xdynamic(15);
Tables.Static=table(quantity,unit,stat,'VariableNames',{'Quantity','Unit','MATLAB'});
Tables.Dynamic=table(quantity,unit,dyn,'VariableNames',{'Quantity','Unit','MATLAB'});
links={'AB','BC','DCE','EF','GFH'};
Tables.AngularVelocity=angular_table(links,first.omega,'omega_',"rad/s");
Tables.AngularAcceleration=angular_table(links,first.alpha,'alpha_',"rad/s^2");
points={'A','B','C','D','E','F','G','H'};
Tables.JointVelocity=linear_table(points,first.v,'v_',"m/s");
Tables.JointAcceleration=linear_table(points,first.a,'a_',"m/s^2");
Tables.COMAcceleration=linear_table({'S1','S2','S3','S4','S5'},first.a_COM,'a_',"m/s^2");
names=fieldnames(Tables);
for j=1:numel(names), writetable(Tables.(names{j}),fullfile(out,['FirstPosition_' names{j} '.csv'])); end
end

function T=angular_table(names,mat,prefix,unit)
N=numel(names); q=strings(N,1);
for j=1:N, q(j)=string([prefix names{j}])+"_z"; end
T=table(q,repmat(unit,N,1),mat(:,3), ...
    'VariableNames',{'Quantity','Unit','MATLAB'});
end

function T=linear_table(names,mat,prefix,unit)
N=3*numel(names); q=strings(N,1); values=zeros(N,1);
for j=1:numel(names)
    field=[prefix names{j}]; rows=3*j-2:3*j;
    q(rows)=[string(field)+"x";string(field)+"y";"|"+string(field)+"|"];
    values(rows)=[mat(j,1:2),norm(mat(j,:))]';
end
T=table(q,repmat(unit,N,1),values, ...
    'VariableNames',{'Quantity','Unit','MATLAB'});
end

function export_cycle(Motion,time,angles,stat,dyn,Q,out)
T=table(time,angles,'VariableNames',{'Time_s','InputAngle_deg'});
names=fieldnames(Motion); axes='xyz';
for j=1:numel(names)
    name=names{j};
    if startsWith(name,'omega_')
        unit='_rad_s';
    elseif startsWith(name,'alpha_')
        unit='_rad_s2';
    elseif startsWith(name,'v_')
        unit='_m_s';
    elseif startsWith(name,'a_')
        unit='_m_s2';
    else
        unit='_m';
    end
    for k=1:3, T.([name '_' axes(k) unit])=Motion.(name)(:,k); end
end
writetable(T,fullfile(out,'FullCycle_Kinematics.csv'));
labels={'FAx_N','FAy_N','FBx_N','FBy_N','FCx_N','FCy_N','FDx_N','FDy_N', ...
    'FEx_N','FEy_N','FFx_N','FFy_N','FGx_N','FGy_N','T_z_Nm'};
writetable(array2table([time,angles,stat],'VariableNames',[{'Time_s','InputAngle_deg'},labels]), ...
    fullfile(out,'FullCycle_Static.csv'));
writetable(array2table([time,angles,dyn,Q(:,1:2)], ...
    'VariableNames',[{'Time_s','InputAngle_deg'},labels,{'Qx_N','Qy_N'}]), ...
    fullfile(out,'FullCycle_Newton.csv'));
end

function plot_classroom(Motion,angles,stat,dyn,show,out)
if show, visibility='on'; else, visibility='off'; end
links={'AB','BC','DCE','EF','GFH'};
points={'B','C','E','F','H'};

% Initial outline and point paths.
f=figure('Name','Geometry and paths','NumberTitle','off','Visible',visibility,'Color','w');
ax=axes(f); hold(ax,'on');
chains={{'A','B'},{'B','C'},{'D','C','E'},{'E','F'},{'G','F','H'}};
for j=1:5
    coords=zeros(numel(chains{j}),2);
    for k=1:numel(chains{j}), v=Motion.(['r_' chains{j}{k}]); coords(k,:)=v(1,1:2); end
    plot(ax,coords(:,1),coords(:,2),'-o','LineWidth',1.5,'DisplayName',links{j});
end
for j=1:numel(points)
    pos=Motion.(['r_' points{j}]);
    plot(ax,pos(:,1),pos(:,2),':','DisplayName',[points{j} ' path']);
end
for nm='ABCDEFGH', v=Motion.(['r_' nm]); text(ax,v(1,1)+.04,v(1,2),nm); end
axis(ax,'equal'); grid(ax,'on'); xlabel(ax,'x (m)'); ylabel(ax,'y (m)'); legend(ax,'Location','eastoutside');
exportgraphics(f,fullfile(out,'01_geometry_paths.png'),'Resolution',160);

% All angular quantities.
f=figure('Name','Link angular motion','NumberTitle','off','Visible',visibility,'Color','w');
layout=tiledlayout(f,2,1);
for k=1:2
    if k==1, prefix='omega_'; label='Angular velocity (rad/s)';
    else, prefix='alpha_'; label='Angular acceleration (rad/s^2)'; end
    ax=nexttile(layout); hold(ax,'on');
    for j=1:5, v=Motion.([prefix links{j}]); plot(ax,angles,v(:,3),'LineWidth',1.2,'DisplayName',links{j}); end
    grid(ax,'on'); xlabel(ax,'Input rotation (deg)'); ylabel(ax,label); legend(ax,'Location','eastoutside');
    xlim(ax,[0 360]);
end
exportgraphics(f,fullfile(out,'02_angular_motion.png'),'Resolution',160);

% Joint velocities and accelerations.
f=figure('Name','Joint linear motion','NumberTitle','off','Visible',visibility,'Color','w');
layout=tiledlayout(f,2,2);
for k=1:4
    if k<=2, prefix='v_'; label='Velocity'; unit='m/s'; else, prefix='a_'; label='Acceleration'; unit='m/s^2'; end
    component=1+mod(k-1,2); ax=nexttile(layout); hold(ax,'on');
    for j=1:5, v=Motion.([prefix points{j}]); plot(ax,angles,v(:,component),'LineWidth',1.2,'DisplayName',points{j}); end
    grid(ax,'on'); xlabel(ax,'Input rotation (deg)'); xlim(ax,[0 360]);
    ylabel(ax,[label ' ' char('x'+component-1) ' (' unit ')']); legend(ax,'Location','best');
end
exportgraphics(f,fullfile(out,'03_joint_motion.png'),'Resolution',160);

% Mass-center accelerations.
f=figure('Name','Mass-center accelerations','NumberTitle','off','Visible',visibility,'Color','w');
layout=tiledlayout(f,2,1);
for k=1:2
    ax=nexttile(layout); hold(ax,'on');
    for j=1:5, v=Motion.(sprintf('a_S%d',j)); plot(ax,angles,v(:,k),'LineWidth',1.2,'DisplayName',sprintf('S%d',j)); end
    grid(ax,'on'); xlabel(ax,'Input rotation (deg)'); xlim(ax,[0 360]);
    ylabel(ax,['Acceleration ' char('x'+k-1) ' (m/s^2)']); legend(ax,'Location','eastoutside');
end
exportgraphics(f,fullfile(out,'04_COM_accelerations.png'),'Resolution',160);

% Torque and dynamic joint-force magnitudes.
f=figure('Name','Joint forces and input torque','NumberTitle','off','Visible',visibility,'Color','w');
layout=tiledlayout(f,2,1); ax=nexttile(layout);
plot(ax,angles,stat(:,15),angles,dyn(:,15),'LineWidth',1.3); grid(ax,'on');
xlabel(ax,'Input rotation (deg)'); ylabel(ax,'Input torque (N m)'); xlim(ax,[0 360]);
legend(ax,{'Static','Newton second law'},'Location','best');
ax=nexttile(layout); hold(ax,'on');
for j=1:7, plot(ax,angles,hypot(dyn(:,2*j-1),dyn(:,2*j)),'DisplayName',['F_' char('A'+j-1)],'LineWidth',1.1); end
grid(ax,'on'); xlabel(ax,'Input rotation (deg)'); ylabel(ax,'Joint force magnitude (N)');
xlim(ax,[0 360]); legend(ax,'Location','eastoutside');
exportgraphics(f,fullfile(out,'05_forces_torque.png'),'Resolution',160);

end

function x=solution_vector(solution,variables)
x=zeros(numel(variables),1);
for j=1:numel(variables), x(j)=double(solution.(char(variables(j)))); end
end
function [s,checks]=kinematics_at_position(s,ref,P)
s.r_B_A=s.r_B-s.r_A; s.r_C_B=s.r_C-s.r_B; s.r_C_D=s.r_C-s.r_D;
s.r_E_D=s.r_E-s.r_D; s.r_F_E=s.r_F-s.r_E; s.r_F_G=s.r_F-s.r_G;
s.r_H_G=s.r_H-s.r_G; s.r_D_A=s.r_D-s.r_A; s.r_G_D=s.r_G-s.r_D;
k_hat=P.k_hat;
K1=[cross(k_hat,s.r_C_B),-cross(k_hat,s.r_C_D)]; K1=K1(1:2,:);
K2=[cross(k_hat,s.r_F_E),-cross(k_hat,s.r_F_G)]; K2=K2(1:2,:);
checks.rcond=[rcond(K1),rcond(K2)];
assert(min(checks.rcond)>1e-12,'Singular velocity/acceleration loop matrix.');

s.omega_AB=P.omega_AB; s.alpha_AB=P.alpha_AB;
rhs=-cross(s.omega_AB,s.r_B_A); solved=K1\rhs(1:2);
s.omega_BC=solved(1)*k_hat; s.omega_DCE=solved(2)*k_hat;
rhs=-cross(s.omega_DCE,s.r_E_D); solved=K2\rhs(1:2);
s.omega_EF=solved(1)*k_hat; s.omega_GFH=solved(2)*k_hat;

rhs=-cross(s.alpha_AB,s.r_B_A)-centripetal(s.omega_AB,s.r_B_A) ...
    -centripetal(s.omega_BC,s.r_C_B)+centripetal(s.omega_DCE,s.r_C_D);
solved=K1\rhs(1:2); s.alpha_BC=solved(1)*k_hat; s.alpha_DCE=solved(2)*k_hat;
rhs=-cross(s.alpha_DCE,s.r_E_D)-centripetal(s.omega_DCE,s.r_E_D) ...
    -centripetal(s.omega_EF,s.r_F_E)+centripetal(s.omega_GFH,s.r_F_G);
solved=K2\rhs(1:2); s.alpha_EF=solved(1)*k_hat; s.alpha_GFH=solved(2)*k_hat;

s.v_A=zeros(3,1); s.v_D=zeros(3,1); s.v_G=zeros(3,1);
s.v_B=cross(s.omega_AB,s.r_B_A);
s.v_C=cross(s.omega_DCE,s.r_C_D);
s.v_E=cross(s.omega_DCE,s.r_E_D);
s.v_F=cross(s.omega_GFH,s.r_F_G);
s.v_H=cross(s.omega_GFH,s.r_H_G);
v_C_via_B=s.v_B+cross(s.omega_BC,s.r_C_B);
v_F_via_E=s.v_E+cross(s.omega_EF,s.r_F_E);

s.a_A=zeros(3,1); s.a_D=zeros(3,1); s.a_G=zeros(3,1);
s.a_B=cross(s.alpha_AB,s.r_B_A)+centripetal(s.omega_AB,s.r_B_A);
s.a_C=cross(s.alpha_DCE,s.r_C_D)+centripetal(s.omega_DCE,s.r_C_D);
s.a_E=cross(s.alpha_DCE,s.r_E_D)+centripetal(s.omega_DCE,s.r_E_D);
s.a_F=cross(s.alpha_GFH,s.r_F_G)+centripetal(s.omega_GFH,s.r_F_G);
s.a_H=cross(s.alpha_GFH,s.r_H_G)+centripetal(s.omega_GFH,s.r_H_G);
a_C_via_B=s.a_B+cross(s.alpha_BC,s.r_C_B)+centripetal(s.omega_BC,s.r_C_B);
a_F_via_E=s.a_E+cross(s.alpha_EF,s.r_F_E)+centripetal(s.omega_EF,s.r_F_E);

anchor={'A','B','D','E','G'}; axis_end={'B','C','E','F','H'};
for j=1:5
    J=anchor{j}; L=P.link_names{j}; nm=sprintf('S%d',j);
    rotation=angle_xy(s.(['r_' axis_end{j}])-s.(['r_' J])) ...
        -angle_xy(ref.(['r_' axis_end{j}])-ref.(['r_' J]));
    r_S_J=rotate_z(rotation)*(ref.(['r_' nm])-ref.(['r_' J]));
    s.(['r_' nm])=s.(['r_' J])+r_S_J;
    s.(['r_' nm '_' J])=r_S_J;
    s.(['v_' nm])=s.(['v_' J])+cross(s.(['omega_' L]),r_S_J);
    s.(['a_' nm])=s.(['a_' J])+cross(s.(['alpha_' L]),r_S_J) ...
        +centripetal(s.(['omega_' L]),r_S_J);
end
checks.residual=[norm(s.r_B_A+s.r_C_B-s.r_C_D-s.r_D_A), ...
    norm(s.r_E_D+s.r_F_E-s.r_F_G-s.r_G_D), ...
    norm(s.v_C-v_C_via_B),norm(s.v_F-v_F_via_E), ...
    norm(s.a_C-a_C_via_B),norm(s.a_F-a_F_via_E)];
end

function a=centripetal(omega,r)
a=cross(omega,cross(omega,r));
end
function angle=angle_xy(r)
angle=atan2(r(2),r(1));
end
function Rz=rotate_z(angle)
Rz=[cos(angle),-sin(angle),0;sin(angle),cos(angle),0;0,0,1];
end
function p=circle_intersection(c1,r1,c2,r2,previous)
d=norm(c2-c1);
assert(d>1e-12 && d<=r1+r2+1e-10 && d>=abs(r1-r2)-1e-10,'No real circle intersection.');
u=(c2-c1)/d; q=(r1^2-r2^2+d^2)/(2*d); h2=r1^2-q^2;
assert(h2>=-1e-10,'Invalid circle intersection.');
mid=c1+q*u; offset=sqrt(max(0,h2))*cross([0;0;1],u);
p1=mid+offset; p2=mid-offset;
if norm(p1-previous)<=norm(p2-previous), p=p1; else, p=p2; end
end

function [M,b,Q]=force_system(s,P)
M=zeros(15); b=zeros(15,2);
connections={{'A',1;'B',1},{'B',-1;'C',1}, ...
    {'D',1;'C',-1;'E',1},{'E',-1;'F',1},{'G',1;'F',-1}};
Q=[P.Q_static,P.payload_mass*(P.g_vector-s.a_H)];
for j=1:5
    rows=3*j-2:3*j; S=s.(sprintf('r_S%d',j)); data=connections{j};
    for k=1:size(data,1)
        nm=data{k,1}; sign_force=data{k,2}; col=2*(double(nm)-double('A'))+1;
        arm=s.(['r_' nm])-S;
        M(rows(1),col)=sign_force; M(rows(2),col+1)=sign_force;
        M(rows(3),col)=-sign_force*arm(2); M(rows(3),col+1)=sign_force*arm(1);
    end
    if j==1, M(rows(3),15)=1; end
    W=P.mass(j)*P.g_vector;
    for c=1:2
        inertial_force=zeros(3,1); inertial_moment=zeros(3,1);
        if c==2
            inertial_force=P.mass(j)*s.(sprintf('a_S%d',j));
            inertial_moment=P.I_centroid_z(j)*s.(['alpha_' P.link_names{j}]);
        end
        external_force=W; external_moment=zeros(3,1);
        if j==5
            external_force=external_force+Q(:,c);
            external_moment=cross(s.r_H-S,Q(:,c));
        end
        rhs_force=inertial_force-external_force;
        rhs_moment=inertial_moment-external_moment;
        b(rows,c)=[rhs_force(1:2);rhs_moment(3)];
    end
end
end

function f=named_forces(x)
for j=1:7, f.(['F_' char('A'+j-1)])=[x(2*j-1:2*j);0]; end
f.T=[0;0;x(15)];
end

function fd=finite_difference_check(R)
dt=R.time_s(2)-R.time_s(1); N=numel(R.time_s)-1;
points={'A','B','C','D','E','F','G','H','S1','S2','S3','S4','S5'};
for j=1:numel(points)
    nm=points{j}; pos=R.motion.(['r_' nm])(1:N,:);
    vfd=(circshift(pos,-1,1)-circshift(pos,1,1))/(2*dt);
    afd=(circshift(pos,-1,1)-2*pos+circshift(pos,1,1))/dt^2;
    fd.(nm).velocity_error_m_s=max(abs(vfd-R.motion.(['v_' nm])(1:N,:)),[],'all');
    fd.(nm).acceleration_error_m_s2=max(abs(afd-R.motion.(['a_' nm])(1:N,:)),[],'all');
end
relative={'r_B_A','r_C_B','r_E_D','r_F_E','r_H_G'};
for j=1:5
    nm=R.parameters.link_names{j}; r=R.motion.(relative{j})(1:N,:);
    angle=atan2(r(:,2),r(:,1)); df=circshift(angle,-1)-angle; db=angle-circshift(angle,1);
    df=atan2(sin(df),cos(df)); db=atan2(sin(db),cos(db));
    fd.(nm).omega_error_rad_s=max(abs((df+db)/(2*dt)-R.motion.(['omega_' nm])(1:N,3)));
    fd.(nm).alpha_error_rad_s2=max(abs((df-db)/dt^2-R.motion.(['alpha_' nm])(1:N,3)));
end
end
