function Report=test_homework1_classroom
folder=fileparts(mfilename('fullpath'));
out=fullfile(folder,'results_matlab');
file=fullfile(out,'classroom_results.mat');
assert(isfile(file),'Run main first.');
S=load(file);
assert(numel(S.time)==721);
assert(abs(S.time(end)-2.592)<1e-12);
assert(abs(S.T_cycle-2.592)<1e-12);
assert(all(S.Motion.alpha_AB==0,'all'));
assert(max(abs(S.Motion.omega_AB(:,3)-2*pi/2.592))<1e-12);
for nm={'A','D','G'}
    assert(all(S.Motion.(['v_' nm{1}])==0,'all'));
    assert(all(S.Motion.(['a_' nm{1}])==0,'all'));
end
assert(S.Diagnostics.max_position_residual_m<1e-10);
assert(S.Diagnostics.max_velocity_residual_m_s<1e-10);
assert(S.Diagnostics.max_acceleration_residual_m_s2<1e-10);
assert(S.Diagnostics.max_force_residual<1e-7);
assert(S.Diagnostics.max_power_residual_W<1e-6);
assert(S.Diagnostics.max_external_force_error_N==0);

% Both load cases use the same prescribed downward force at H.
expected_Q=[0 -200 0];
assert(isequal(S.P.Q_static,expected_Q'));
assert(isequal(S.P.Q_dynamic,expected_Q'));
assert(strcmp(S.P.load_model,'prescribed_force_at_H'));
assert(~isfield(S.P,'payload_mass'));
assert(isequal(S.FirstPosition.Q_static,expected_Q));
assert(isequal(S.FirstPosition.Q_dynamic,expected_Q));
assert(isequal(S.Q_history,repmat(expected_Q,numel(S.time),1)));
assert(abs(S.static_history(1,15)-(-504.372529988217))<1e-7);
assert(abs(S.dynamic_history(1,15)-(-47.1749048698292))<1e-7);

% Independently recover both torques from saved motion and energy rates.
external_power=S.Motion.v_H*expected_Q';
kinetic_rate=zeros(numel(S.time),1);
for j=1:numel(S.P.mass)
    velocity=S.Motion.(sprintf('v_S%d',j));
    acceleration=S.Motion.(sprintf('a_S%d',j));
    omega=S.Motion.(['omega_' S.P.link_names{j}]);
    alpha=S.Motion.(['alpha_' S.P.link_names{j}]);
    external_power=external_power+S.P.mass(j)*(velocity*S.P.g_vector);
    kinetic_rate=kinetic_rate+S.P.mass(j)*sum(velocity.*acceleration,2) ...
        +S.P.I_centroid_z(j)*sum(omega.*alpha,2);
end
input_speed=S.Motion.omega_AB(:,3);
static_power_torque=-external_power./input_speed;
dynamic_power_torque=(kinetic_rate-external_power)./input_speed;
torque_error=max(abs([static_power_torque-S.static_history(:,15), ...
    dynamic_power_torque-S.dynamic_history(:,15)]),[],'all');
assert(torque_error<1e-7,'Independent energy-rate torque check failed.');

% Main results contain MATLAB values only.
assert(~isfield(S,'PMKS') && ~isfield(S,'PMKSComparison'));
table_names={'Static','Dynamic','AngularVelocity','AngularAcceleration', ...
    'JointVelocity','JointAcceleration','COMAcceleration'};
row_counts=[22 22 5 5 24 24 15];
for j=1:numel(table_names)
    T=S.Tables.(table_names{j});
    assert(height(T)==row_counts(j));
    assert(isequal(T.Properties.VariableNames,{'Quantity','Unit','MATLAB'}));
    assert(all(isfinite(T.MATLAB)));
    assert(isfile(fullfile(out,['FirstPosition_' table_names{j} '.csv'])));
end

% Optional regression against the previous mechanics implementation.
baseline=fullfile(folder,'results_6061T6_steady','results.mat');
regression=NaN;
if isfile(baseline)
    old=load(baseline,'R'); old=old.R;
    assert(numel(old.time_s)==numel(S.time));
    regression=0; names=fieldnames(S.Motion);
    for j=1:numel(names)
        regression=max(regression,max(abs(S.Motion.(names{j})-old.motion.(names{j})),[],'all'));
    end
    regression=max(regression,max(abs(S.static_history-old.forces.static_loaded),[],'all'));
    regression=max(regression,max(abs(S.dynamic_history-old.forces.dynamic_force_only),[],'all'));
    assert(regression<1e-7,'Regression mismatch.');
end

source=fileread(fullfile(folder,'main.m'));
assert(all(double(source)<128),'The source must use English ASCII comments.');
assert(~contains(lower(source),'pmks'),'The main script must contain no PMKS logic or data.');
assert(~contains(source,'readmatrix(') && ~contains(source,'readtable('));
figures=dir(fullfile(out,'*.png'));
assert(numel(figures)==5);
expected={'01_geometry_paths.png','02_angular_motion.png','03_joint_motion.png', ...
    '04_COM_accelerations.png','05_forces_torque.png'};
for j=1:numel(expected), assert(isfile(fullfile(out,expected{j}))); end

Report=struct('passed',true,'positions',numel(S.time), ...
    'matlab_only',true,'figure_count',numel(figures), ...
    'prescribed_force_N',expected_Q, ...
    'initial_static_torque_Nm',S.static_history(1,15), ...
    'initial_dynamic_torque_Nm',S.dynamic_history(1,15), ...
    'max_energy_rate_torque_error_Nm',torque_error, ...
    'max_regression_difference',regression,'diagnostics',S.Diagnostics);
fid=fopen(fullfile(out,'test_report.json'),'w');
assert(fid>0); closer=onCleanup(@()fclose(fid));
fprintf(fid,'%s',jsonencode(Report,PrettyPrint=true));
disp(Report);
end
