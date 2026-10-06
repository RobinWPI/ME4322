function r = lab3_full_parameter_analysis(p,model,outputDir)
% Component parameters and damping sensitivity.

a = model.a; b = model.b; gamma = model.gamma;
Jeq = model.Jeq; Keq = model.Keq;
c = model.photo.staticCalibration;
Krange = gamma*[c.KeqOverGammaLow_N_per_rad c.KeqOverGammaHigh_N_per_rad];
calibratedK = gamma*c.KeqOverGammaMidpoint_N_per_rad;
assert(abs(Keq-calibratedK) <= 1e-12*calibratedK, ...
    'Lab3:StiffnessCheck','Ten-step stiffness must match static calibration.');
assert(abs(2*p.mLongLever+p.mB+p.mSpringRemainder-p.mAssembly) < 1e-12, ...
    'Lab3:MassAllocation','The assumed assembly mass allocation must close.');

%% Stiffness reduction
k = zeros(10,1);
k(1) = (p.K/2)*p.L1^2;
k(2) = (p.K/2)*p.L4^2;
k(3) = k(2)/p.L3^2;
k(4) = p.L5^2*k(3);
k(5) = (k(1)+k(4))/p.L2^2;
k(6) = 2*k(5);
k(7) = k(6)+p.K3;
k(8) = k(7)*p.L6^2;
k(9) = k(8)/p.L7^2+p.K4;
k(10) = k(9)*p.Rp^2;
r.stiffnessSteps = table((1:10)',k, ...
    ["N*m/rad";"N*m/rad";"N/m";"N*m/rad";"N/m";"N/m";"N/m";"N*m/rad";"N/m";"N*m/rad"], ...
    'VariableNames',{'Step','EquivalentStiffness','Unit'});

%% Inertia at the dial
terms = [p.JD;p.Jp;p.mU*gamma^2;p.mB*(b*p.Rp)^2; ...
    (p.JL1+p.JL2)/p.L2^2*(b*p.Rp)^2; ...
    (p.JS1+p.JS2)*(p.L5/(p.L2*p.L3))^2*(b*p.Rp)^2; ...
    p.mr*p.Rp^2;p.JR*(p.Rp/p.L7)^2];
assert(abs(sum(terms)-Jeq) <= 1e-12*Jeq,'Lab3:InertiaCheck','Inertia contributions must sum to Jeq.');
r.inertiaTerms = table(["Dial";"Pinion";"Platform";"Collector"; ...
    "Long levers";"Short levers";"Rack";"Lever 5"],terms,100*terms/Jeq, ...
    'VariableNames',{'Contribution','Inertia_kg_m2','Share_pct'});

%% Assumed damping cases
wn = sqrt(Keq/Jeq);
thetaSS = gamma*p.F0/Keq;
zeta = p.zetaCases;
Deq = 2*zeta*sqrt(Jeq*Keq);
t = linspace(0,30/wn,3001)'; tau = wn*t;
angles = zeros(numel(t),numel(zeta));
speeds = angles;
maxError = zeros(numel(zeta),1); overshoot = maxError;
options = odeset('RelTol',1e-9,'AbsTol',1e-11);
for j = 1:numel(zeta)
    [~,x] = ode45(@(t,x) [x(2); ...
        (gamma*p.F0-Deq(j)*x(2)-Keq*x(1))/Jeq],t,[0;0],options);
    angles(:,j) = x(:,1); speeds(:,j) = x(:,2);
    z = zeta(j);
    if z < 1
        wd = sqrt(1-z^2);
        exact = 1-exp(-z*tau).*(cos(wd*tau)+z/wd*sin(wd*tau));
        overshoot(j) = 100*exp(-pi*z/wd);
    elseif z == 1
        exact = 1-exp(-tau).*(1+tau);
    else
        r1 = -z+sqrt(z^2-1); r2 = -z-sqrt(z^2-1);
        exact = 1+(r2*exp(r1*tau)-r1*exp(r2*tau))/(r1-r2);
    end
    maxError(j) = max(abs(angles(:,j)-thetaSS*exact));
end
scaledError = maxError/thetaSS;
assert(all(scaledError < 1e-7),'Lab3:FullODE','Physical-time ODE verification failed.');
r.response = table();
for j = 1:numel(zeta)
    n = numel(t); q = p.Rp*angles(:,j); sB = -b*q; sU = a*sB;
    rows = table(t,repmat(zeta(j),n,1),repmat(Deq(j),n,1),angles(:,j), ...
        rad2deg(angles(:,j)),speeds(:,j),q,sB,sU,q/p.L7, ...
        'VariableNames',{'Time_s','AssumedZeta','Deq_N_m_s_per_rad', ...
        'Theta_rad','Theta_deg','Omega_rad_s','Rack_m','CollectorUp_m','PlatformUp_m','Lever5_rad'});
    r.response = [r.response;rows];
end
r.validation = table(zeta',Deq',overshoot,maxError,scaledError, ...
    'VariableNames',{'AssumedZeta','Deq_N_m_s_per_rad', ...
    'AnalyticalOvershoot_pct','MaxODEError_rad','MaxNormalizedODEError'});

%% Parameters
name = ["L1";"L2";"L3";"L4";"L5";"L6";"L7";"Pinion outside diameter"; ...
    "Pinion teeth";"Rp";"a";"b";"gamma";"K1=K2=K";"K3";"K4"; ...
    "mU";"mD";"mS_total";"mAssembly";"mB";"mr";"mLongLever each"; ...
    "Spring/fittings remainder";"JL1";"JL2";"JS1";"JS2";"JR";"Jp";"JD"; ...
    "Jeq";"Keq lower";"Keq midpoint";"Keq upper";"Default zeta";"DB"; ...
    "Default Deq";"omega_n midpoint";"f_n midpoint";"F0";"theta_ss midpoint"];
value = [p.L1;p.L2;p.L3;p.L4;p.L5;p.L6;p.L7;p.pinionOutsideDiameter; ...
    p.pinionTeeth;p.Rp;a;b;gamma;p.K;p.K3;p.K4;p.mU;p.mD;p.mS_total; ...
    p.mAssembly;p.mB;p.mr;p.mLongLever;p.mSpringRemainder; ...
    p.JL1;p.JL2;p.JS1;p.JS2;p.JR;p.Jp;p.JD;Jeq;Krange(1);Keq;Krange(2); ...
    p.assumedDampingRatio;p.DB;model.Deq;wn;wn/(2*pi);p.F0;thetaSS];
unit = [repmat("m",8,1);"1";"m";"1";"1";"m";repmat("N/m",3,1); ...
    repmat("kg",8,1);repmat("kg*m^2",8,1);repmat("N*m/rad",3,1); ...
    "1";"N*s/m";"N*m*s/rad";"rad/s";"Hz";"N";"rad"];
basis = [repmat("User supplied",9,1);"Standard unshifted full-depth teeth"; ...
    repmat("Geometry",3,1);"Assumed";"Calibrated with assumed K and K4";"Assumed"; ...
    repmat("Balance display",4,1);repmat("Assumed",3,1);"Assumed assembly allocation; omitted inertia"; ...
    repmat("Assumed mass; uniform end-pivot rod",2,1); ...
    repmat("Measured pair mass; equal uniform end-pivot rods",2,1); ...
    "Assumed";"Assumed";"Measured mass; estimated diameter; uniform disk"; ...
    "Full component model";repmat("Baseline-corrected static fit",3,1); ...
    "Assumed";"Derived from assumed zeta";"Derived from assumed zeta"; ...
    repmat("Full inertia and midpoint stiffness",2,1);"Apple nominal mass times g";"Static midpoint"];
r.parameters = table(name,value,unit,basis,'VariableNames',{'Parameter','Value_SI','Unit','Basis'});
r.KeqRange = Krange;
fprintf('K = %.6g; K3 = %.6g; K4 = %.6g N/m.\n',p.K,p.K3,p.K4);
fprintf('Physical ODE max error = %.3g rad.\n',max(maxError));
if strlength(string(outputDir)) == 0, return; end
if ~isfolder(outputDir), mkdir(outputDir); end
writetable(r.parameters,fullfile(outputDir,'full_parameters.csv'));
writetable(r.inertiaTerms,fullfile(outputDir,'full_inertia.csv'));
writetable(r.stiffnessSteps,fullfile(outputDir,'full_stiffness_steps.csv'));
writetable(r.response,fullfile(outputDir,'physical_time_response.csv'));
writetable(r.validation,fullfile(outputDir,'physical_time_validation.csv'));
f = figure('Color','w','Name','Full-model damping sensitivity', ...
    'Units','inches','Position',[1 1 3.35 2.65]);
styles = {'-','--','-.',':'};
for j = 1:numel(zeta)
    plot(t,rad2deg(angles(:,j)),styles{1+mod(j-1,numel(styles))}, ...
        'LineWidth',1.15); hold on;
end
yline(rad2deg(thetaSS),'k:','HandleVisibility','off');
xlabel('Time (s)'); ylabel('Incremental dial rotation (deg)');
title('Predicted damping sensitivity');
legend(compose('\\zeta = %g',zeta),'Location','northeast','FontSize',7);
grid on; xlim([0 max(t)]); ylim([0 1.65*rad2deg(thetaSS)]);
set(gca,'FontName','Times New Roman','FontSize',8);
exportgraphics(f,fullfile(outputDir,'Fig18_Physical_Response.png'),'Resolution',400);
savefig(f,fullfile(outputDir,'Fig18_Physical_Response.fig'));
end
