function r = Lab3_MATLAB(p, outputDir)
% Full model: Jeq*theta_ddot + Deq*theta_dot + Keq*theta = gamma*F.
% theta: incremental dial rotation (rad).

if nargin < 1
    p = lab3_parameters();
end
if nargin < 2
    outputDir = fullfile(fileparts(mfilename('fullpath')), 'results');
end
r = struct('status', "static_calibration_and_normalized", 'parameters', p);
r.photo = lab3_photo_analysis(p, outputDir);
%% Parameters
geometry = {'L1','L2','L3','L4','L5','L6','L7','Rp'};
staticFields = [geometry, {'K','K3','K4','F0'}];
dynamicFields = {'mU','mB','mr','JL1','JL2','JS1','JS2','JR','Jp','JD','DB'};
if reportMissing(p, staticFields)
    fprintf('Physical-time response requires missing inputs.\n');
    if strlength(string(outputDir)) > 0
        save(fullfile(outputDir, 'lab3_results.mat'), 'r');
    end
    return
end
validateFields(p, geometry, true);
validateFields(p, {'K','K3','K4','F0'}, false);

a = p.L1 / p.L2;
aF = p.L4 * p.L5 / (p.L3 * p.L2);
b = p.L6 / p.L7;
assert(abs(a - aF) <= 1e-3 * max(a, aF), ...
    'Lab3:Geometry', 'Rigid-platform geometry requires L1*L3 = L4*L5.');
assert(p.L1 <= p.L5 && p.L5 <= p.L2 && p.L4 <= p.L3, ...
    'Lab3:Geometry', 'Check the A-E-C-B and D-F-C point order.');

%% Equivalent stiffness
Keq1R = (p.K / 2) * p.L1^2;
Keq2R = (p.K / 2) * p.L4^2;
Keq3T = Keq2R / p.L3^2;
Keq4R = p.L5^2 * Keq3T;
Keq5T = (Keq1R + Keq4R) / p.L2^2;
Keq6T = 2 * Keq5T;
Keq7T = Keq6T + p.K3;
Keq8R = Keq7T * p.L6^2;
Keq9T = Keq8R / p.L7^2 + p.K4;
Keq = Keq9T * p.Rp^2;
gamma = a * b * p.Rp;
assert(isfinite(Keq) && Keq > 0, 'Lab3:Stiffness', 'Keq must be positive.');
r.a = a;
r.b = b;
r.gamma = gamma;
r.Keq = Keq;
r.thetaStatic = gamma * p.F0 / Keq;
r.status = "static_only";

%% Static response
force = linspace(0, p.F0, 301)';
thetaStatic = gamma * force / Keq;
qStatic = p.Rp * thetaStatic;
uStatic = gamma * thetaStatic;
r.static = table(force, rad2deg(thetaStatic), qStatic * 1000, ...
    uStatic * 1000, 'VariableNames', ...
    {'Force_N','Dial_deg','Rack_mm','PlatformDown_mm'});
fprintf('Keq = %.6g N*m/rad; gamma = %.6g m\n', Keq, gamma);
fprintf('Static dial angle = %.6g deg at %.6g N\n', ...
    rad2deg(r.thetaStatic), p.F0);

%% Equivalent inertia and damping
if ~reportMissing(p, dynamicFields)
    validateFields(p, dynamicFields, false);
    JL = p.JL1 + p.JL2;
    JS = p.JS1 + p.JS2;
    MleverB = JL / p.L2^2 + JS * (p.L5 / (p.L2 * p.L3))^2;
    MB = p.mB + a^2 * p.mU + MleverB;
    JPe = MB * p.L6^2 + p.JR;
    Mqe = JPe / p.L7^2 + p.mr;
    Jeq = Mqe * p.Rp^2 + p.Jp + p.JD;
    Deq = p.DB * b^2 * p.Rp^2;
    assert(isfinite(Jeq) && Jeq > 0, 'Lab3:Inertia', 'Jeq must be positive.');
    assert(isfinite(Deq), 'Lab3:Damping', 'Deq must be finite.');
    wn = sqrt(Keq / Jeq);
    zeta = Deq / (2 * sqrt(Jeq * Keq));
    r.MB = MB;
    r.JPe = JPe;
    r.Mqe = Mqe;
    r.Jeq = Jeq;
    r.Deq = Deq;
    r.wn = wn;
    r.zeta = zeta;
    r.transferNumerator = gamma;
    r.transferDenominator = [Jeq, Deq, Keq];

    %% Step response
    poles = roots([Jeq, Deq, Keq]);
    if Deq > 0
        tEnd = max(8 / min(-real(poles)), 4 * 2*pi / wn);
    else
        tEnd = 8 * 2*pi / wn;
    end
    r.horizonCapped = tEnd > 40 * 2*pi / wn;
    tEnd = min(tEnd, 40 * 2*pi / wn);
    if r.horizonCapped
        fprintf('Time horizon capped at 40 natural periods.\n');
    end
    tspan = linspace(0, tEnd, 2001);
    options = odeset('RelTol', 1e-9, 'AbsTol', 1e-11);
    ode = @(t,x) [x(2); (gamma*p.F0 - Deq*x(2) - Keq*x(1)) / Jeq];
    [t, x] = ode45(ode, tspan, [0; 0], options);
    theta = x(:,1);
    q = p.Rp * theta;
    sB = -b * q;
    sU = a * sB;
    phi = q / p.L7;
    r.transient = table(t, theta, x(:,2), q, sB, sU, phi, ...
        'VariableNames', {'Time_s','Theta_rad','Omega_rad_s', ...
        'Rack_m','CollectorUp_m','PlatformUp_m','Lever5_rad'});
    r.status = "complete";
    fprintf('Jeq = %.6g kg*m^2; Deq = %.6g N*m*s/rad\n', Jeq, Deq);
    fprintf('wn = %.6g rad/s; zeta = %.6g\n', wn, zeta);
    r.full = lab3_full_parameter_analysis(p,r,outputDir);
end

%% Plots and data
if strlength(string(outputDir)) > 0
    if ~isfolder(outputDir)
        mkdir(outputDir);
    end
    writetable(r.static, fullfile(outputDir, 'static_response.csv'));
    f = figure('Color','w', 'Name','Lab 3 static response');
    tiledlayout(2,1, 'TileSpacing','compact');
    nexttile;
    plot(force, rad2deg(thetaStatic), 'LineWidth',1.5);
    xlabel('Applied force (N)'); ylabel('Dial angle (deg)');
    title('Static dial response'); legend('Model','Location','best'); grid on;
    nexttile;
    plot(force, [qStatic, uStatic]*1000, 'LineWidth',1.5);
    xlabel('Applied force (N)'); ylabel('Displacement (mm)');
    title('Static displacement');
    legend('Rack','Platform downward','Location','best'); grid on;
    exportgraphics(f, fullfile(outputDir, 'static_response.png'), 'Resolution',300);
    savefig(f, fullfile(outputDir, 'static_response.fig'));

    if r.status == "complete"
        writetable(r.transient, fullfile(outputDir, 'step_response.csv'));
        f = figure('Color','w', 'Name','Lab 3 step response');
        tiledlayout(2,1, 'TileSpacing','compact');
        nexttile;
        plot(t, rad2deg(theta), 'LineWidth',1.5); hold on;
        yline(rad2deg(r.thetaStatic), '--', 'LineWidth',1.2);
        xlabel('Time (s)'); ylabel('Dial angle (deg)');
        title(sprintf('Step response: F = %.4g N', p.F0));
        legend('ODE solution','Static equilibrium','Location','best'); grid on;
        nexttile;
        plot(t, [q, -sU]*1000, 'LineWidth',1.5);
        xlabel('Time (s)'); ylabel('Displacement (mm)');
        title('Rack and platform response');
        legend('Rack','Platform downward','Location','best'); grid on;
        exportgraphics(f, fullfile(outputDir, 'step_response.png'), 'Resolution',300);
        savefig(f, fullfile(outputDir, 'step_response.fig'));
    end
    save(fullfile(outputDir, 'lab3_results.mat'), 'r');
end

phiMax = abs(p.Rp * r.thetaStatic / p.L7);
if r.status == "complete"
    phiMax = max(phiMax, max(abs(r.transient.Lever5_rad)));
end
if phiMax > 0.1
    warning('Lab3:SmallMotion', 'Lever 5 rotation reaches %.3g rad; check the small-motion model.', phiMax);
end
end

function missing = reportMissing(p, names)
missingNames = names(cellfun(@(n) ~isfield(p,n) || ...
    (isnumeric(p.(n)) && isscalar(p.(n)) && isnan(p.(n))), names));
missing = ~isempty(missingNames);
if missing
    fprintf('Missing parameters: %s\n', strjoin(missingNames, ', '));
end
end

function validateFields(p, names, positive)
for k = 1:numel(names)
    if positive
        validateattributes(p.(names{k}), {'numeric'}, ...
            {'scalar','real','finite','positive'}, mfilename, names{k});
    else
        validateattributes(p.(names{k}), {'numeric'}, ...
            {'scalar','real','finite','nonnegative'}, mfilename, names{k});
    end
end
end

function p = lab3_parameters()
% SI units; assumed values are labeled.

p.L1 = 0.060;                  % m; A to E
p.L2 = 0.240;                  % m; A to B
p.L3 = 0.120;                  % m; D to C
p.L4 = 0.060;                  % m; D to F
p.L5 = 0.120;                  % m; A to C
p.L6 = 0.008;                  % m; P to B
p.L7 = 0.020;                  % m; P to S
p.pinionOutsideDiameter = 0.005; % m
p.pinionTeeth = 10;
p.Rp = p.pinionOutsideDiameter*p.pinionTeeth/(2*(p.pinionTeeth+2));
% Assumed standard unshifted full-depth teeth.
p.zetaCases = [0.2 0.5 1 2];  % Assumed damping ratios
p.assumedDampingRatio = 0.5;  % Assumed default damping ratio

p.K = 500;                     % N/m; each retaining spring, assumed
p.K4 = 50;                     % N/m; rack spring, assumed

p.mU = 0.56511;                % kg; upper platform and fittings
p.mD = 0.00923;                % kg; dial
p.mS_total = 0.04389;           % kg; both short levers
p.mAssembly = 0.10842;          % kg; long-lever, bracket and spring assembly
p.mRetainer = 0.00141;          % kg; one retaining spring
p.mRackSpring = 0.00043;        % kg; rack return spring
p.DD = 0.120;                  % m; estimated dial diameter
p.retainerCoilLength = 0.010;   % m; approximate free coil body
p.rackFreeCoilLength = 0.013;   % m; approximate free coil body
p.rackInstalledCoilLength = 0.040; % m; approximate installed coil body
p.nominalLbPerRev = 300;        % lb/rev; printed dial graduations
p.rawBaselineIndicationLb = [9 11]; % lb; unloaded printed-scale reading range
p.rawBaselineMidpointLb = 10;  % lb; same baseline used before and after
p.beforeIndicationLb = [12 14]; % lb; raw before-disassembly reading range
p.afterIndicationLb = [12 14];  % lb; raw after-reassembly reading range
p.loadedIndicationLb = p.afterIndicationLb; % lb; raw range used for calibration
p.calibrationScenario = "MacBook Air";
p.calibrationMassKg = 1.24;    % kg; Apple nominal mass for the MacBook Air
p.calibrationMassSource = "https://support.apple.com/en-gb/111867";
p.mB = 0.020;                  % kg; collector, assumed
p.mr = 0.005;                  % kg; rack, assumed
p.mLongLever = 0.040;          % kg; each long lever, assumed
p.mSpringRemainder = p.mAssembly-2*p.mLongLever-p.mB;
% Spring/fittings remainder has no modeled inertia.

p.JL1 = p.mLongLever*p.L2^2/3; % kg*m^2; uniform end-pivot rod
p.JL2 = p.JL1;
p.JS1 = p.mS_total*p.L3^2/6;   % kg*m^2; equal uniform end-pivot rods
p.JS2 = p.JS1;
p.JR = 4e-7;                  % kg*m^2; Lever 5 about P, assumed
p.Jp = 5e-10;                  % kg*m^2; pinion inertia, assumed
p.JD = p.mD * p.DD^2 / 8;      % kg*m^2; uniform thin-disk estimate

p.F0 = p.calibrationMassKg * 9.80665; % N; nominal reference weight

% K3 calibrated with assumed K and K4.
a = p.L1/p.L2; b = p.L6/p.L7; gamma = a*b*p.Rp;
deltaTheta = deg2rad((mean(p.loadedIndicationLb)-p.rawBaselineMidpointLb) ...
    *360/p.nominalLbPerRev);
p.KeqCalibration = gamma*p.F0/deltaTheta;
p.K3 = (p.KeqCalibration/p.Rp^2-p.K4)/b^2-2*p.K*a^2;
assert(p.K3 > 0,'Lab3:Calibration','The assumed spring allocation requires K3 > 0.');
MB = p.mB+a^2*p.mU+(p.JL1+p.JL2)/p.L2^2 ...
    +(p.JS1+p.JS2)*(p.L5/(p.L2*p.L3))^2;
Jeq = p.Jp+p.JD+p.Rp^2*(p.mr+b^2*MB+p.JR/p.L7^2);
p.DB = 2*p.assumedDampingRatio*sqrt(Jeq*p.KeqCalibration)/(b*p.Rp)^2;
end

function r = lab3_photo_analysis(p, outputDir)
% Static calibration and normalized response.

g = 9.80665;
kgPerLb = 0.45359237;
r.JD = p.mD * p.DD^2 / 8;
r.degPerLb = 360 / p.nominalLbPerRev;
r.forcePerRad = p.nominalLbPerRev * kgPerLb * g / (2*pi);
massLb = (0:1:p.nominalLbPerRev)';
forceN = massLb * kgPerLb * g;
thetaDeg = r.degPerLb * massLb;
r.nominal = table(massLb, forceN, thetaDeg, ...
    'VariableNames', {'IndicatedMass_lb','EquivalentForce_N','NominalDial_deg'});

%% Before/after readings
validateattributes(p.rawBaselineIndicationLb, {'numeric'}, ...
    {'row','numel',2,'real','finite'});
validateattributes(p.rawBaselineMidpointLb, {'numeric'}, ...
    {'scalar','real','finite'});
validateattributes(p.beforeIndicationLb, {'numeric'}, ...
    {'row','numel',2,'real','finite'});
validateattributes(p.afterIndicationLb, {'numeric'}, ...
    {'row','numel',2,'real','finite'});
stageRange = [p.beforeIndicationLb; p.afterIndicationLb];
assert(all(stageRange(:,2) >= stageRange(:,1)), ...
    'Lab3:ReadingRange', 'Reading interval endpoints must be ordered.');
baseline = p.rawBaselineIndicationLb;
baselineMidpoint = p.rawBaselineMidpointLb;
assert(baseline(1) <= baselineMidpoint && baselineMidpoint <= baseline(2), ...
    'Lab3:Baseline', 'Baseline midpoint must lie inside its reading interval.');
assert(isequal(p.loadedIndicationLb,p.afterIndicationLb), ...
    'Lab3:CalibrationStage', 'Static calibration must use the after-reassembly range.');
stageMidpoint = mean(stageRange,2);
netStageRange = [stageRange(:,1)-baseline(2),stageRange(:,2)-baseline(1)];
netStageMidpoint = stageMidpoint-baselineMidpoint;
netStageKg = netStageRange * kgPerLb;
r.beforeAfter = table(["Before disassembly";"After reassembly"], ...
    repmat(p.calibrationMassKg,2,1),repmat(baseline(1),2,1), ...
    repmat(baselineMidpoint,2,1),repmat(baseline(2),2,1), ...
    stageRange(:,1),stageMidpoint,stageRange(:,2), ...
    netStageRange(:,1),netStageMidpoint,netStageRange(:,2), ...
    netStageKg(:,1),netStageMidpoint*kgPerLb,netStageKg(:,2), ...
    repmat("Photo intervals; shared baseline; stage follows upload order",2,1), ...
    'VariableNames', {'Stage','NominalReferenceMass_kg', ...
    'RawBaselineLow_lb','RawBaselineMidpoint_lb','RawBaselineHigh_lb', ...
    'RawIndicationLow_lb','RawIndicationMidpoint_lb','RawIndicationHigh_lb', ...
    'NetIndicationLow_lb','NetIndicationMidpoint_lb','NetIndicationHigh_lb', ...
    'NetIndicatedMassLow_kg','NetIndicatedMassMidpoint_kg','NetIndicatedMassHigh_kg', ...
    'ReadingBasis'});
% Shared baseline cancels in the reading difference.
changeLb = [p.afterIndicationLb(1)-p.beforeIndicationLb(2), ...
    p.afterIndicationLb(2)-p.beforeIndicationLb(1)];
changeMidpoint = stageMidpoint(2)-stageMidpoint(1);
r.beforeAfterChange = table("After minus before",changeLb(1), ...
    changeMidpoint,changeLb(2),changeLb(1)*kgPerLb, ...
    changeMidpoint*kgPerLb,changeLb(2)*kgPerLb, ...
    "Shared baseline cancels; conservative photo bounds, not a confidence interval", ...
    'VariableNames', {'Comparison','ChangeLow_lb','MidpointChange_lb', ...
    'ChangeHigh_lb','ChangeLow_kg','MidpointChange_kg','ChangeHigh_kg','Basis'});
r.changeResolved = changeLb(1)>0 || changeLb(2)<0;

%% Static calibration
validateattributes(p.calibrationMassKg, {'numeric'}, ...
    {'scalar','real','finite','positive'});
validateattributes(p.loadedIndicationLb, {'numeric'}, ...
    {'vector','numel',2,'real','finite'});
assert(p.loadedIndicationLb(2) >= p.loadedIndicationLb(1) && ...
    p.loadedIndicationLb(1) > baseline(2), ...
    'Lab3:ReadingRange', 'Check the loaded and unloaded reading intervals.');
calForce = p.calibrationMassKg * g;
netRange = [p.loadedIndicationLb(1)-baseline(2), ...
    p.loadedIndicationLb(2)-baseline(1)];
netMidpoint = mean(p.loadedIndicationLb)-baselineMidpoint;
angleDeg = netRange * r.degPerLb;
angleMidpointDeg = netMidpoint*r.degPerLb;
angleRad = deg2rad(angleDeg);
angleMidpointRad = deg2rad(angleMidpointDeg);
sLow = angleRad(1) ./ calForce;
sMidpoint = angleMidpointRad ./ calForce;
sHigh = angleRad(2) ./ calForce;
kOverGammaLow = calForce / angleRad(2);
kOverGammaMidpoint = calForce / angleMidpointRad;
kOverGammaHigh = calForce / angleRad(1);
expectedNetLb = p.calibrationMassKg / kgPerLb;
expectedRawLb = baselineMidpoint + expectedNetLb;
massRatio = [netRange(1),netMidpoint,netRange(2)] * kgPerLb / p.calibrationMassKg;
n = numel(calForce);
r.staticCalibration = table(p.calibrationScenario, p.calibrationMassKg, ...
    calForce,repmat(baseline(1),n,1),repmat(baselineMidpoint,n,1),repmat(baseline(2),n,1), ...
    repmat(p.loadedIndicationLb(1),n,1), repmat(p.loadedIndicationLb(2),n,1), ...
    repmat(netRange(1),n,1),repmat(netMidpoint,n,1),repmat(netRange(2),n,1), ...
    repmat(angleDeg(1),n,1),repmat(angleMidpointDeg,n,1),repmat(angleDeg(2),n,1), ...
    repmat(angleRad(1),n,1),repmat(angleMidpointRad,n,1),repmat(angleRad(2),n,1), ...
    sLow,sMidpoint,sHigh,kOverGammaLow,kOverGammaMidpoint,kOverGammaHigh, ...
    expectedNetLb,expectedRawLb,massRatio(1),massRatio(2),massRatio(3), ...
    repmat("Apple nominal mass for the MacBook Air",n,1), ...
    repmat("After-reassembly raw reading minus shared photo baseline",n,1), ...
    p.calibrationMassSource, ...
    'VariableNames', {'Scenario','NominalMass_kg','Force_N', ...
    'RawBaselineLow_lb','RawBaselineMidpoint_lb','RawBaselineHigh_lb', ...
    'RawLoadedLow_lb','RawLoadedHigh_lb','NetLow_lb','NetMidpoint_lb','NetHigh_lb', ...
    'DeltaAngleLow_deg','DeltaAngleMidpoint_deg','DeltaAngleHigh_deg', ...
    'DeltaAngleLow_rad','DeltaAngleMidpoint_rad','DeltaAngleHigh_rad', ...
    'SThetaLow_rad_per_N','SThetaMidpoint_rad_per_N','SThetaHigh_rad_per_N', ...
    'KeqOverGammaLow_N_per_rad','KeqOverGammaMidpoint_N_per_rad','KeqOverGammaHigh_N_per_rad', ...
    'NominalExpectedNet_lb','NominalExpectedRaw_lb','NetMassRatioLow','NetMassRatioMidpoint','NetMassRatioHigh', ...
    'MassBasis','ReadingBasis','MassSource'});
assert(all(abs(sLow .* kOverGammaHigh - 1) < 1e-12) && ...
    all(abs(sHigh .* kOverGammaLow - 1) < 1e-12), ...
    'Lab3:StaticCalibration', 'Reciprocal interval check failed.');

%% Normalized step response
zeta = [0.2, 0.5, 1, 2];
tau = linspace(0, 30, 3001)';
y = zeros(numel(tau), numel(zeta));
overshoot = zeros(numel(zeta),1);
maxError = zeros(numel(zeta),1);
options = odeset('RelTol',1e-9, 'AbsTol',1e-11);
for k = 1:numel(zeta)
    z = zeta(k);
    [~, x] = ode45(@(t,x) [x(2); 1-2*z*x(2)-x(1)], tau, [0;0], options);
    y(:,k) = x(:,1);
    if z < 1
        wd = sqrt(1-z^2);
        exact = 1-exp(-z*tau).*(cos(wd*tau)+z/wd*sin(wd*tau));
        overshoot(k) = 100*exp(-pi*z/wd);
    elseif z == 1
        exact = 1-exp(-tau).*(1+tau);
    else
        r1 = -z+sqrt(z^2-1);
        r2 = -z-sqrt(z^2-1);
        exact = 1+(r2*exp(r1*tau)-r1*exp(r2*tau))/(r1-r2);
    end
    maxError(k) = max(abs(y(:,k)-exact));
end
r.normalized = array2table([tau,y], 'VariableNames', ...
    {'Tau','Zeta_0_2','Zeta_0_5','Zeta_1','Zeta_2'});
r.validation = table(zeta', overshoot, maxError, 'VariableNames', ...
    {'DampingRatio','AnalyticalOvershoot_pct','MaxODEError'});
assert(all(maxError < 1e-7), 'Lab3:ODESolution', 'ODE verification failed.');

%% Measurements
parameter = ["mU";"mD";"mS_total";"mAssembly";"mRetainer";"mRackSpring"; ...
    "DD";"retainerCoilLength";"rackFreeCoilLength";"rackInstalledCoilLength";"JD"];
value = [p.mU;p.mD;p.mS_total;p.mAssembly;p.mRetainer;p.mRackSpring; ...
    p.DD;p.retainerCoilLength;p.rackFreeCoilLength;p.rackInstalledCoilLength;r.JD];
unit = [repmat("kg",6,1);repmat("m",4,1);"kg*m^2"];
basis = [repmat("Balance display",6,1);repmat("Photo estimate",4,1);"Uniform thin disk"];
figureSource = ["25";"24";"23";"22";"21";"26";"20";"18";"19";"15";"20, 24"];
r.measurements = table(parameter,value,unit,basis,figureSource, ...
    'VariableNames', {'Parameter','Value_SI','Unit','Basis','DisassemblyFigure'});
fprintf('Static calibration: F = %.6g N; theta = %.6g deg.\n', ...
    calForce,angleMidpointDeg);
fprintf('Normalized ODE max error = %.3g.\n',max(maxError));

if strlength(string(outputDir)) == 0
    return
end
if ~isfolder(outputDir)
    mkdir(outputDir);
end
writetable(r.measurements, fullfile(outputDir,'photo_measurements.csv'));
writetable(r.nominal, fullfile(outputDir,'nominal_dial.csv'));
writetable(r.staticCalibration, fullfile(outputDir,'static_calibration.csv'));
writetable(r.beforeAfter, fullfile(outputDir,'before_after_comparison.csv'));
writetable(r.beforeAfterChange, fullfile(outputDir,'before_after_change.csv'));
writetable(r.normalized, fullfile(outputDir,'normalized_response.csv'));
writetable(r.validation, fullfile(outputDir,'ode_validation.csv'));

%% Report figures
f = figure('Color','w','Name','Baseline-corrected calibration', ...
    'Units','inches','Position',[1 1 3.35 2.65]);
fitForce = linspace(0,1.08*max(calForce),201)';
colors = [0.10 0.36 0.68];
hold on;
bands = gobjects(n,1);
for k = 1:n
    lower = rad2deg(sLow(k)*fitForce);
    upper = rad2deg(sHigh(k)*fitForce);
    bands(k) = fill([fitForce;flipud(fitForce)], [lower;flipud(upper)], ...
        colors(k,:), 'FaceAlpha',0.22, 'EdgeColor','none');
    plot(fitForce,rad2deg(sMidpoint(k)*fitForce), 'Color',colors(k,:), ...
        'LineWidth',0.8, 'HandleVisibility','off');
    errorbar(calForce(k),angleMidpointDeg,angleMidpointDeg-angleDeg(1), ...
        angleDeg(2)-angleMidpointDeg, 'o', ...
        'Color',colors(k,:), 'MarkerFaceColor','w', 'MarkerSize',3.5, ...
        'LineWidth',1.1, 'CapSize',7, 'HandleVisibility','off');
end
nominalLine = plot(fitForce,rad2deg(fitForce/r.forcePerRad), ...
    'k--','LineWidth',1.1);
xlabel('Applied force (N)'); ylabel('Incremental dial rotation (deg)');
title('Baseline-corrected calibration');
legend([bands;nominalLine], ["Fitted range";"Nominal dial"], ...
    'Location','northwest','FontSize',7);
grid on; xlim([0 max(fitForce)]); ylim([0 1.08*rad2deg(max(sHigh)*max(fitForce))]);
set(gca,'FontName','Times New Roman','FontSize',8);
exportgraphics(f,fullfile(outputDir,'Fig17_Static_Calibration.png'),'Resolution',400);
savefig(f,fullfile(outputDir,'Fig17_Static_Calibration.fig'));

f = figure('Color','w','Name','Normalized damping response', ...
    'Units','inches','Position',[1 1 3.35 2.6]);
styles = {'-','--','-.',':'};
for k = 1:numel(zeta)
    plot(tau,y(:,k),styles{k},'LineWidth',1.15); hold on;
end
yline(1,'k:','HandleVisibility','off');
xlabel('Normalized time, \tau = \omega_n t');
ylabel('Normalized angle, \theta / \theta_{ss}');
title('Damping-ratio sensitivity');
legend('\zeta = 0.2','\zeta = 0.5','\zeta = 1','\zeta = 2', ...
    'Location','northeast');
grid on; xlim([0 30]); ylim([0 1.65]);
set(gca,'FontName','Times New Roman','FontSize',8);
exportgraphics(f,fullfile(outputDir,'Fig18_Normalized_Response.png'),'Resolution',400);
savefig(f,fullfile(outputDir,'Fig18_Normalized_Response.fig'));
end

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
