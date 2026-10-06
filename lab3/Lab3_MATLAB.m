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
