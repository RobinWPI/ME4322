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
