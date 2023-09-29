%Loading the data to be used in this example
load WindCpData Bgrid TSRgrid CpData

%Values used for physical parameters
GBRatio = 88;      % Gearbox ratio, unitless
GBEff = 1.0;       % Gearbox efficiency, unitless (=1 for perfect eff.)
GenEff = 0.95;     % Generator efficiency, unitless (=1 for perfect eff.)
Jgen = 53;         % Generator Inertia about high-speed shaft, kg*m^2
Jrot = 3e6;        % Rotor inertia about low-speed shaft, kg*m^2
Jeq = Jrot+Jgen*GBRatio^2; % Equiv. inertia about low-speed shaft, kg*m^2
R = 35;            % Rotor radius, m
rho = 1.225;       % Air density, kg/m^3
wPA = 8.6;         % Pitch actuator natural frequency, rad/s
zetaPA = 0.8;      % Pitch actuator damping ratio, unitless
Bmax = 90;         % Maximum blade pitch angle, degrees
Kaw = 0.5;         % Anti-windup gain, rad/s/degree

%Opens Simulink model of rotor dynamics
open_system('WindTurbineOpenLoop')

PRated = 1.5e6; %rated power of the turbine (1.5MW)
wRatedHSS = 1800*(2*pi/60); %High Speed Shaft rated speed
wRatedLSS = wRatedHSS/GBRatio; %Low Speed Shaft rater speed, derived to express gearbox

GenTRated = PRated/(wRatedHSS*GenEff);%Rated generator power
WindRated = 11.2; %wind speed for when Cp<max(Cp)

% The controller switches between two operating regions delimited 
% by the rated operating point:
% 
% Region 2 (torque control): For wind speeds below rated, the blade pitch is 
% set equal to its optimal (most efficient) value and the generator torque 
% is set to a value proportional to ω
% 2
% .
% 
% Region 3 (blade pitch control): For wind speeds above rated, the generator 
% torque is set to its rated value while the blade pitch is adjusted to 
% maintain the rated rotor speed and deliver the rated power.
% 
% The generator torque in Region 2 is set to GenTRated = Kreg2×wRatedLSS^2, 
% where you choose Kreg2 such that there is a smooth transition with Region 3. 
% The rotor speed is wRatedLSS and generator torque is GenTRated.

Kreg2 = GenTRated / wRatedLSS^2;

%Finding max power coefficient (Cp) and optimal tip speed ratio and pitch
%angle
CpMax = max(CpData,[],'all');
[i,j] = find(CpData==CpMax);
TSRopt = TSRgrid(i);
Bopt = Bgrid(j);


WindData = sort([4:0.5:24 WindRated]);

nW = numel(WindData);
wLSSeq = zeros(nW,1); 
GenTeq = zeros(nW,1); 
BladePitcheq = zeros(nW,1);
Peq = zeros(nW,1); 


for i=1:nW
    Wind = WindData(i);
    if Wind<=WindRated
        % Region 2: Torque Control
        wLSSeq(i) = Wind/WindRated*wRatedLSS;
        GenTeq(i) = Kreg2*wLSSeq(i)^2;
        wHSS = wLSSeq(i)*GBRatio;
        Peq(i) = GenTeq(i)*wHSS*GenEff; % wRatedHSS*GenEff;
        BladePitcheq(i) = Bopt;
        % Populate operating point
        op(i) = operpoint('WindTurbineOpenLoop');
        op(i).States.x = wLSSeq(i);
        op(i).Inputs(1).u = Wind;
        op(i).Inputs(2).u = BladePitcheq(i);
        op(i).Inputs(3).u = GenTeq(i);
    end
end

opt = findopOptions('DisplayReport','off', 'OptimizerType','lsqnonlin');
opt.OptimizationOptions.Algorithm = 'trust-region-reflective';

opspec = operspec('WindTurbineOpenLoop');
for i=1:nW
    Wind = WindData(i);
    if Wind>WindRated
        % Region 3: Blade Pitch Control
        wLSSeq(i) = wRatedLSS;
        GenTeq(i) = GenTRated;
        Peq(i) = PRated;
        % Trim condition
        opspec.States.Known = 1;
        opspec.States.SteadyState = 1;
        opspec.Inputs(1).Known=1;
        opspec.Inputs(1).u = Wind;
        opspec.Inputs(2).min = BladePitcheq(i-1);
        opspec.Inputs(3).Known=1;
        opspec.Inputs(3).u = GenTeq(i);
        % Compute corresponding operating point
        op(i) = findop('WindTurbineOpenLoop',opspec,opt);
        % Log steady-state blade pitch angle
        BladePitcheq(i) = op(i).Inputs(2).u;
    end
end


%Plotting Results to be seen
clf
subplot(2,2,1)
plot(WindData,wLSSeq,'b',WindRated,wRatedLSS,'ro');
grid on; 
xlabel('Wind Speed, m/s'); 
title('Rotor Speed on LSS, rad/s');

subplot(2,2,2)
plot(WindData,GenTeq,'b',WindRated,GenTRated,'ro');
grid on; 
xlabel('Wind Speed, m/s'); 
title('Generator Torque on HSS, N*m');

subplot(2,2,3)
plot(WindData,Peq/1e6,'b',WindRated,PRated/1e6,'ro');
grid on; 
xlabel('Wind Speed, m/s'); 
title('Electric Power, MW');

subplot(2,2,4)
plot(WindData,BladePitcheq,'b');
grid on; 
xlabel('Wind Speed, m/s'); 
title('Blade Pitch, degrees');
