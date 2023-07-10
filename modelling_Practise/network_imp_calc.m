%This script contains the bus and line matrix for an example three 
% bus 
% network shown in Figure 6.1. The WIG is connected at Bus 3 and Bus 
% 1 is the infinite bus. The WTG output is fixed and hence bus 3 is 
% defined as a load bus. Ensure that functions,
% form_Ymatrix.m and power_flow.m, given in Chapter 2 are saved in 
% the current working folder. The functions are used to obtain the 
% 4 power flow and admittance matrix. The network impedance to use in
% the network block of Simulink program is calculated from the 
% 4 admittance matrix. 
clear all 
%4**** Part 1: Power Flow, Calculation of Network Impedance   
% bus data format
% bus: number, voltage(pu), angle(degree), P _gen(pu), q_gen(pu), 
%       p_load(pu), q_load(pu),G-shunt (pu), B shunt (pu); 
%       bus_type bus_type - 1, swing bus  
%                         - 2, generator bus (PV bus) 
%                         - 3, load bus (PQ bus) 

bus = [... 
    01 1.05 0.00 1.00 0.00 2.00 0.30 0.00 0.00 1; 
    02 1.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 3; 
    03 1.00 0.00 0.80 0.10 0.00 0.00 0.00 0.00 3];

% line data format 
% line: from bus, to bus, resistance fpu) , reactance (pu), 
%   line charging(pu), tap ratio 
line = [ 
    01 02 0.037 0.37 0.001 0.0 0.0;
    02 03 0.010 0.10 0.000 0.0 0.0]; 

Y = form_Ymatrix(bus,line); 
[bus_sln, flow] = power_flow(Y, bus, line); 
% Generator bus 
Gen_Bus = 3; 
%Infinite bus voltage 
vinf = bus_sln(1,2).*exp(1i*bus_sln(1,3)*pi/180); 
%Generator bus voltage 
Vg = bus_sln(Gen_Bus,2).*exp(1i*bus_sln(Gen_Bus,3)*pi/180);
%Current injection from generator 
d_ig = conj((bus_sln(Gen_Bus, 4)+1i*bus_sln(Gen_Bus, 5))/Vg); 

Znet = (Vg- vinf)/d_ig; % Network Impedance 
