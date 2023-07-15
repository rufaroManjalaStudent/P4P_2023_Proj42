%Values associated with network using network imp_calc
%Znet = 0.0472 + 1i*0.4700;  d_Vinf = 1; 
%Dmachs = 3; 
% Values associated with generator
d_Lm = 4;   d_Xm = d_Lm;    d_Rs = 0.005; d_Rr = 0.0055;
d_Lss = 4.04; d_Lrr = 4.0602; d_kopt=1; d_ktg = 0.3;
d_ctg = 0.01; d_Ht = 4; d_Hg= 0.4;

d_Ls_d = d_Lss - (d_Lm^2/d_Lrr);
d_Kmrr = d_Lm/d_Lrr;
d_R2 = d_Kmrr^2*d_Rr;
d_R1 = d_Rs + d_R2;
d_Trd = d_Lrr/d_Rr;

Vdfig = 0.9794+ 1i*0.3983;
d_vsq= real (Vdfig);    d_vsd = imag (Vdfig);
d_Theta = angle(Vdfig);

disq = 0.8544;  d_isd =  0.2454;
dirq = -0.9629; d_ird = -0.0020;
d_vrq = 0.0357; d_vrd = 0.0154;

%Values associated with Filter
d_Ri= 0.0; d_Rg= 0.0; d_Rc= 0.7333; d_Li = 0.1667;
d_Lg= 0.0033; d_Cf = 0.0150;
diiq = -0.0361; d_iid = 0.0024; 
d_igq =  -0.0303; d_igd = -0.0123;
d_viq = 0.9790; d_vid = 0.3922; 
d_vcq = 0.9837; d_vcd = 0.3874;

%Values associated with converters 
d_Cdc = 2; d_VDC = 1.5; d_Qfilter = 0; d_Qs = 0.1;

d_MSC_IL1_kp = -0.23;   d_MSC_IL1_ki = -3;   d_MSC_IL1_iv = 0.0389;
d_MSC_IL2_kp = -0.23;   d_MSC_IL2_ki = -3;   d_MSC_IL2_iv = 0.00078156;
d_MSC_OL1_kp = 0;       d_MSC_OL1_ki = -60;  d_MSC_OL1_iv = -0.8927; 
d_MSC_OL2_kp = 0;       d_MSC_OL2_ki=90;     d_MSC_OL2_iv  =  0.3610;

d_GSC_IL1_kp = 0.3; d_GSC_IL1_ki= 200;  d_GSC_IL1_iv = 1.0546; 
d_GSC_IL2_kp = 0.3; d_GSC_IL2_ki = 200; d_GSC_IL2_iv = -0.0055; 
d_GSC_OL1_kp = -22; d_GSC_OL1_ki = -870; d_GSC_OL1_iv = -0.0327; 
d_GSC_OL2_kp = 0; d_GSC_OL2_ki = -60; d_GSC_OL2_iv = 0;

%Values associated with turbine
rho = 1.225;
d_wtrated = 3.0337; d_wt = 0.9688; d_wg = 0.9688;
d_bl = 40.05; d_Lambda = 8.1;
d_Beta = 0; d_vw = 14.5316; d_Ts = 0.9385; 

Dmachs = [3]; %Bus where PMSG is connected
Omega = 2*pi*50;
if size(Dmachs,1)  %NOTE potential typo just changed to Dmachs from Pmachs
    network_imp_calc %Running this first to acquire bus_sln
    find_dfig_state_initial_conditions
end
