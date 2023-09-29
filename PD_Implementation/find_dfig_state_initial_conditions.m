%STEP 2 BEGINS: Gather machine parameter for DFIG
d_welb = 2*pi*60; % electrical base speed rad/sec
d_ws = 1.0; % Synchronous speed in pu

% DFIG parameters
%data_DP- (mach# bus# d_Lm d_Rs d_Rr d_Lss d_Lrr d_Kopt d_ktg d_ctg d_Ht d_Hg BaseMVA]
data_DF = [1 5 4 0.005 0.0055 4.04 4.0602 1 0.3 0.01 4 0.4 5];

%DFIG-Filter parameters
%FLTR = [d_Ri, d_Rg, d_Rc, d_Li, d_Lg, d cf]:
d_FLTR = [10.000, 0.000, 0.7333, 0.1667, 0.0033, 0.0150];
d_Cdc = 2; % Converter capacitor

%DFIG-MSC Controller Parameters
d_MSC_IL1_kp = -0.23; d_MSC_IL1_ki = -3;
d_MSC_IL2_kp = -0.23; d_MSC_IL2_ki = -3;
d_MSC_OL1_kp = 0; d_MSC_OL1_ki = -60;
d_MSC_OL2_kp = 0; d_MSC_OL2_ki = 90;

%DFIG-GSC Controller Parameters

d_GSC_IL1_kp = 0.3; d_GSC_IL1_ki = 200;
d_GSC_IL2_kp = 0.3; d_GSC_IL2_ki = 200;
d_GSC_OL1_kp = -22; d_GSC_OL1_ki = -870;
d_GSC_OL2_kp = 0; d_GSC_OL2_ki = -60;

%Read filter parameters from FLTR vector
d_Ri= 0.0;
d_Rg= 0.0;
d_Rc= d_FLTR(3);
d_Li= d_FLTR(4);
d_Lg= d_FLTR (5);
d_Cf= d_FLTR (6);

%Read machine data from data_DE  
d_Lm = data_DF(3); 
d_xm = d_Lm;
d_Rs = data_DF(4);% stator resistance
d_Rr = data_DF(5);%rotor resistance
d_Lss = data_DF(6);% stator self inductance
d_Lrr = data_DF(7) ;%rotor inductance
d_kopt = data_DF(8);
d_ktg= data_DF(9);
d_ctg = data_DF(10);
d_Ht = data_DF(11);
d_Hg = data_DF(12);
d_base = data_DF(13);

d_bl = 40.05; % blade length
d_wtrated = 3.0337; % rated turbine speed
rho = 1.225; %air density

% Calcuating derived variable using equation () - )
d_Ls_d = d_Lss -(d_Lm^2/d_Lrr);
d_Kmrr = d_Lm/d_Lrr;
d_R2 = d_Kmrr^2*d_Rr;
d_R1 = d_Rs + d_R2;
d_Tr = d_Lrr/d_Rr;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%STEP 2 ENDS
%STEP 3 BEGINS: Initialization of State variables for generator,
%Converter and Filter

Vdfig = bus_sln(Dmachs,2).*exp(1i*bus_sln(Dmachs,3)*pi/180);
Pdfig = bus_sln(Dmachs, 4);
Qdfig = bus_sln(Dmachs, 5);
d_vsq = real(Vdfig); d_vsd = imag(Vdfig);
d_Theta = angle(Vdfig);

xdfig = zeros(size(Dmachs,1),15);
    for d_index = 1:size(Dmachs,1)
        xdfig0 = ones(1,15);
    if Pdfig(d_index) <1 % If below rated speed
            
% The function solve is used to slnve set of SSCs described in the
%function init_dfig_mpt. The output is stored in xdfig variable

        xdfig(d_index,:) = fsolve(@(x)...
init_dfig_mpt(x,Vdfig(d_index),Pdfig(d_index),...
Qdfig(d_index),data_DF,d_FLTR),...
            xdfig0, optimset('TolFun',1e-16,'TolX',1e-16));
    else
%The function fslnve is used to slnve set of SSCs described in the
%function init_dfig_cpt. The output is stored in xdfig variable

            xdfig(d_index,:) = fsolve(@(x)...
init_dfig_cpt(x,Vdfig(d_index),Pdfig(d_index),...
Qdfig(d_index),data_DF,d_FLTR),...
            xdfig0, optimset('TolFun',1e-16,'TolX',1e-16));
    end
   end

    
d_isq = xdfig(:,1); d_isd = xdfig(:,2);
d_irq = xdfig(:, 3); d_ird = xdfig(:, 4);
d_vrq = xdfig(:,5); d_vrd = xdfig(:,6);
d_iiq= xdfig(:,7); d_iid= xdfig(:,8);
d_igq = xdfig(:,9); d_igd = xdfig(:,10);
d_viq = xdfig(:,11); d_vid = xdfig(:, 12);
d_vcq = xdfig(:, 13); d_vcd = xdfig(:,14);
d_wg = xdfig(:,15);

%d_esq and d_esd are calculated using (6.12) and (6.13)
d_esq =  d_Kmrr.*d_ws.*(d_Lrr.*d_ird + d_Lm.*d_isd);
d_esd = -d_Kmrr.*d_ws.*(d_Lrr.*d_irq + d_Lm.*d_isq);



%********* STEP 4 BEGINS: Initialization of converter controllers
%and turbine

% Parameters for RSC controller model

d_vr_dash = (d_vrq+1i*d_vrd).*exp(-1i*d_Theta);
d_ir_dash = (d_irq+1i*d_ird).*exp(-1i*d_Theta);
d_MSC_IL1_iv = real(d_vr_dash);
d_MSC_IL2_iv = imag(d_vr_dash);
d_MSC_OL1_iv = real(d_ir_dash);
d_MSC_OL2_iv = imag(d_ir_dash);

d_Qs = Qdfig; % Reactive power reference RSC controller

% B2B capacitor
d_VDC = 1.5;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Parameters for GSC controller model
d_vi_dash = (d_viq+1i*d_vid).*exp(-1i*d_Theta);
d_ii_dash = (d_igq+1i*d_igd).*exp(-1i*d_Theta);

d_GSC_IL1_iv = real(d_vi_dash);
d_GSC_IL2_iv = imag(d_vi_dash);
d_GSC_OL1_iv = real(d_ii_dash);
d_GSC_OL2_iv = imag(d_ii_dash);

d_Qfilter = 0; %Reactive power reference GSC controller

 %Turbine initialisation
d_Tg = d_esq.*d_isq+d_esd.*d_isd;
d_TS = d_Tg;
d_wt = d_wg;
d_Pt = d_Ts.*d_wt;


for d_index = 1:size(Dmachs,1)
    
    if d_wg(d_index) >=1 % Above rated wind speed operation
            d_Sw(d_index) = 15;
            d_Lambda = 3.0337*d_wg(d_index).*d_bl/d_Sw(d_index);
Cp_req = (d_base*d_Pt(d_index)*1e6)/(0.5*1.225*pi*d_bl^22*d_Sw(d_index)^3);
            d_Beta(d_index) = find_beta(Cp_req, d_Lambda);
        else
            
%Below rated wind speed

d_Lambda = 8.1;
d_Beta = 0;

d_Sw(d_index,1) = (d_Pt(d_index)*d_base*1e6/(0.5*1.225*pi*d_bl^2*0.48))^(1/3);
    end
end


