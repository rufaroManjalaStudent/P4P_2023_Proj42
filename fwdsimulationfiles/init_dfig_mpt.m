function F = init_dfig_mpt(x,Vdfig,Pg,Qg, data_DF, FLTR)
% This function is used to initialize DFIG for maximum power point
%tracking
%region. The prefix 'd' is not used in the variables inside the
%function
%for ease of reading. The variables defined inside function are not
%visible
%outside it.
% Get voltage, and power from Vdfig
vqs = real(Vdfig);
vds = imag(Vdfig);
ws = 1.0; % synchronous speed

%Get machine parameters from data_DFIG matrix
Lm = data_DF(:,3);
Rs = data_DF(:,4); %stator resistance
Rr = data_DF(:,5); % rotor resistance
Lss = data_DF(:,6); %stator self inductance
Lrr = data_DF(:,7); %rotor inductance
kopt = data_DF(8);

%Get filter parameters from FLTR matrix
R1f = FLTR(1); R2f = FLTR(2);
Rcf = FLTR(3);
L1f = FLTR(4);
L2f = FLTR(5);
Cf = FLTR(6);
% x is the solution vector. The commented section below shows index
%of
% various parameters
% x(l) = iqs, x(2) = ids, x(3) = iqr, x(4) = idr
% x(5) = var, x(6) = vdr
% x(7) = iig, x(8) = i1d, x(9) = 12q, x(10) = 12d
% x(11) = viq, x(12) = vid, x(13) = vcq, x(14) = vcd ,x(15) = wr

%set of Ax-b - O equations. Refer Section xx for details.
F = [-Rs*x(1)+ws*(Lss*x(2)+Lm*x(4))-vqs; %SSC 1
    -Rs*x(2)-ws*(Lss*x(1)+Lm*x(3))-vds; % SSC 2
    -Rr*x(3)+(ws-x(15))*(Lrr*x(4)+Lm*x(2))-x(5); % SSC 3
    -Rr+x(4)-(ws-x(15))*(Lrr*x(3)+Lm*x(1))-x(6); % SSC 4
    Lm*(x(1)*x(4)-x(2)*x(3))-kopt*x(15)^2; % SSC 5
    vqs*x(1)+vds*x(2)+vqs*x(9)+vds*x(10)-Pg; %SSC 6
    -vqs*x(2)+vds*x(1)-Qg; %SSC 7
    -vqs*x(10)+vds*x(9);%SSC 8
    x(5)*x(3)+x(6)*x(4)-x(11)*x(7)-x(12)*x(8); % SSC 9
    x(7)*(R1f+Rcf)-x(8)*L1f-x(9)*Rcf + x(13)-x(11); % SSC 10
    x(8)*(R1f+Rcf)+x(7)*L1f-x(10)*Rcf+x(14)-x(12); %SSC11
    -x(7)*Rcf+x(9)*(Rcf+R2f)-x(10)*L2f+ vqs-x(13); % SSC 12
    -x(8)*Rcf+x(10)*(Rcf+R2f)+x(9)*L2f+vds-x(14); %SSC 13
    x(7)-x(9)+Cf*x(14); % SSC 14
    x(8)-x(10)-Cf*x(13)] ; % SSC 15
