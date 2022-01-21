function cdmp = init_fs(cdmp,encVSrep)

%% INITIALISE PARAMETERS

data_length = cdmp.(append(encVSrep,'length'));
alpha_s     = cdmp.(append(encVSrep,'alpha_s'));
tau         = cdmp.(append(encVSrep,'tau'));

S           = zeros(data_length,1);
dS          = zeros(data_length,1);

% Canoncial System 
s = 1;
for i=1:data_length
    S(i) = s; 
    dS(i) = - alpha_s / tau * s;
    s = s + dS(i)*cdmp.dt;
end

% Radial Basis Function centers
switch cdmp.rbf_mode 
    case 'equal_in_s'
        % Centers equal distance in s, not in t
        rbf_cs = linspace(S(1), S(end), cdmp.rbf_num)'; 
    case 'equal_in_t'
        % Centers equal distance in t, not in s
        rbf_cs = exp(-alpha_s/tau*(linspace(0,data_length*cdmp.dt,cdmp.rbf_num)')); 
end

% Radial Basis Function widths
rbf_hs = 1./diff(rbf_cs).^2;
rbf_hs = cdmp.rbf_width.*[rbf_hs; rbf_hs(end)];

% Radial Basis Functions
psi = exp(-((S * ones(1,length(rbf_cs)) ...
     - ones(data_length,1)*rbf_cs').^2) ...
     .*(ones(data_length,1)*rbf_hs'));


%% STORE PARAMETERS IN CDMP STRUCTURE

cdmp.(append(encVSrep,'s'))      = S;
cdmp.(append(encVSrep,'rbf_cs')) = rbf_cs;
cdmp.(append(encVSrep,'rbf_hs')) = rbf_hs;
cdmp.(append(encVSrep,'psi'))    = psi;


end


