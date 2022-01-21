function cdmp = run_cdmp(cdmp)

%% INITIALISE PARAMETERS

% Temporal Parameters
if isnan(cdmp.rep_tau)
    cdmp.rep_tau  = cdmp.demo_tau;
end
cdmp.rep_length = round(cdmp.rep_tau / cdmp.dt); 

% Canonical System Parameters
if isnan(cdmp.rep_alpha_s)
    % s reaches 0.001 
    cdmp.rep_alpha_s  = - log(0.001) * cdmp.rep_tau / (cdmp.rep_length * cdmp.dt); 
end

% Initialise Canonical System and Radial Basis Functions
cdmp = init_fs(cdmp,'rep_');

% Initialise reproduction trajectory
rep_pos    = repmat(cdmp.rep_start(1:3),cdmp.rep_length,1);
rep_linVel = zeros(cdmp.rep_length,3);
rep_linAcc = zeros(cdmp.rep_length,3);
rep_quat   = repmat(cdmp.rep_start(4:7),cdmp.rep_length,1);
rep_angVel = zeros(cdmp.rep_length,3);
rep_angAcc = zeros(cdmp.rep_length,3);
log_q      = ones(cdmp.rep_length,4) * 2 .* quatlog(quatnormalize(quatmultiply(cdmp.rep_goal(4:7),quatconj(rep_quat(1,:)))));

% Scaling factors
Dp = cdmp.rep_goal(1:3) - cdmp.rep_start(1:3);
Do = quatlog(quatnormalize(quatmultiply(cdmp.rep_goal(4:7),quatconj(cdmp.rep_start(4:7)))));


%% Translational DMP
        
% Obtained forcing function from weights
% fp = psi * wp * s / sum(psi)
cdmp.rep_fp = ((cdmp.rep_psi*cdmp.wp).*repmat(cdmp.rep_s,1,size(cdmp.wp,2))) ...
              ./ repmat((sum(cdmp.rep_psi,2)+1e-3),1,size(cdmp.wp,2));

% Integration of acceleration to velocity and position
for i=2:cdmp.rep_length
    rep_pos(i,:)    = rep_pos(i-1,:) + rep_linVel(i-1,:) * cdmp.dt;
    rep_linVel(i,:) = rep_linVel(i-1,:) + rep_linAcc(i-1,:) * cdmp.dt;
    rep_linAcc(i,:) = cdmp.K*(cdmp.rep_goal(1:3)-rep_pos(i,:)) / cdmp.rep_tau^2 ...
                     - cdmp.D*rep_linVel(i,:)      / cdmp.rep_tau   ...
                     - cdmp.K.*Dp.*cdmp.rep_s(i,:) / cdmp.rep_tau^2 ...
                     + cdmp.K.*cdmp.rep_fp(i,:)    / cdmp.rep_tau^2 ;
end
        
cdmp.rep_pos    = rep_pos;
cdmp.rep_linVel = rep_linVel;
cdmp.rep_linAcc = rep_linAcc;


%% Orientation DMP

% Obtained forcing function from weights
% fo = wo * psi(s) * s / sum(psi(s))
cdmp.rep_fo = ((cdmp.rep_psi*cdmp.wo).*repmat(cdmp.rep_s,1,size(cdmp.wo,2))) ...
             ./ repmat((sum(cdmp.rep_psi,2)+realmin),1,size(cdmp.wo,2));

% Integration of acceleration to velocity and position
% omega_dot = 1 / tau^2 * (K * 2log(g*conj(q)) - D * tau * omega - K * 2log(g*q0)* s  + fo)
for i=2:cdmp.rep_length
    log_q(i,:)      = 2 * quatlog( quatnormalize( quatmultiply(cdmp.rep_goal(4:7),quatconj(rep_quat(i-1,:)))) );
    rep_quat(i,:)   = quatmultiply( quatexp(cdmp.dt*[0 rep_angVel(i-1,:)]/2) , rep_quat(i-1,:));
    rep_angVel(i,:) = rep_angVel(i-1,:) + cdmp.dt * rep_angAcc(i-1,:);
    rep_angAcc(i,:) = cdmp.K * log_q(i,2:4)                     / cdmp.rep_tau^2 ...
                      - cdmp.D * rep_angVel(i-1,:)              / cdmp.rep_tau ...
                      - cdmp.K * 2 * Do(2:4) .* cdmp.rep_s(i,:) / cdmp.rep_tau^2 ...
                      + cdmp.rep_fo(i,:)                        / cdmp.rep_tau^2 ;
end

cdmp.rep_quat   = rep_quat;
cdmp.rep_angVel = rep_angVel;
cdmp.rep_angAcc = rep_angAcc;


%% ORDER CDMP FIELDS

cdmp = orderfields(cdmp);


end

