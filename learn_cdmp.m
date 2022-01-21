function cdmp = learn_cdmp(cdmp)

%% INITIALISE PARAMETERS

% Transformation System Parameters
if isnan(cdmp.D)
    cdmp.D        = 2*sqrt(cdmp.K);
else 
    cdmp.D        = params.D;
end
cdmp.alpha_z      = cdmp.D;
cdmp.beta_z       = cdmp.alpha_z/4;

% Canonical System Parameters
if isnan(cdmp.demo_alpha_s)
    % s reaches 0.001 
    cdmp.demo_alpha_s = - log(0.001) * cdmp.demo_tau / (cdmp.demo_length * cdmp.dt); 
end

% Initialise Start/Goal Positions
cdmp.demo_start     = [cdmp.demo_pos(1,:)   , cdmp.demo_quat(1,:)];
cdmp.demo_start_vel = zeros(1,6);
cdmp.demo_goal      = [cdmp.demo_pos(end,:) , cdmp.demo_quat(end,:)];
cdmp.demo_goal_vel  = zeros(1,6);

% Initialise Canonical System and Radial Basis Functions
cdmp = init_fs(cdmp,'demo_');

% Scaling factors
Dp = cdmp.demo_pos(end,:) - cdmp.demo_pos(1,:);
Do = quatlog(quatnormalize(quatmultiply(cdmp.demo_quat(end,:),quatconj(cdmp.demo_quat(1,:)))));

% Error defined by Koutras2019
e_q = 2 * quatlog(quatnormalize(quatmultiply(cdmp.demo_quat(end,:),quatconj(cdmp.demo_quat))));


%% Translational DMP
        
% Calculate forcing function from translation data
cdmp.demo_fp = 1/cdmp.K * (cdmp.demo_tau^2*cdmp.demo_linAcc ...
             + cdmp.D*cdmp.demo_tau*cdmp.demo_linVel ...
             + cdmp.K.*Dp.*cdmp.demo_s ...
             - cdmp.K*(cdmp.demo_pos(end,:)-cdmp.demo_pos));

% Linear Least Squares to obtain DMP weights
% Aw = f for each dimension with A = (g-p0)* psi / sum(psi) * s
wp = zeros(size(cdmp.demo_psi,2),size(cdmp.demo_pos,2));
for i = 1:size(cdmp.demo_pos,2)
    A = cdmp.demo_psi./sum(cdmp.demo_psi,2).*cdmp.demo_s;
    wp(:,i) = A\cdmp.demo_fp(:,i);
end    

cdmp.wp = wp;


%% Orientation DMP

% Calculate forcing function from rotation data
% fo = D * sum(w_i * psi_i(s)) / sum(psi_i) * s
% fo = tau^2 * omega_dot + D * tau * omega + K * 2log(g*q0)* s - K * 2log(g*conj(q))
cdmp.demo_fo = cdmp.demo_tau^2 * cdmp.demo_angAcc ...
             + cdmp.D * cdmp.demo_tau * cdmp.demo_angVel ...
             + cdmp.K * 2 * Do(2:4) .* cdmp.demo_s ...
             - cdmp.K * e_q(:,2:4);

% Linear Least Squares to obtain DMP weights
% Aw = f for each dimension with A = (g-q0)* psi / sum(psi) * s
wo = zeros(size(cdmp.demo_psi,2),size(cdmp.demo_fo,2));
for i = 1:3
    A = cdmp.demo_psi./sum(cdmp.demo_psi,2).*cdmp.demo_s;
    wo(:,i) = A\cdmp.demo_fo(:,i);
end

cdmp.wo = wo;


%% ORDER CDMP FIELDS

cdmp = orderfields(cdmp);


end

