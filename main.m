clear
close all


%% LOAD DATA

load('demo_data/ur5e_pickplace_experiment.mat');


%% Parameters

% % Comment / Uncomment desired Parameter settings % % 

% Parameters to replicate demonstration data with COMPLETE LOW ACCURACY
% settingname     = 'onemodelfitsall_10rbf';
% rbf_num_low     = 10;
% rbf_num_high    = 10;
% K               = 100;
% rbf_width       = 2;
% tau_scaler_fast = 1;
% tau_scaler_slow = 1;

% Parameters to replicate demonstration data with COMPLETE HIGH ACCURACY
% settingname     = 'onemodelfitsall_200rbf';
% rbf_num_low     = 200;
% rbf_num_high    = 200;
% K               = 100;
% rbf_width       = 2;
% tau_scaler_fast = 1;
% tau_scaler_slow = 1;

% Parameters to improve demonstration with RBF / TAU OPTIMISED
settingname     = 'mtm_optimised';
rbf_num_low     = 10;
rbf_num_high    = 200;
K               = 100;
rbf_width       = 2;
tau_scaler_fast = 0.5;
tau_scaler_slow = 1;


%% REACH

% tic

rep_start_reach = [demo_traj_reach.pos{1,:}, demo_traj_reach.quat{1,:}]; % + [0.03, 0.03, 0.03, 0, 0, 0, 0];
rep_goal_reach  = [demo_traj_reach.pos{end,:}, demo_traj_reach.quat{end,:}];

cdmp_reach = getCDMP(demo_traj_reach, 'rbf_num', rbf_num_low, ...
                    'tau_scaler', tau_scaler_fast, ...
                    'rep_start', rep_start_reach, ...
                    'rep_goal',  rep_goal_reach);


%% GRASP 

rep_start_grasp = [cdmp_reach.rep_pos(end,:), cdmp_reach.rep_quat(end,:)];
rep_goal_grasp  = [demo_traj_grasp.pos{end,:}, demo_traj_grasp.quat{end,:}];

cdmp_grasp = getCDMP(demo_traj_grasp, 'rbf_num', rbf_num_high, ...
                    'tau_scaler', tau_scaler_slow, ...
                    'rep_start', rep_start_grasp, ...
                    'rep_goal',  rep_goal_grasp);


%% MOVE 

rep_start_move = [cdmp_grasp.rep_pos(end,:), cdmp_grasp.rep_quat(end,:)];
rep_goal_move  = [demo_traj_move.pos{end,:},  demo_traj_move.quat{end,:}];

cdmp_move = getCDMP(demo_traj_move, 'rbf_num', rbf_num_low, ...
                    'tau_scaler', tau_scaler_fast, ...
                    'rep_start', rep_start_move, ...
                    'rep_goal',  rep_goal_move);


%% POSITION 

rep_start_position = [cdmp_move.rep_pos(end,:), cdmp_move.rep_quat(end,:)];
rep_goal_position  = [demo_traj_position.pos{end,:}, demo_traj_position.quat{end,:}];

cdmp_position = getCDMP(demo_traj_position, 'rbf_num', rbf_num_high, ...
                    'tau_scaler', tau_scaler_slow, ...
                    'rep_start', rep_start_position, ...
                    'rep_goal',  rep_goal_position);


%% RELEASE 

rep_start_release = [cdmp_position.rep_pos(end,:), cdmp_position.rep_quat(end,:)];
rep_goal_release  = [demo_traj_release.pos{end,:}, demo_traj_release.quat{end,:}];

cdmp_release = getCDMP(demo_traj_release, 'rbf_num', rbf_num_high, ...
                    'tau_scaler', tau_scaler_slow, ...
                    'rep_start', rep_start_release, ...
                    'rep_goal',  rep_goal_release);

% toc


%% STORE AS .MAT FILE

cdmp_mtm.reach = cdmp_reach;
cdmp_mtm.grasp = cdmp_grasp;
cdmp_mtm.move  = cdmp_move;
cdmp_mtm.position = cdmp_position;
cdmp_mtm.release = cdmp_release;
cdmp_mtm.mtmconstants = mtmconstants;

save(append('cdmp_mtm_data/',settingname),'cdmp_mtm');

fprintf("\n saved ! :) \n \n");


%% SUPPORTING FUNCTION 

function cdmp = getCDMP(demo_traj, cdmp)
    arguments
        demo_traj
        cdmp.K              {mustBeNumeric} = 100              % Spring Parameter of DS
        cdmp.rbf_num       {mustBePositive} = 100              % Number of RBFs
        cdmp.rbf_width     {mustBePositive} = 2                % Widths of RBFs
        cdmp.rep_start      {mustBeNumeric}
        cdmp.rep_start_vel  {mustBeNumeric} = zeros(1,6)
        cdmp.rep_goal       {mustBeNumeric}
        cdmp.rep_goal_vel   {mustBeNumeric} = zeros(1,6)
        cdmp.tau_scaler     {mustBeNumeric} = 1                % Temporal scaling factor for reproduction
    end
    
    cdmp.D            = NaN;                                   % Damper Parameter of DS            - default: critically damped
    cdmp.demo_alpha_s = NaN;                                   % Constant of Canonical System      - default: s reaches 0.001
    cdmp.rbf_mode     = 'equal_in_t';                          % Equal distribution of RBF centers - equal_in_s OR equal_in_t

    % Derived Parameters
    cdmp.demo_pos     = demo_traj.pos.Variables;
    cdmp.demo_linVel  = demo_traj.linVel.Variables;
    cdmp.demo_linAcc  = demo_traj.linAcc.Variables;

    cdmp.demo_quat    = demo_traj.quat.Variables;
    cdmp.demo_angVel  = demo_traj.angVel.Variables;
    cdmp.demo_angAcc  = demo_traj.angAcc.Variables;

    cdmp.demo_length  = size(demo_traj.pos,1);                 % Sample Quantity
    cdmp.samplerate   = demo_traj.pos.Properties.SampleRate;   % [Hz]

    cdmp.dt           = 1/cdmp.samplerate;                     % [s]
    cdmp.demo_tau     = cdmp.demo_length*cdmp.dt;              % Demonstration duration in [s]

    % Learn CDMP
    cdmp = learn_cdmp(cdmp);

    % Parameters for Reproduction
    cdmp.rep_alpha_s   = NaN;                                  % Constant of Canonical System      - default: s reaches 0.001
    if(~isfield(cdmp,'rep_tau'))
        cdmp.rep_tau       = cdmp.demo_tau * cdmp.tau_scaler;  % New Reproduction Duration in [s]  - default: demo_tau
    end

    % Reproduce CDMP
    cdmp = run_cdmp(cdmp);
    
end
