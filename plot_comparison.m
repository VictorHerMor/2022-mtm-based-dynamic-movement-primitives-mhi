clear
close all
format longG


%% LOAD cdmp-MTM MODEL DATA

load('cdmp_mtm_data/onemodelfitsall_10rbf.mat')
cdmp_mtm_onemodelfitsall_10rbf = cdmp_mtm;

load('cdmp_mtm_data/onemodelfitsall_200rbf.mat')
cdmp_mtm_onemodelfitsall_200rbf = cdmp_mtm;

load('cdmp_mtm_data/mtm_optimised.mat')
cdmp_mtm_mtm_optimised = cdmp_mtm;

clearvars cdmp_mtm


%% PLOT CARTESIAN POSE

figure

plotDemoTrajectory(cdmp_mtm_onemodelfitsall_10rbf,'b');

plotRepTrajectory(cdmp_mtm_onemodelfitsall_10rbf,'r');
plotRepTrajectory(cdmp_mtm_onemodelfitsall_200rbf,'m');
plotRepTrajectory(cdmp_mtm_mtm_optimised,'g');

subplot(2,4,4)
    legend({'demonstration data','onemodelfitsall 10rbf','onemodelfitsall 200rbf','mtm optimised'},'Location','south')
    

%% SUPPORTING FUNCTIONS

% Plot the demonstration data (works with any .mat file)
function [] = plotDemoTrajectory(cdmp,color)
    timestamps_reach    = (0:(length(cdmp.reach.demo_pos)-1))*cdmp.reach.dt;
    timestamps_grasp    = (0:(length(cdmp.grasp.demo_pos)-1))*cdmp.grasp.dt + (timestamps_reach(end)+cdmp.reach.dt);
    timestamps_move     = (0:(length(cdmp.move.demo_pos)-1))*cdmp.move.dt + (timestamps_grasp(end)+cdmp.grasp.dt);
    timestamps_position = (0:(length(cdmp.position.demo_pos)-1))*cdmp.position.dt + (timestamps_move(end)+cdmp.move.dt);
    timestamps_release  = (0:(length(cdmp.release.demo_pos)-1)) *cdmp.release.dt + (timestamps_position(end)+cdmp.position.dt);

    timespan = seconds([timestamps_reach, timestamps_grasp, timestamps_move, timestamps_position, timestamps_release]);
    transtimes_demo = [timestamps_reach(1), timestamps_reach(end), timestamps_grasp(end), timestamps_move(end), timestamps_position(end), timestamps_release(end)];
    
    titles = {'Pos x','Pos Y','Pos Z','Quat W','Quat X',' Quat Y','Quat Z'};
    
    for i=1:3
        subplot(2,4,i)
            hold on
            plot(timespan, [cdmp.reach.demo_pos(:,i); cdmp.grasp.demo_pos(:,i); cdmp.move.demo_pos(:,i); cdmp.position.demo_pos(:,i); cdmp.release.demo_pos(:,i)] ,'Color',color,'LineWidth',2)
            for j=1:length(transtimes_demo)
                xline(seconds(transtimes_demo(j)),append('--',color));
            end
            title(titles(i));
            hold off
    end
    
    subplot(2,4,4)
        plot3([cdmp.reach.demo_pos(:,1); cdmp.grasp.demo_pos(:,1); cdmp.move.demo_pos(:,1); cdmp.position.demo_pos(:,1); cdmp.release.demo_pos(:,1)],...
              [cdmp.reach.demo_pos(:,2); cdmp.grasp.demo_pos(:,2); cdmp.move.demo_pos(:,2); cdmp.position.demo_pos(:,2); cdmp.release.demo_pos(:,2)],...
              [cdmp.reach.demo_pos(:,3); cdmp.grasp.demo_pos(:,3); cdmp.move.demo_pos(:,3); cdmp.position.demo_pos(:,3); cdmp.release.demo_pos(:,3)],'Color',color,'LineWidth',2);
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        title('Translational Dimensions')
    
    for i=1:4
        subplot(2,4,i+4)
            hold on
            plot(timespan, [cdmp.reach.demo_quat(:,i); cdmp.grasp.demo_quat(:,i); cdmp.move.demo_quat(:,i); cdmp.position.demo_quat(:,i); cdmp.release.demo_quat(:,i)] ,'Color',color,'LineWidth',2)
            for j=1:length(transtimes_demo)
                xline(seconds(transtimes_demo(j)),append('--',color));
            end
            title(titles(i+3))
            hold off
    end
end

% Plot the reproduction data of the selected cdmp_mtm_model
function [] = plotRepTrajectory(cdmp,color) 
    timestamps_reach_rep    = (0:(length(cdmp.reach.rep_pos)-1))*cdmp.reach.dt;
    timestamps_grasp_rep    = (0:(length(cdmp.grasp.rep_pos)-1))*cdmp.grasp.dt + (timestamps_reach_rep(end)+cdmp.reach.dt);
    timestamps_move_rep     = (0:(length(cdmp.move.rep_pos)-1))*cdmp.move.dt + (timestamps_grasp_rep(end)+cdmp.grasp.dt);
    timestamps_position_rep = (0:(length(cdmp.position.rep_pos)-1))*cdmp.position.dt + (timestamps_move_rep(end)+cdmp.move.dt);
    timestamps_release_rep  = (0:(length(cdmp.release.rep_pos)-1))*cdmp.release.dt + (timestamps_position_rep(end)+cdmp.position.dt);

    timespan_rep = seconds([timestamps_reach_rep, timestamps_grasp_rep, timestamps_move_rep, timestamps_position_rep, timestamps_release_rep]);
    transtimes_rep = [timestamps_reach_rep(1), timestamps_reach_rep(end), timestamps_grasp_rep(end), timestamps_move_rep(end), timestamps_position_rep(end), timestamps_release_rep(end)];

    titles = {'Pos x','Pos Y','Pos Z','Quat W','Quat X',' Quat Y','Quat Z'};
    mtm_names = {'REACH','GRASP','MOVE','POSITION','RELEASE',''};
    
    for i=1:3
        subplot(2,4,i)
            hold on
            plot(timespan_rep, [cdmp.reach.rep_pos(:,i); cdmp.grasp.rep_pos(:,i); cdmp.move.rep_pos(:,i); cdmp.position.rep_pos(:,i); cdmp.release.rep_pos(:,i)] ,'Color',color,'LineWidth',1.5)
            for j=1:length(transtimes_rep)
                xline(seconds(transtimes_rep(j)),append('--',color),mtm_names(j));
            end
            title(titles(i));
            hold off
    end
    
    subplot(2,4,4)
        hold on
        plot3([cdmp.reach.rep_pos(:,1); cdmp.grasp.rep_pos(:,1); cdmp.move.rep_pos(:,1); cdmp.position.rep_pos(:,1); cdmp.release.rep_pos(:,1)],...
              [cdmp.reach.rep_pos(:,2); cdmp.grasp.rep_pos(:,2); cdmp.move.rep_pos(:,2); cdmp.position.rep_pos(:,2); cdmp.release.rep_pos(:,2)],...
              [cdmp.reach.rep_pos(:,3); cdmp.grasp.rep_pos(:,3); cdmp.move.rep_pos(:,3); cdmp.position.rep_pos(:,3); cdmp.release.rep_pos(:,3)],'Color',color,'LineWidth',1.5);
        view([225 20]);
        hold off

    for i=1:4
        subplot(2,4,i+4)
            hold on
            plot(timespan_rep, [cdmp.reach.rep_quat(:,i); cdmp.grasp.rep_quat(:,i); cdmp.move.rep_quat(:,i); cdmp.position.rep_quat(:,i); cdmp.release.rep_quat(:,i)] ,'Color',color,'LineWidth',1.5)
            for j=1:length(transtimes_rep)
                xline(seconds(transtimes_rep(j)),append('--',color),mtm_names(j));
            end
            title(titles(i+3));
            hold off
    end
end
