function stateTraj_plot(problem,conMode)
x1_max = problem.x1_max;
x1_min = problem.x1_min;
x2_max = problem.x2_max;
x2_min = problem.x2_min;

if conMode == 1
    %load data
    t_log_struct = load('./matfile_storage/fixed_t_log.mat');
    x_log_struct = load('./matfile_storage/fixed_x_log.mat');
    u_log_struct = load('./matfile_storage/fixed_u_log.mat');
    x_tk_log = load('./matfile_storage/fixed_x_tk_log.mat');
    t_update_log = load('./matfile_storage/fixed_t_update_log.mat');
    x1_min_log_struct = load('./matfile_storage/fixed_x1_min_log.mat');
    x1_max_log_struct = load('./matfile_storage/fixed_x1_max_log.mat');
    x2_min_log_struct = load('./matfile_storage/fixed_x2_min_log.mat');
    x2_max_log_struct = load('./matfile_storage/fixed_x2_max_log.mat');

    %Extract Vector
    x_log = x_log_struct(1).x_log;
    t_log = t_log_struct(1).t_log;
    u_log = u_log_struct(1).u_log;

    x1_min_log = x1_min_log_struct(1).x1_min_log;
    x1_max_log = x1_max_log_struct(1).x1_max_log;
    x2_min_log = x2_min_log_struct(1).x2_min_log;
    x2_max_log = x2_max_log_struct(1).x2_max_log;

    x1_update = x_tk_log(1).x_update(:,1);
    x2_update = x_tk_log(1).x_update(:,2);
    t_update = t_update_log(1).t_update;
    x_update = [x1_update,x2_update];

    % Plot x1, x2 vs time for Periodic Control
    figure('DefaultAxesFontSize',24)
    subplot(1,3,1)
    hold on
    plot(t_log,x_log(:,1),'LineWidth',2)
    plot(t_log,x1_min_log,'r--','LineWidth',2)
    plot(t_log,x1_max_log,'m:','LineWidth',2)
    plot(t_update,x_update(:,1),'bo','MarkerSize',10)
    legend('x_1 trajectory','x_{1,min}','x_{1,max}','Control Update')
    xlabel('t')
    ylabel('x1')
    title('Position x_1 over time')
    ylim([x1_min-2,x1_max+2])
    subplot(1,3,2)
    hold on
    plot(t_log,x_log(:,2),'LineWidth',2)
    plot(t_log,x2_min_log,'r--','LineWidth',2)
    plot(t_log,x2_max_log,'m:','LineWidth',2)
    plot(t_update,x_update(:,2),'bo','MarkerSize',10)
    legend('x_2 trajectory','x_{2,min}','x_{2,max}','Control Update')
    xlabel('t')
    ylabel('x2')
    title('Velocity x_2 over time')
    ylim([x2_min-2,x2_max+2])
    subplot(1,3,3)
    hold on
    plot(t_log,u_log,'LineWidth',2)
    legend('Fixed Time Control')
    xlabel('t')
    ylabel('u')
    title('Control u over time')

elseif conMode == 2
    %load data
    t_log_struct = load('./matfile_storage/selfTriggered_t_log.mat');
    x_log_struct = load('./matfile_storage/selfTriggered_x_log.mat');
    u_log_struct = load('./matfile_storage/selfTriggered_u_log.mat');
    
    x_tk_log = load('./matfile_storage/selfTriggered_x_tk_log.mat');
    t_update_struct = load('./matfile_storage/selfTriggered_t_update.mat');
    x1_min_log_struct = load('./matfile_storage/selfTriggered_x1_min_log.mat');
    x1_max_log_struct = load('./matfile_storage/selfTriggered_x1_max_log.mat');
    x2_min_log_struct = load('./matfile_storage/selfTriggered_x2_min_log.mat');
    x2_max_log_struct = load('./matfile_storage/selfTriggered_x2_max_log.mat');
    
    %Extract vector from data
    x_log = x_log_struct(1).x_log;
    t_log = t_log_struct(1).t_log;
    u_log = u_log_struct(1).u_log;

    x1_min_log = x1_min_log_struct(1).x1_min_log;
    x1_max_log = x1_max_log_struct(1).x1_max_log;
    x2_min_log = x2_min_log_struct(1).x2_min_log;
    x2_max_log = x2_max_log_struct(1).x2_max_log;

    x1_update = x_tk_log(1).x_update(:,1);
    x2_update = x_tk_log(1).x_update(:,2);
    t_update = t_update_struct(1).t_update;
    x_update = [x1_update,x2_update];
    

    
    % Plot x1, x2 vs time for Self-triggered Control
    figure('DefaultAxesFontSize',24)
    subplot(1,3,1)
    hold on
    plot(t_log,x_log(:,1),'LineWidth',2)
    plot(t_log,x1_min_log,'r--','LineWidth',2)
    plot(t_log,x1_max_log,'m:','LineWidth',2)
    plot(t_update,x_update(:,1),'bo','MarkerSize',10)
    legend('x_1 trajectory','x_{1,min}','x_{1,max}','Control Update')
    xlabel('t')
    ylabel('x1')
    title('Position x_1 over time')
    ylim([x1_min-2,x1_max+2])
    subplot(1,3,2)
    hold on
    plot(t_log,x_log(:,2),'LineWidth',2)
    plot(t_log,x2_min_log,'r--','LineWidth',2)
    plot(t_log,x2_max_log,'m:','LineWidth',2)
    plot(t_update,x_update(:,2),'bo','MarkerSize',10)
    legend('x_2 trajectory','x_{2,min}','x_{2,max}','Control Update')
    xlabel('t')
    ylabel('x2')
    title('Velocity x_2 over time')
    ylim([x2_min-2,x2_max+2])
    subplot(1,3,3)
    hold on
    plot(t_log,u_log,'LineWidth',2)
    legend('Self-Triggered Control')
    xlabel('t')
    ylabel('u')
    title('Control u over time')

end

end