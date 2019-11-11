%% Problem Setup
clear all; close all;clc
global problem
problem.x1_min=-10;
problem.x1_max=10;
problem.x2_min=-10;
problem.x2_max=10;
problem.x1_desired=5;
problem.x_tk=[-4;-3];
problem.P = [1 0.5;0.5 1]; %CLF weighting matrix
problem.epsilon = 0.6; %CLF Convergence Rate Term
problem.H = [1 0;0 1]; %Cost Function Wieghting Matrix
problem.lipshitzConst= 1;

%CBF Pole Selection
Ab=[zeros(1,1) eye(1);zeros(1,2)];
Bb=[zeros(1,1);1];
pb=-0.5*[20;21];
problem.K_b=place(Ab,Bb,pb);

%% Self-Triggered Controller with CLF-CBF

x1_tk = problem.x_tk(1);
x2_tk = problem.x_tk(2);
x1_max = problem.x1_max;
x1_min = problem.x1_min;
x2_max = problem.x2_max;
x2_min = problem.x2_min;
K_b = problem.K_b;
lipshitzConst = problem.lipshitzConst;

x_tk=[x1_tk x2_tk];


%Preallocate a size because matlab cannot take A(end) for A = []
x_log = [0,0];
t_log = [0];
u_log = [];
d_log = [];
tau_clf_cbf_log = [];
control_update_index = []; %record indices where the self-triggered control updates

for k = 1:25 %update control 12 times
    u = QPcontroller(0,x_tk); %Calculate optimal u at current timeStep and let the system runs
    [tau_global,tau_eta]= updatePeriod(x_tk,u);
    tau_clf_cbf_log = [tau_clf_cbf_log;tau_global];
    tspan = [0 tau_global]; % Simulation Length
    [t,x] = ode45(@(t,x) sys_dynamics(t,x,u(1)), tspan, x_tk);
    
    plot(x(:,1),x(:,2),'LineWidth',2)
    t_log = [t_log;t_log(end)+t];
    x_log = [x_log;x];
    u_log = [u_log;u(1)*ones(length(t),1)];
    d_log = [d_log;u(2)*ones(length(t),1)];

    control_update_index = [control_update_index;length(t_log)+1]; %the timestep index where control needs to be updated
    hold on
    xlim([x1_min,x1_max])
    ylim([x2_min,x2_max])
    xlabel('x_1')
    ylabel('x_2')
    x_tk = [x(end,1) x(end,2)];
end


% Array processing
t_log = t_log(2:end);
x_log = x_log(2:end,:); 
control_update_index = control_update_index-1;
control_update_index(end) = [];
control_update_index = [1;control_update_index];


t_update = t_log(control_update_index);
x_update = x_log(control_update_index,:);
u_update = u_log(control_update_index,:);
d_update = d_log(control_update_index,:);


x1_min_log = x1_min*ones(length(t_log),1);
x1_max_log = x1_max*ones(length(t_log),1);
x2_min_log = x2_min*ones(length(t_log),1);
x2_max_log = x2_max*ones(length(t_log),1);


%Save Parameters including initial state
x_update(1,1) = problem.x_tk(1);
x_update(1,2) = problem.x_tk(2);

save('./matfile_storage/tau_clf_cbf_log.mat','tau_clf_cbf_log');
save('./matfile_storage/selfTriggered_x_tk_log.mat','x_update');
save('./matfile_storage/selfTriggered_u_tk_log.mat','u_update');
save('./matfile_storage/selfTriggered_d_tk_log.mat','d_update');
save('./matfile_storage/selfTriggered_t_update.mat','t_update')
save('./matfile_storage/selfTriggered_x_log.mat','x_log');
save('./matfile_storage/selfTriggered_t_log.mat','t_log');
save('./matfile_storage/selfTriggered_u_log.mat','u_log');
save('./matfile_storage/selfTriggered_x1_min_log.mat','x1_min_log');
save('./matfile_storage/selfTriggered_x1_max_log.mat','x1_max_log');
save('./matfile_storage/selfTriggered_x2_min_log.mat','x2_min_log');
save('./matfile_storage/selfTriggered_x2_max_log.mat','x2_max_log');

%% Fixed Time Step CLF-CBF Controller
x1_tk = problem.x_tk(1);
x2_tk = problem.x_tk(2);
x1_max = problem.x1_max;
x1_min = problem.x1_min;
x2_max = problem.x2_max;
x2_min = problem.x2_min;
K_b = problem.K_b;
lipshitzConst = problem.lipshitzConst;

x_tk=[x1_tk x2_tk];
x_log = [0,0];
t_log = [0];
u_log = [];
d_log = [];
control_update_index = [];


for i=1:40
    u = QPcontroller(0,x_tk); %Calculate optimal u at current timeStep and let the system runs
    
    tspan = [0 0.5]; % Simulation Length
    [t,x] = ode45(@(t,x) sys_dynamics(t,x,u(1)), tspan, x_tk);    
    plot(x(:,1),x(:,2),'LineWidth',2)
    t_log = [t_log;t_log(end)+t];
    x_log = [x_log;x];
    u_log = [u_log;u(1)*ones(length(t),1)];
    d_log = [d_log;u(2)*ones(length(t),1)];
    control_update_index = [control_update_index;length(t_log)+1]; %the timestep index where control needs to be updated
    hold on
    xlim([x1_min,x1_max])
    ylim([x2_min,x2_max])
    xlabel('x_1')
    ylabel('x_2')
    x_tk = [x(end,1) x(end,2)];

end

% Array processing
t_log = t_log(2:end);
x_log = x_log(2:end,:);
control_update_index = control_update_index-1;
control_update_index(end) = [];
control_update_index = [1;control_update_index];

t_update = t_log(control_update_index);
x_update = x_log(control_update_index,:);
u_update = u_log(control_update_index,:);
d_update = d_log(control_update_index,:);


x1_min_log = x1_min*ones(length(t_log),1);
x1_max_log = x1_max*ones(length(t_log),1);
x2_min_log = x2_min*ones(length(t_log),1);
x2_max_log = x2_max*ones(length(t_log),1);


%Save Parameters including initial state
x_update(1,1) = problem.x_tk(1);
x_update(1,2) = problem.x_tk(2);

save('./matfile_storage/fixed_x_tk_log.mat','x_update');
save('./matfile_storage/fixed_u_tk_log.mat','u_update');
save('./matfile_storage/fixed_d_tk_log.mat','d_update');
save('./matfile_storage/fixed_t_update_log.mat','t_update');
save('./matfile_storage/fixed_x_log.mat','x_log');
save('./matfile_storage/fixed_t_log.mat','t_log');
save('./matfile_storage/fixed_u_log.mat','u_log');
save('./matfile_storage/fixed_x1_min_log.mat','x1_min_log');
save('./matfile_storage/fixed_x1_max_log.mat','x1_max_log');
save('./matfile_storage/fixed_x2_min_log.mat','x2_min_log');
save('./matfile_storage/fixed_x2_max_log.mat','x2_max_log');

