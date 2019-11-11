
function clf_plot(problem,conMode)

if conMode == 1 %periodic
    x_tk_log = load('./matfile_storage/fixed_x_tk_log.mat');
    u_tk_log = load('./matfile_storage/fixed_u_tk_log.mat');
    d_tk_log = load('./matfile_storage/fixed_d_tk_log.mat');
    tauFix = 0.5;
    
    %Data Prossing, convert struc to vectors
    x1_tk_sim = x_tk_log(1).x_update(:,1);
    x2_tk_sim = x_tk_log(1).x_update(:,2);
    u_sim = u_tk_log(1).u_update;
    d_sim = d_tk_log(1).d_update;


    % Acquire Additonal paramters
    x1_d = problem.x1_desired;
    epsilon = problem.epsilon;
    k = 1:size(u_sim); %number of steps
    eta_value = zeros(length(u_sim),1);


    for i=k
        eta_value(i,1) = clf_function(tauFix,epsilon,x1_d,x1_tk_sim(i),x2_tk_sim(i),u_sim(i),d_sim(i));
    end

    figure('DefaultAxesFontSize',24)
    plot(k,eta_value,'*','MarkerSize',10)
    ylim([-10 2])
    xlabel('k-th step')
    ylabel('\eta(t = \tau)')
    
elseif conMode == 2 %self-triggered
    x_tk_log = load('./matfile_storage/selfTriggered_x_tk_log.mat');
    u_tk_log = load('./matfile_storage/selfTriggered_u_tk_log.mat');
    d_tk_log = load('./matfile_storage/selfTriggered_d_tk_log.mat');
    tauVari_log = load('./matfile_storage/tau_clf_cbf_log.mat');
    tauVari = tauVari_log(1).tau_clf_cbf_log;
    
     %Data Prossing, convert struc to vectors
    x1_tk_sim = x_tk_log(1).x_update(:,1);
    x2_tk_sim = x_tk_log(1).x_update(:,2);
    u_sim = u_tk_log(1).u_update;
    d_sim = d_tk_log(1).d_update;

    % Acquire Additonal paramters
    x1_d = problem.x1_desired;
    epsilon = problem.epsilon;
    k = 1:size(u_sim); %number of steps
    eta_value = zeros(length(u_sim),1);


    for i=k
        eta_value(i,1) = clf_function(tauVari(i),epsilon,x1_d,x1_tk_sim(i),x2_tk_sim(i),u_sim(i),d_sim(i));
    end

    figure('DefaultAxesFontSize',24)
    plot(k,eta_value,'*','MarkerSize',10)
    ylim([-10 2])
    xlabel('k-th step')
    ylabel('\eta(t = \tau)')
    
    
end


    
end

function eta_sim = clf_function(tau,epsilon,x1_d,x1_tk_sim,x2_tk_sim,u_sim,d_sim)
[tOde,xOde]=ode45(@(t,x) sys_dynamics(t,[x1_tk_sim;x2_tk_sim],u_sim),[0 tau],[x1_tk_sim;x2_tk_sim]);

eta_sim = eta_V(xOde(end,1),xOde(end,2),x1_d,u_sim,d_sim,epsilon); %Calculate eta at the end of the updating period

% eta_sim =
% eta_V(x1_tk_sim(k),x2_tk_sim(k),x1_d,u_sim(k),d_sim(k),epsilon); %for
% plotting margin
end

function eta = eta_V(x1,x2,x1_d,u,d,epsilon)
 eta = (2*(x1-x1_d)*x2+x2^2)+(x1-x1_d+2*x2)*u+epsilon*((x1-x1_d)^2+x2*(x1-x1_d)+x2^2)-d;
end
