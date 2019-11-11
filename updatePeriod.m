function [tau_global,tau_eta] = updatePeriod(x_tk,u)
global problem
x1_max = problem.x1_max;
x1_min = problem.x1_min;
x2_max = problem.x2_max;
x2_min = problem.x2_min;
K_b = problem.K_b;
lipshitzConst = problem.lipshitzConst;
epsilon = problem.epsilon;
P = problem.P;
x1_d = problem.x1_desired;


x1_tk = x_tk(1);
x2_tk = x_tk(2);

%Determine tau_eta
D_1 = 2*epsilon*x2_min^2+(6*u(1)+3*epsilon*u(1))*x2_min+2*epsilon*u(1)*x1_min+(3+2*epsilon)*u(1)^2-2*epsilon*x1_d*u(1); %x1_min, x2_min
D_2 = 2*epsilon*x2_max^2+(6*u(1)+3*epsilon*u(1))*x2_max+2*epsilon*u(1)*x1_max+(3+2*epsilon)*u(1)^2-2*epsilon*x1_d*u(1); %x1_max, x2_max
D_3 = 2*epsilon*x2_max^2+(6*u(1)+3*epsilon*u(1))*x2_max+2*epsilon*u(1)*x1_min+(3+2*epsilon)*u(1)^2-2*epsilon*x1_d*u(1); %x1_min, x2_max
D_4 = 2*epsilon*x2_min^2+(6*u(1)+3*epsilon*u(1))*x2_min+2*epsilon*u(1)*x1_max+(3+2*epsilon)*u(1)^2-2*epsilon*x1_d*u(1); %x1_max, x2_min
D_max = max([D_1,D_2,D_3,D_4]);

eta_dot_t0 =(2*x2_tk+u(1)+epsilon*(2*(x1_tk-x1_d)+x2_tk))*x2_tk+(2*(x1_tk-x1_d)+2*x2_tk+2*u(1)+epsilon*(x1_tk-x1_d+2*x2_tk))*u(1);

tau_eta_1 = (-eta_dot_t0+sqrt(eta_dot_t0^2-4*D_max*eta_V(x1_tk,x2_tk,x1_d,u,epsilon)/2))/(D_max);
tau_eta_2 = (-eta_dot_t0-sqrt(eta_dot_t0^2-4*D_max*eta_V(x1_tk,x2_tk,x1_d,u,epsilon)/2))/(D_max);

tau_1 = fzero(@(t) zeta_B1_bound(t,lipshitzConst,x1_tk, x2_tk,x1_min,u(1),K_b),0.5);
tau_2 = fzero(@(t) zeta_B2_bound(t,lipshitzConst,x1_tk, x2_tk,x1_max,u(1),K_b),0.5);
tau_3 = zeta_B3(x2_tk,x2_min,u(1),K_b)/norm(K_b(1)*u(1)); %analytically determine root
tau_4 = zeta_B4(x2_tk,x2_max,u(1),K_b)/norm(K_b(1)*u(1)); %analytically determine root
tau_eta = max(tau_eta_1,tau_eta_2);

tau_global = min([tau_1,tau_2,tau_3,tau_4,tau_eta]);



end


function eta_bound = eta_V_bound(t,x1_tk,x2_tk,x1_d,u,epsilon,x1_min,x1_max,x2_min,x2_max)
 
 D_1 = 2*epsilon*x2_min^2+(6*u(1)+3*epsilon*u(1))*x2_min+2*u(1)*x1_min+(3+2*epsilon)*u(1)^2-2*epsilon*x1_d*u(1); %x1_min, x2_min
 D_2 = 2*epsilon*x2_max^2+(6*u(1)+3*epsilon*u(1))*x2_max+2*u(1)*x1_max+(3+2*epsilon)*u(1)^2-2*epsilon*x1_d*u(1); %x1_max, x2_max
 D_3 = 2*epsilon*x2_max^2+(6*u(1)+3*epsilon*u(1))*x2_max+2*u(1)*x1_min+(3+2*epsilon)*u(1)^2-2*epsilon*x1_d*u(1); %x1_min, x2_max
 D_4 = 2*epsilon*x2_min^2+(6*u(1)+3*epsilon*u(1))*x2_min+2*u(1)*x1_max+(3+2*epsilon)*u(1)^2-2*epsilon*x1_d*u(1); %x1_max, x2_min
 D_max = max([D_1,D_2,D_3,D_4]);
 
 eta_dot_t0 =(2*x2_tk+u(1)+epsilon*(2*(x1_tk-x1_d)+x2_tk))*x2_tk+(2*(x1_tk-x1_d)+2*x2_tk+2*u(1)+epsilon*(x1_tk-x1_d+2*x2_tk))*u(1);
 eta_bound = (D_max*t^2/2)+eta_dot_t0*t+eta_V(x1_tk,x2_tk,x1_d,u,epsilon);
end


function zeta_1_bound = zeta_B1_bound(t,lipshitzConst,x1_tk, x2_tk,x1_min,u,K_b)
    zeta_1_bound = (K_b(1)*(x2_tk-rBound([x1_tk x2_tk],t,lipshitzConst,u(1)))-K_b(2)*norm(u))*t + zeta_B1(x1_tk,x2_tk,x1_min,u(1),K_b);
end

function zeta_2_bound = zeta_B2_bound(t,lipshitzConst,x1_tk, x2_tk,x1_max,u,K_b)
    zeta_2_bound = (-K_b(1)*(x2_tk+rBound([x1_tk x2_tk],t,lipshitzConst,u(1)))-K_b(2)*norm(u))*t + zeta_B2(x1_tk,x2_tk,x1_max,u(1),K_b);
end

function zeta_3_bound = zeta_B3_bound(t,x2_tk,x2_min,u,K_b)
    zeta_3_bound = -norm(K_b(1)*u(1))*t + zeta_B3(x2_tk,x2_min,u(1),K_b);
end

function zeta_4_bound = zeta_B4_bound(t,x2_tk,x2_max,u,K_b)
    zeta_4_bound = -norm(K_b(1)*u(1))*t + zeta_B4(x2_tk,x2_max,u(1),K_b);
end

function zeta_1 = zeta_B1(x1,x2,x1_min,u,K_b)
    zeta_1 = K_b(2)*x2+K_b(1)*(x1-x1_min)+u;
end

function zeta_2 = zeta_B2(x1,x2,x1_max,u,K_b)
    zeta_2 = -K_b(2)*x2+K_b(1)*(-x1+x1_max)-u;
end

function zeta_3 = zeta_B3(x2,x2_min,u,K_b)
    zeta_3 = K_b(1)*(x2-x2_min)+u;
end

function zeta_4 = zeta_B4(x2,x2_max,u,K_b)
    zeta_4 = K_b(1)*(-x2+x2_max)-u;
end

function eta = eta_V(x1,x2,x1_d,u,epsilon)
% eta = (2*(x1-x1_d)*x2+x2^2)+(x1-x1_d+2*x2)*u(1)+epsilon*((x1-x1_d)^2+x2*(x1-x1_d)+x2^2)-u(2)-10;
 eta = (2*(x1-x1_d)*x2+x2^2)+(x1-x1_d+2*x2)*u(1)+epsilon*((x1-x1_d)^2+x2*(x1-x1_d)+x2^2)-u(2);
end


