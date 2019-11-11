function [u_optimal] = QPcontroller(t,x)
global problem
x1 = x(1); %x_tk1
x2 = x(2); %x_tk2
x1_max = problem.x1_max;
x1_min = problem.x1_min;
x2_max = problem.x2_max;
x2_min = problem.x2_min;



x1_desired = problem.x1_desired;
K_b = problem.K_b;
P = problem.P;
H = problem.H;
epsilon = problem.epsilon;
%CLF
V = [(x1-x1_desired);x2]'*P*[(x1-x1_desired);x2]; %Control Lyapunov Function

%CBF
B1 = x1-x1_min; 
B2 = -x1+x1_max;
B3 = x2-x2_min;
B4 = -x2+x2_max;


% A = [2*x2 -1;-1 0;1 0;-1 0;1 0];
% b = [-2*x1*x2-epsilon*V;K_b(1)*B1+K_b(2)*x2;K_b(1)*B2-K_b(2)*x2;K_b(1)*B3;K_b(1)*B4];
%Modivied CLF constraint

A = [2*x2+(x1-x1_desired) 0;-1 0;1 0;-1 0;1 0];
b = [-x2*(2*(x1-x1_desired)+x2)-epsilon*V;K_b(1)*B1+K_b(2)*x2;K_b(1)*B2-K_b(2)*x2;K_b(1)*B3;K_b(1)*B4]; %exponential convergence constraint



% %Only CLF Constraint
%A = [2*x2+(x1-x1_desired) -1];
%b = [-2*x2*x1-x2^2-epsilon*V];

f = [0 0];
lb = -10;
ub = 10;

u_optimal = quadprog(H,f,A,b,[],[],[],[]);
end

