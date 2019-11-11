function dxdt = sys_dynamics(t,x,u)
dxdt= zeros(2,1);
dxdt(1) = x(2);
dxdt(2) = u;
end