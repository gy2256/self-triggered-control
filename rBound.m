function [trajBoundRadius] = rBound(x_tk,t,lipShtizConst,u)
r0 = norm([x_tk(2);u])/lipShtizConst;
trajBoundRadius = r0*exp(lipShtizConst*t)-r0;
end

