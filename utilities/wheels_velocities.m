function [vR,vL] = wheels_velocities(u1,u2,d)

syms vr vl

eqn1 = (vr-vl)/(2*d) == u2;
eqn2 = (vr+vl)/2 == u1;

sol = solve([eqn1, eqn2], [vr, vl]);
vR = double(sol.vr);
vL = double(sol.vl);

