clc; clear all; 
syms l(t) l_d(t) chi(t) chi_d(t) chi_dd l_dd m g real;

assume(l(t), 'real');
assume(l_d(t), 'real');
assume(chi(t), 'real');
assume(chi_d(t), 'real');

r = [l*sin(chi); l*cos(chi)];
r_d = [(l_d*sin(chi) + l*cos(chi)*chi_d); (l_d*cos(chi) - l*cos(chi)*chi_d)];


T = (1/2)* m * (r_d.'*r_d);
V = m*g*[0 1]*r;

L = T - V;


eom = [

diff(diff(L, chi_d), t) - diff(L, chi) == 0;
diff(diff(L, l_d), t) - diff(L, l) == 0;

]

eom = subs(eom, diff(chi_d(t), t), chi_dd);
eom = subs(eom, diff(chi(t), t), chi_d);
eom = subs(eom, diff(l_d(t), t), l_dd);
eom = subs(eom, diff(l(t), t), l_d);

t = solve(eom, [chi_dd, l_dd]);

simplify(t.chi_dd)