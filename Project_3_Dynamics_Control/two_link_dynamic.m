function dx = two_link_dynamic(t, x, params)
%define the q and dq values:
q1 = x(1);
q2 = x(2);
dq1 =  x(3);
dq2 =  x(4);

% Extract symbolic expressions
D_sym  = params.D_sym;
C_dq_sym = params.C_dq_sym;
G_sym  = params.G_sym;

% Substitute numerical values
sym_vars_full = {'q1','q2','dq1','dq2','m1','m2','l1','lc1','l2','lc2','I1','I2','g','tau1','tau2'};
vals_full     = { q1,  q2,  dq1,  dq2,  params.m1, params.m2, ...
                  params.l1, params.lc1, params.l2, params.lc2, ...
                  params.I1, params.I2, params.g, params.tau(1), params.tau(2) };

sym_vars_pos  = {'q1','q2','m1','m2','l1','lc1','l2','lc2','I1','I2','g','tau1','tau2'};
vals_pos      = { q1,  q2,  params.m1, params.m2, params.l1, params.lc1, ...
                  params.l2, params.lc2, params.I1, params.I2, params.g, ...
                  params.tau(1), params.tau(2) };

D_num = double(subs(D_sym,    sym_vars_full, vals_full));
C_num = double(subs(C_dq_sym, sym_vars_full, vals_full));
G_num = double(subs(G_sym,    sym_vars_pos,  vals_pos));



% Control torque
tau = params.tau;

% Compute accelerations
ddq =  D_num \ ( tau - C_num - G_num );

dx = [dq1; dq2; ddq(1); ddq(2)];;
end