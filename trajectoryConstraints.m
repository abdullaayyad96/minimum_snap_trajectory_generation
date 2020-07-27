function [c,ceq] = trajectoryConstraints(p, t_forced_roll, forced_roll, ts, n_order)
%TRAJECTORYCONSTRAINTS Summary of this function goes here
%   Detailed explanation goes here

[c1, ceq1] = attitude_contraint(p, t_forced_roll, forced_roll, ts, n_order);

c = c1;%, c2];%, c2, c3];
ceq = ceq1;%, ceq2];%, ceq2, ceq3];

%[c, ceq] = window_constraint(p, attitude_time_step, ts, n_order, 'x', 0.2);

end

