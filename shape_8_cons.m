function [c, c_eq] = shape_8_cons(x_subs, x, t, polynomial, pos_x, vel_x, acc_x, jerk_x, pos_y, vel_y, acc_y, jerk_y, T)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

polynomial_constraints = [];
pose_constraints = [];
vel_contraints = [];
acc_contraints = [];
jerk_contraints = [];
%%
%constraints 
polynomial_constraints = [polynomial_constraints; polynomial(0, x_subs) + pi/2];
pose_constraints = [pose_constraints; pos_x(0, x_subs)];
vel_contraints = [vel_contraints; vel_x(0, x_subs)];
acc_contraints = [acc_contraints; acc_x(0, x_subs)];
jerk_contraints = [jerk_contraints; jerk_x(0, x_subs)];
pose_constraints = [pose_constraints; pos_y(0, x_subs)];
vel_contraints = [vel_contraints; vel_y(0, x_subs)];
acc_contraints = [acc_contraints; acc_y(0, x_subs)];
jerk_contraints = [jerk_contraints; jerk_y(0, x_subs)];

polynomial_constraints = [polynomial_constraints; polynomial(T, x_subs) - 3*pi/2];
pose_constraints = [pose_constraints; pos_x(T, x_subs)];
vel_contraints = [vel_contraints; vel_x(T, x_subs)];
acc_contraints = [acc_contraints; acc_x(T, x_subs)];
jerk_contraints = [jerk_contraints; jerk_x(T, x_subs)];
pose_constraints = [pose_constraints; pos_y(T, x_subs)];
vel_contraints = [vel_contraints; vel_y(T, x_subs)];
acc_contraints = [acc_contraints; acc_y(T, x_subs)];
jerk_contraints = [jerk_contraints; jerk_y(T, x_subs)];

c = [];
c_eq = double([polynomial_constraints; pose_constraints; vel_contraints; acc_contraints]);
end

