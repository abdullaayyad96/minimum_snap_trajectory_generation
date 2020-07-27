function [c, ceq] = jerk_constraints(p, time_steps, ts, n_order, max_jerk)
%ATTITUDE_CONTRAINT Summary of this function goes here
%   - p: polynomial parameters for all trajectory segments of x y and z
%   - time_step: time_step to impose attitude contraint
%   - ts: time stamps of all the start/end of each polynomial segment
%   - n_order: list containing the order of polynomial for each channel
%   - channel_contraint: 'pitch' or 'roll'

n_poly = length(ts)-1;
n_coef = cumsum(n_order+1);

%divide polynomial parameters to x y and z
polys = {};
for j=1:3 
    offset = (j-1) * n_coef(end);
    for i=1:n_poly
        if i<2
            start_index=1;
        else
            start_index = n_coef(i-1)+1;
        end
        polys{j, i} = p((offset + start_index): (offset + n_coef(i)));
    end
end
polys_x = {polys{1, :}};
polys_y = {polys{2, :}};
polys_z = {polys{3, :}};

%compute z jerk at desired time
jz = polys_vals_cell(polys_z,ts,time_steps,3);

c = abs(jz) - max_jerk * ones(size(jz));%sumsqr(attitude_vector - desired_attitude_vector) - 0.0384;
%ceq = attitude_vector - desired_attitude_vector;
ceq = [];

end

