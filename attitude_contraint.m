function [c, ceq] = attitude_contraint(p, time_step, ts, n_order, channel_contraint)
%ATTITUDE_CONTRAINT Summary of this function goes here
%   - p: polynomial parameters for all trajectory segments of x y and z
%   - time_step: time_step to impose attitude contraint
%   - ts: time stamps of all the start/end of each polynomial segment
%   - n_order: list containing the order of polynomial for each channel
%   - channel_contraint: 'pitch' or 'roll'

n_poly = length(ts)-1;
n_coef = cumsum(n_order+1);

%determine appropriate polynomial semgent
trajectory_segment = 0;
for i=1:length(ts)
    if (time_step > ts(i)) && (time_step < ts(i+1))
        trajectory_segment = i;
        break
    end
end

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

%compute accelerations at desired time
ax = polys_vals_cell(polys_x,ts,time_step,2);
ay = polys_vals_cell(polys_y,ts,time_step,2);
az = polys_vals_cell(polys_z,ts,time_step,2);
g = 9.81;

attitude_vector = [ax, ay, az+g] / norm([ax, ay, az+g]);

desired_attitude_vector = [0, 0, 0];
if channel_contraint == 'pitch'
    desired_attitude_vector = [0, 0.7071, 0.7071];
else
    desired_attitude_vector = [1, 0, 0];
end

c = norm([ax, ay, az+g]) - 10;%sumsqr(attitude_vector - desired_attitude_vector) - 0.0384;
%ceq = attitude_vector - desired_attitude_vector;
ceq = az + g

end

