function [c, ceq] = attitude_contraint(p, t_forced_roll, forced_roll, ts, n_order)
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
for i=1:length(ts)-1
    if (t_forced_roll >= ts(i)) && (t_forced_roll <= ts(i+1))
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
g = 9.81;
ax = polys_vals_cell(polys_x,ts,t_forced_roll,2);
ay = polys_vals_cell(polys_y,ts,t_forced_roll,2);
az = polys_vals_cell(polys_z,ts,t_forced_roll,2);

roll = -atan2(ay, az + 9.81);

c = [];
ceq = [ax, roll - forced_roll];

end

