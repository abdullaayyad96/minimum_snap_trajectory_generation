function [c, ceq] = window_constraint(p, window_time_step, ts, n_order, window_direction, window_duration)
%ATTITUDE_CONTRAINT Summary of this function goes here
%   - p: polynomial parameters for all trajectory segments of x y and z
%   - time_step: time_step to impose attitude contraint
%   - ts: time stamps of all the start/end of each polynomial segment
%   - n_order: list containing the order of polynomial for each channel
%   - window_direction: 'x' or 'y'

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

%compute velocities and accelerations at desired time
vx = polys_vals_cell(polys_x,ts,window_time_step,1);
vy = polys_vals_cell(polys_y,ts,window_time_step,1);
vz = polys_vals_cell(polys_z,ts,window_time_step,1);
ax = polys_vals_cell(polys_x,ts,window_time_step,2);
ay = polys_vals_cell(polys_y,ts,window_time_step,2);
az = polys_vals_cell(polys_z,ts,window_time_step,2);
g = 9.81;

%values during window duration
window_timesteps = (window_time_step - window_duration/2): 0.1 : (window_time_step + window_duration/2);
vx_vals = polys_vals_cell(polys_x,ts,window_timesteps,1);
vy_vals = polys_vals_cell(polys_y,ts,window_timesteps,1);
vz_vals = polys_vals_cell(polys_z,ts,window_timesteps,1);
ax_vals = polys_vals_cell(polys_x,ts,window_timesteps,2);
ay_vals = polys_vals_cell(polys_y,ts,window_timesteps,2);
az_vals = polys_vals_cell(polys_z,ts,window_timesteps,2);

if window_direction=='x'
    x_desired_vel = 1;
    y_desired_vel = 0;
else
    x_desired_vel = 0;
    y_desired_vel = 1;    
end


c = [ax_vals - 0.1, ay_vals - 0.1 , az_vals + g - 0.1];
ceq = [vx_vals - x_desired_vel * vx_vals, vy_vals - y_desired_vel * vy_vals] ;

end

