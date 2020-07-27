function [c,ceq] = shape_constraints(p, ts, n_order)
%TRAJECTORYCONSTRAINTS Summary of this function goes here
%   Detailed explanation goes here

ts_enforce = linspace(0, ts(end), 10);

px = [];
py = [];

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

for i=1:length(ts_enforce)
    time_slot = ts_enforce(i);
    
    px = [px; polys_vals_cell(polys_x,ts,time_slot,0)];
    py = [py; polys_vals_cell(polys_y,ts,time_slot,0)];
end

c = [];
ceq = py.^4 - py.^2 + px.^2;

end

