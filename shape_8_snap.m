function [cost] = shape_8_snap(x_subs, x, snap_f, T)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%%
% cost = double(subs(snap_f, x, x_subs));
cost = integral(@(tt) snap_f(tt, x_subs), 0, T);
end

