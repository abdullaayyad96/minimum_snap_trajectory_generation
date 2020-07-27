%The value of a polynomial at times tt
function vals = polys_vals_cell(polys,ts,tt,r)
idx = 1;
N = length(tt);
vals = zeros(1,N);
for i = 1:N
    t = tt(i);
    if t<ts(idx)
        vals(i) = 0;
    else
        while idx<length(ts) && t>ts(idx+1)
            idx = idx+1;
        end
        vals(i) = poly_val(polys{idx},t,r);
%         vals(i) = poly_val(polys{1}(:, idx),t,r);
    end
end
end

