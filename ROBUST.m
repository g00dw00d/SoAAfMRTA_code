function [lower,upper] = ROBUST(c,W,E,upper_E,available_k)
lower = inf(size(E));
upper = inf(size(E));
upper(E == 1) = upper_E(E == 1);
for i = 1:size(E,1)
    for j = 1:size(E,2)
        B_ij = find(available_k(i,j,:) == 1);
        if ~isempty(B_ij)
            margins = zeros(size(B_ij));
            for k = 1:(length(B_ij) - 1)
                margins(k) = c(W{B_ij(k)} == 1) + upper_E(W{B_ij(k)} == 1);
            end
            if E(i,j) == 0            
                margins(end) = c(W{B_ij(end)} == 1) + upper_E(W{B_ij(end)} == 1);    
            end
            lower(i,j)  = c(i,j) - max(margins);
        end
    end
end


end