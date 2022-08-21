function upper_E = MINMID(c,W,E,available_k)
upper_E = inf(size(c));
level = zeros(size(c));
n = size(c,2);
m = size(c,1) - n; 


[Ito,Ifrom] = meshgrid(1:n,1:(m + n)); 

c_selected = c(E == 1);
Ito_selected = Ito(E == 1);
Ifrom_selected = Ifrom(E == 1);

[~,Ia] = sort(c_selected,'descend');

M = zeros(m + n,n);
for idx = 1:length(c_selected)
    k = find(available_k(Ifrom_selected(Ia(idx)),Ito_selected(Ia(idx)),:) == 1,1,'last');
    AvailThisRound = available_k(:,:,k);
    IE = find((AvailThisRound == 1) & W{k} == 0);
    upper_E(W{k} == 1) = min(max(level(IE),(c(W{k} == 1) + c(IE)) / 2) - c(W{k} == 1));
    level(IE) = max(level(IE),c(W{k} == 1) + upper_E(W{k} == 1));     
end