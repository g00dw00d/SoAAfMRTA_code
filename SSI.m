function [A_r,W,E,U,available_k] = SSI(c_T0,c_TT)
%%
[m,n] = size(c_T0);

c = [c_T0;c_TT];

% bigM = 10 * max(max(max(c_T0)),max(max(c_TT)));

%assiged tasks
A = zeros(1,n);

%selected edges
E = zeros(m + n,n);


%
A_r = zeros(m,n);


%%
W = cell(m,1);
U = cell(m,1);

available_now = zeros(m + n,n);
available_k = zeros(m + n,n,n);
% B_e = zeros(m + n,n,n); 

[Ito,Ifrom] = meshgrid(1:n,1:(m + n)); 
%
%% first round candidate edges
available_now(1:m,:) = 1; 


for AuctionRoundCounter = 1:n

    %% Auction
    available_k(:,:,AuctionRoundCounter) = available_now;
    
    c_now = c(available_now == 1);
    Ito_now = Ito(available_now == 1);
    Ifrom_now = Ifrom(available_now == 1);
    [~,w_ind] = min(c_now);

    w_to = Ito_now(w_ind);
    w_from = Ifrom_now(w_ind);

    W{AuctionRoundCounter} = zeros(m + n,n);
    W{AuctionRoundCounter}(w_from,w_to) = 1;

    A(w_to) = 1;
    E(w_from,w_to) = 1;
    if w_from <= m
        A_r(w_from,w_to) = 1;
    else
        r = find(A_r(:,w_from - m) == 1);
        A_r(r(1),w_to) = 1;
    end
    %% Second best
    available_now(w_from,w_to) = 0;

    c_now = c(available_now == 1);
    Ito_now = Ito(available_now == 1);
    Ifrom_now = Ifrom(available_now == 1);
    [~,u_ind] = min(c_now);

    u_to = Ito_now(u_ind);
    u_from = Ifrom_now(u_ind);

    U{AuctionRoundCounter} = zeros(m + n,n);
    U{AuctionRoundCounter}(u_from,u_to) = 1;
    
    %%
    available_now = zeros(m + n,n);
    for i2 = 1:n
        for i1 = 1:m
            if A(i2) == 0
                available_now(i1,i2) = 1;
            end
        end
        for i1 = 1:n
            if (A(i1) == 1) && (A(i2) == 0)
                available_now(i1 + m,i2) = 1;
            end
        end
    end   
end


end