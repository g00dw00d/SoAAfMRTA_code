function cost = PathCost(Edges,c_T0_loc,c_TT)

cost = 0;

for it = 1:size(Edges,1)
    if Edges(it,1) == 0
        cost = cost + c_T0_loc(Edges(it,2));
    else
        if Edges(it,2) == 0
            cost = cost + c_T0_loc(Edges(it,1));
        else
            cost = cost + c_TT(Edges(it,1),Edges(it,2));
        end
    end
end    


end