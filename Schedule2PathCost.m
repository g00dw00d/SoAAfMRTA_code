function cost = Schedule2PathCost(Schedule,c_T0_loc,c_TT)

if isempty(Schedule)
    cost = 0;
else
    cost = c_T0_loc(Schedule(1));
    for it = 2:length(Schedule)
        cost = cost + c_TT(Schedule(it - 1),Schedule(it));
    end    
end

end