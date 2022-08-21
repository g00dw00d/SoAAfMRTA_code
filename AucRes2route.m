function nonzero_route = AucRes2route(E,A_r,ir)
[m,n] = size(A_r);

NeedToVisit = A_r(ir,:);
VisitedSequence = [0];

cangoto = find(E(ir,:) == 1);
goto = cangoto(1);

nonzero_route = [0,goto];
NeedToVisit(goto) = 0;
VisitedSequence = [VisitedSequence, goto];

while sum(NeedToVisit > 0)
    if VisitedSequence(end) == 0 %last element in sequence is the robot origin
        cangoto = find(E(ir,:) .* NeedToVisit);
    else %last element in sequence is not the robot origin
        cangoto = find((E(VisitedSequence(end) + m,:) .* NeedToVisit) | (E(m + 1:m + n,VisitedSequence(end))' .* NeedToVisit));
    end
    if ~isempty(cangoto)
        goto = cangoto(1);
    else
        NoConnection = true; 
        ps = 1;
        while NoConnection 
            if VisitedSequence(end) == 0 %last element in sequence is the robot origin
                if VisitedSequence(ps) == 0 %considered connection point is robot origin
                    
                else %considered connection point is not robot origin
                    if (E(ir,VisitedSequence(ps)) == 1) 
                        goto = VisitedSequence(ps);
                        NoConnection = false;
                    end
                end               
            else %last element in sequence is not the robot origin
                if VisitedSequence(ps) == 0 %considered connection point is robot origin
                    if (E(ir,VisitedSequence(end)) == 1) 
                        goto = VisitedSequence(ps);
                        NoConnection = false;
                    end                    
                else %considered connection point is not robot origin
                    if (E(VisitedSequence(end) + m,VisitedSequence(ps)) == 1) || (E(VisitedSequence(ps) + m,VisitedSequence(end)) == 1) 
                        goto = VisitedSequence(ps);
                        NoConnection = false;
                    end
                end
            end
            ps = ps + 1;
            if ps > length(VisitedSequence)
                pause
            end    
        end
    end

    nonzero_route = [nonzero_route; [VisitedSequence(end),goto]];
    if goto > 0
        NeedToVisit(goto) = 0;
    end
    VisitedSequence = [VisitedSequence, goto];
end



end