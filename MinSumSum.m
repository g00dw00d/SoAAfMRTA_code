function [schedule,cost,diagnositics] = MinSumSum(c_T0,c_TT)

% number of tasks
n = size(c_T0,2);

%number of robots
m = size(c_T0,1);

%steps in schedule
smax = n;%ceil(n / m);


%% null task
%cost of null in the first step
c_N0 = zeros(m,1);

%cost of null after task T
c_NT = zeros(n,1);

%cost of task T after null in previous step
bigM = m * (max(max(c_T0)) + smax * max(max(c_TT)));

c_TN = bigM * ones(1,n);

%% costs
c_0 = [c_T0, c_N0];

c = [c_TT, c_NT; c_TN, 0];



%%
Pi = binvar(m,n + 1,smax,'full');
gamma = sdpvar(m,n + 1,smax,n + 1,'full');

constraints = [];



%assign every time spot for every robot to one task or null
for i = 1:m
    for s = 1:smax
        constraints  = [constraints, sum(Pi(i,:,s)) == 1];
    end
end

%assign one time spot of one robot to every task (not including null)
for j = 1:n
    constraints  = [constraints, sum(sum(Pi(:,j,:),1),3) == 1];
end


constraints = [constraints, ...
    gamma >= 0, gamma <= 1, ...
    ];

for s = 2:smax
    for i = 1:m
        for j = 1:(n + 1)
            for jprev = 1:(n + 1)
                constraints = [constraints, ...
                    gamma(i,j,s,jprev) <= Pi(i,j,s), ...
                    gamma(i,j,s,jprev) <= Pi(i,jprev,s - 1), ...
                    gamma(i,j,s,jprev) >= -1 + Pi(i,j,s) + Pi(i,jprev,s - 1), ...
                    ];
            end
        end
    end
end



%bound every individual robot cost
w = sdpvar(m,n + 1,smax);

constraints = [constraints, w(:,:,1) == Pi(:,:,1) .* c_0];

for i = 1:m 
    for j = 1:n + 1
        for s = 2:smax
            constraints = [constraints, w(i,j,s) == sum(squeeze(gamma(i,j,s,:)) .* squeeze(c(:,j)))];
        end
    end
end

objective = sum(sum(sum(w,1),2),3);

ops = sdpsettings;
ops.solver = 'gurobi';
ops.verbose = 1;

diagnositics = optimize(constraints,objective,ops);

Assignment = value(Pi(:,1:n,:));
schedule = cell(m,1);
for ir = 1:m
    for s = 1:smax
        if sum(Assignment(ir,:,s)) > 0
            schedule{ir} = [schedule{ir}, find(Assignment(ir,:,s) == 1)];
        end
    end
end
cost = value(objective);




