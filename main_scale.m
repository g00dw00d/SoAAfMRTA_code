

%% Define robot and task positions

xmax = 120;
ymax = 90;

% number of tasks
% n = 100;

%task locations
p_T = [xmax * rand(1,n); ymax * rand(1,n)];


%number of robots
m = 5;

%initial posistions of robots
p_R = [xmax * rand(1,m); ymax * rand(1,m)];


% %steps in schedule
% smax = n;%ceil(n / m);


%% plot robots and tasks
% figure(101)
% clf
% 
% %robot colours
% RobCol = zeros(3,m);
% 
% %plot task positions
% plot(p_T(1,:),p_T(2,:),'kx','MarkerSize',4)
% 
% hold on
% 
% %plot robot inistial positions
% for i = 1:m
%     RobCol(:,i) = [i / m, 0.5,1 - i / m]';    
%     plot(p_R(1,i), p_R(2,i),'o','MarkerSize',4,'MarkerEdgeColor',RobCol(:,i))
% end
% 
% for i = 1:m
%     text(p_R(1,i),p_R(2,i)+3,['$r_{',num2str(i),'}$'],'Color',RobCol(:,i),'HorizontalAlignment','center','Interpreter','latex')    
% end
% for j = 1:n
%     text(p_T(1,j),p_T(2,j)+3,['$t_{',num2str(j),'}$'],'Color','k','HorizontalAlignment','center','Interpreter','latex') 
% end
% 
% axis equal
% axis([0,xmax,0,ymax])
% xlabel('x-coordinate [m]')
% ylabel('y-coordinate [m]')


%% Determine costs

%intrinsic task cost
max_intrinsic = 0;
c_int = max_intrinsic * rand(1,n);

%cost of going from robot initial position to task
distcost = 1;
[X_TR_1,X_TR_0] = meshgrid(p_T(1,:),p_R(1,:));
[Y_TR_1,Y_TR_0] = meshgrid(p_T(2,:),p_R(2,:));
c_dist_0 = distcost * sqrt((X_TR_1 - X_TR_0) .^ 2 + (Y_TR_1 - Y_TR_0) .^ 2); 

%cost of going to next task 
[X_TT_kp1,X_TT_k] = meshgrid(p_T(1,:));
[Y_TT_kp1,Y_TT_k] = meshgrid(p_T(2,:));
c_dist = distcost * sqrt((X_TT_kp1 - X_TT_k) .^ 2 + (Y_TT_kp1 - Y_TT_k) .^ 2); 

%cost of task from robot initial position
c_T0 = c_dist_0 + repmat(c_int,m,1);

%cost of task (columm) conditioned on last task (row)
c_TT = c_dist + repmat(c_int,n,1);



%%
tic
[A_r,W,E,U,available_k] = SSI(c_T0,c_TT);

time_SSI_step1 = toc;
%%
tic

rob_route = cell(m,1);
for ir = 1:m
    if sum(A_r(ir,:)) > 0
        rob_route{ir} = AucRes2route(E,A_r,ir);        
    end    
end

time_SSI_step2 = toc;

%% plot paths
% figure(102)
% clf
% 
% %
% plot(p_T(1,:),p_T(2,:),'kx','MarkerSize',4)
% hold on
% mh = min(min(c_T0))/ 1;
% mh = 10;%ll9.8590;
% 
% for i = 1:m
%     plot(p_R(1,i), p_R(2,i),'o','MarkerSize',4,'MarkerEdgeColor',RobCol(:,i))
%     for s = 1:size(rob_route{i},1)
%         if rob_route{i}(s,1) == 0
%             quiver(p_R(1,i), p_R(2,i), p_T(1,rob_route{i}(s,2)) - p_R(1,i), p_T(2,rob_route{i}(s,2)) - p_R(2,i),1,'Color',RobCol(:,i),'MaxHeadSize',mh / norm([p_T(1,rob_route{i}(s,2)) - p_R(1,i), p_T(2,rob_route{i}(s,2)) - p_R(2,i)]))            
%         else
%             if rob_route{i}(s,2) == 0
%                 quiver(p_T(1,rob_route{i}(s,1)), p_T(2,rob_route{i}(s,1)), p_R(1,i) - p_T(1,rob_route{i}(s,1)), p_R(2,i) - p_T(2,rob_route{i}(s,1)),1,'Color',RobCol(:,i),'MaxHeadSize',mh / norm([p_R(1,i) - p_T(1,rob_route{i}(s,1)), p_R(2,i) - p_T(2,rob_route{i}(s,1))]))                   
%             else
%                 quiver(p_T(1,rob_route{i}(s,1)), p_T(2,rob_route{i}(s,1)), p_T(1,rob_route{i}(s,2)) - p_T(1,rob_route{i}(s,1)), p_T(2,rob_route{i}(s,2)) - p_T(2,rob_route{i}(s,1)),1,'Color',RobCol(:,i),'MaxHeadSize',mh / norm([p_T(1,rob_route{i}(s,2)) - p_T(1,rob_route{i}(s,1)), p_T(2,rob_route{i}(s,2)) - p_T(2,rob_route{i}(s,1))]))
%             end
%         end
%     end
% end
% 
% for i = 1:m
%     text(p_R(1,i),p_R(2,i)+3,['$r_{',num2str(i),'}$'],'Color',RobCol(:,i),'HorizontalAlignment','center','Interpreter','latex')    
% end
% for j = 1:n
%     text(p_T(1,j),p_T(2,j)+3,['$t_{',num2str(j),'}$'],'Color','k','HorizontalAlignment','center','Interpreter','latex') 
% end
% axis equal
% axis([0,xmax,0,ymax])
% xlabel('x-coordinate [m]')
% ylabel('y-coordinate [m]')
% title('SSI') 


%% evaluate assignment cost
% RobCost_r = zeros(m,1);
% for ir = 1:m
%     RobCost_r(ir) = PathCost(rob_route{ir},c_T0(ir,:),c_TT);
% end
% cost_SSI = sum(RobCost_r);


%% MIMID
c = [c_T0;c_TT];

tic
upper_E = MINMID(c,W,E,available_k);
time_MINMID = toc;



%% ROBUST
% c = [c_T0;c_TT];
% upper_E_manual = inf(m +n,n);
% for ar = 1:n
% %     upper_E(W{ar} == 1) = rand * (c(U{ar} == 1) - c(W{ar} == 1));
%     upper_E_manual(W{ar} == 1) = (c(U{ar} == 1) - c(W{ar} == 1)) / 2;
% end

tic 
[lower,upper] = ROBUST([c_T0;c_TT],W,E,upper_E,available_k);
time_ROBUST = toc;


%%
% [Ito,Ifrom] = meshgrid(1:n,1:(m + n)); 
% figure(106)
% clf
% surf(Ifrom,Ito,-[lower_T0_capped;lower_TT_capped])
% xlabel('Vertex 1')
% ylabel('Vertex 2')
% zlabel('Smallest allowable perturbation')
% axis([1,m + n,1,n,-50,0])
% 
% figure(107)
% clf
% surf(Ifrom,Ito,[upper_T0_capped;upper_TT_capped])
% xlabel('Vertex 1')
% ylabel('Vertex 2')
% zlabel('Largest allowable perturbation')
% axis([1,m + n,1,n,0,50])

%% test allowable perturbation
%%
% expN = 1000;
% c_test = zeros(m + n,n,expN);
% 
% lower_T0 = lower(1:m,:);
% lower_TT = lower(m + 1:m + n,:);
% upper_T0 = upper(1:m,:);
% upper_TT = upper(m + 1:m + n,:);
% 
% 
% rf = 50;
% lower_T0_capped = min(lower_T0,rf);
% lower_TT_capped = min(lower_TT,rf);
% upper_T0_capped = min(upper_T0,rf);
% upper_TT_capped = min(upper_TT,rf);
% 
% 
% for testit = 1:expN    
% 
%     c_add_TT = -lower_TT_capped + rand(size(c_TT)) .* (upper_TT_capped + lower_TT_capped);
%     c_add_T0 = -lower_T0_capped + rand(size(c_T0)) .* (upper_T0_capped + lower_T0_capped);
%     c_TT_p = c_TT + c_add_TT;
%     c_T0_p = c_T0 + c_add_T0;
%     
%     c_test(:,:,testit) = [c_T0_p;c_TT_p];
% 
%     
%     [~,W_comp,~,~,~] = SSI(c_T0_p,c_TT_p);
%     
%     CompRounds = zeros(n,1);
%     for round = 1:n
%         CompRounds(round) = prod(prod(W{round} == W_comp{round}));
%     end
%     IsSame = prod(CompRounds);
%     if ~IsSame
%         IsSame
%     end
% end



%%
% figure(105)
% clf
% for j = 1:n
%     for i = 1:(m + n)
%         subplot(m + n,n,(j - 1) * (m + n) + i)
%         plot(zeros(1,expN),squeeze(c_test(i,j,:)),'.');
%         hold on
%         plot(0,c(i,j),'kx')
%         ax = gca;
%         ax.XTickLabel = [];
%         xlabel(['edge (',num2str(i),',',num2str(j),')']);
%         axis([-1,1,0,150])
%     end
% end
% 
% 



%% true min-sum-sum optimal solution
% [schedule_MinSumSum,cost_MinSumSum_Ob,diagnositics_MinSumSum] = MinSumSum(c_T0,c_TT);
% 
% RobCost_MinSumSum = zeros(m,1);
% Assignment_MinSumSum = zeros(m,n,n);
% PastCost_MinSumSum = cell(m,1);
% for ir = 1:m
%     PastCost_MinSumSum{ir} = zeros(1,length(schedule_MinSumSum{ir}));
%     for io = 1:length(schedule_MinSumSum{ir})
%         Assignment_MinSumSum(ir,schedule_MinSumSum{ir}(io),io) = 1;
%         PastCost_MinSumSum{ir}(io) = Schedule2PathCost(schedule_MinSumSum{ir}(1:io),c_T0(ir,:),c_TT);
%     end
%     RobCost_MinSumSum(ir) = Schedule2PathCost(schedule_MinSumSum{ir},c_T0(ir,:),c_TT);
% end
% cost_MinSumSum = sum(RobCost_MinSumSum);
% 
% %% compare
% OptimalityGap = cost_SSI - cost_MinSumSum;
% 
% CostRatio = cost_SSI / cost_MinSumSum
% 
% %% plot optimal
% figure(103)
% clf
% 
% %
% plot(p_T(1,:),p_T(2,:),'kx','MarkerSize',4)
% hold on
% 
% for i = 1:m
%     
%     plot(p_R(1,i), p_R(2,i),'o','MarkerSize',4,'MarkerEdgeColor',RobCol(:,i))
% %     text(p_R(1,i),p_R(2,i)+3,'0','Color',RobCol(:,i),'HorizontalAlignment','center') 
%     
%     if ~isempty(schedule_MinSumSum{i})
%         quiver(p_R(1,i), p_R(2,i), p_T(1,schedule_MinSumSum{i}(1)) - p_R(1,i), p_T(2,schedule_MinSumSum{i}(1)) - p_R(2,i),1,'Color',RobCol(:,i),'MaxHeadSize',mh / norm([ p_T(1,schedule_MinSumSum{i}(1)) - p_R(1,i), p_T(2,schedule_MinSumSum{i}(1)) - p_R(2,i)]))
%         
% %         text(p_T(1,schedule_MinSumSum{i}(1)),p_T(2,schedule_MinSumSum{i}(1))+3,num2str(PastCost_MinSumSum{i}(1),'%.2f'),'Color',RobCol(:,i),'HorizontalAlignment','center') 
%     
%         for s = 2:length(schedule_MinSumSum{i})
%             quiver(p_T(1,schedule_MinSumSum{i}(s - 1)), p_T(2,schedule_MinSumSum{i}(s - 1)), p_T(1,schedule_MinSumSum{i}(s)) - p_T(1,schedule_MinSumSum{i}(s - 1)), p_T(2,schedule_MinSumSum{i}(s)) - p_T(2,schedule_MinSumSum{i}(s - 1)),1,'Color',RobCol(:,i),'MaxHeadSize',mh / norm([ p_T(1,schedule_MinSumSum{i}(s)) - p_T(1,schedule_MinSumSum{i}(s - 1)), p_T(2,schedule_MinSumSum{i}(s)) - p_T(2,schedule_MinSumSum{i}(s - 1))]))
%             
% %             text(p_T(1,schedule_MinSumSum{i}(s)),p_T(2,schedule_MinSumSum{i}(s))+3,num2str(PastCost_MinSumSum{i}(s),'%.2f'),'Color',RobCol(:,i),'HorizontalAlignment','center') 
%         end        
%     end
% end
% 
% for i = 1:m
%     text(p_R(1,i),p_R(2,i)+3,['$r_{',num2str(i),'}$'],'Color',RobCol(:,i),'HorizontalAlignment','center','Interpreter','latex')    
% end
% for j = 1:n
%     text(p_T(1,j),p_T(2,j)+3,['$t_{',num2str(j),'}$'],'Color','k','HorizontalAlignment','center','Interpreter','latex') 
% end
% 
% axis equal
% axis([0,xmax,0,ymax])
% xlabel('x-coordinate [m]')
% ylabel('y-coordinate [m]')
% title('min-sum-sum optimal') 

