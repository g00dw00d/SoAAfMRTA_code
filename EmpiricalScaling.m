n_range = 5:1:100;
avN = 20;
TIME_SSI_step1 = zeros(avN,length(n_range));
TIME_SSI_step2 = zeros(avN,length(n_range));
TIME_MINMID = zeros(avN,length(n_range));
TIME_ROBUST = zeros(avN,length(n_range));

for in = 1:length(n_range)
    n = n_range(in)
    for iav = 1:avN
        main_scale
        
        TIME_SSI_step1(iav,in) = time_SSI_step1;
        TIME_SSI_step2(iav,in) = time_SSI_step2;
        TIME_MINMID(iav,in) = time_MINMID;
        TIME_ROBUST(iav,in) = time_ROBUST;
    end
end


%%
TIME_SSI = TIME_SSI_step1 + TIME_SSI_step2;
TIME_Sensitivity = TIME_MINMID + TIME_ROBUST;

%% averages
TIME_SSI_step1_av = sum(TIME_SSI_step1,1) * 1 / avN;
TIME_SSI_step2_av = sum(TIME_SSI_step2,1) * 1 / avN;
TIME_SSI_av = sum(TIME_SSI,1) * 1 / avN;
TIME_MINMID_av = sum(TIME_MINMID,1) * 1 / avN;
TIME_ROBUST_av = sum(TIME_ROBUST,1) * 1 / avN;
TIME_Sensitivity_av = sum(TIME_Sensitivity,1) * 1 / avN;

%% medians
TIME_SSI_step1_med = median(TIME_SSI_step1,1);
TIME_SSI_step2_med = median(TIME_SSI_step2,1);
TIME_SSI_med = median(TIME_SSI,1);
TIME_MINMID_med = median(TIME_MINMID,1);
TIME_ROBUST_med = median(TIME_ROBUST,1);
TIME_Sensitivity_med = median(TIME_Sensitivity,1);

%% plotting average
figure(301)
clf
subplot(2,1,1)
plot(n_range,TIME_SSI_av,'k-','LineWidth',2)
hold on
plot(n_range,TIME_SSI_step1_av,'b--','LineWidth',1)
plot(n_range,TIME_SSI_step2_av,'r:','LineWidth',1)
title('Average times for assignment and routing')
legend('Overall','Step 1','Step 2','Location','northwest')
xlabel('Number of tasks')
ylabel('Computation time [s]')
axis([n_range(1),n_range(end),0,TIME_SSI_av(end)])

subplot(2,1,2)
plot(n_range,TIME_Sensitivity_av,'k-','LineWidth',2)
hold on
plot(n_range,TIME_MINMID_av,'b--','LineWidth',1)
plot(n_range,TIME_ROBUST_av,'r:','LineWidth',1)
axis([n_range(1),n_range(end),0,TIME_Sensitivity_av(end)])


title('Average times for analysing sensitivity')
legend('Overall','MINMID','ROBUST','Location','northwest')
xlabel('Number of tasks')
ylabel('Computation time [s]')

%% plotting median
figure(302)
clf
subplot(2,1,1)
plot(n_range,TIME_SSI_med,'k-','LineWidth',2)
hold on
plot(n_range,TIME_SSI_step1_med,'b--','LineWidth',1)
plot(n_range,TIME_SSI_step2_med,'r:','LineWidth',1)
title('Median times for assignment and routing')
legend('Overall','Step 1','Step 2','Location','northwest')
xlabel('Number of tasks')
ylabel('Computation time [s]')
axis([n_range(1),n_range(end),0,TIME_SSI_med(end)])

subplot(2,1,2)
plot(n_range,TIME_Sensitivity_med,'k-','LineWidth',2)
hold on
plot(n_range,TIME_MINMID_med,'b--','LineWidth',1)
plot(n_range,TIME_ROBUST_med,'r:','LineWidth',1)
axis([n_range(1),n_range(end),0,TIME_Sensitivity_med(end)])


title('Median times for analysing sensitivity')
legend('Overall','MINMID','ROBUST','Location','northwest')
xlabel('Number of tasks')
ylabel('Computation time [s]')
