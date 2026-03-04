clear
close all
clc

%% Initialize the Model Parameters

addpath('functions')
load('kep_SSRGT.mat') % Data of Earth orbit around Sun.
run('Model_Parameters.m');

model                   =       'Dynamics_2023';

load_system(model)
FastRestartStatus       =       get_param(model, 'FastRestart');

if strcmp(FastRestartStatus,'on')
    set_param(model, 'FastRestart', 'off');
end

%% Define simulation option
% for achieving a number n of revolution of the orbit
n_orbit         =       1;
T_rev           =       2*pi/n;
tf              =       n_orbit*T_rev;
FH              =       tf/50;

n_wheels        =       3;
step            =       10;
kp              =       2;
ki              =       1e-3;
kd              =       1e-3;
PID0            =       [kp;ki;kd];

%% Initialize Simulink


set_param(model, 'SolverType', 'Fixed-step');
set_param(model, 'Solver', 'FixedStepAuto');
set_param(model, 'FixedStep', num2str(step));
set_param(model, 'StartTime', '0');
set_param(model, 'StopTime', num2str(FH));
set_param(model,  'RelTol', '1e-12');
set_param(model, 'AbsTol', '1e-12');


%% Problem Initialization
Disturbances=0;

% Uncomment for random initial condition:
% 
% flag = 1;
% if flag == 1
%     disp('Searching for a valid random quaternion...')
% end
% while flag==1
%     v = randn(4,1);
%     q0 = v/norm(v);
%     simOut = sim(model);
%     pointing_error = simOut.get('q_error');
%     [roll, pitch, yaw] = quaternionToEuler(pointing_error(1,:));
%     if abs(roll)<=deg2rad(15) && abs(pitch)<=deg2rad(85) && abs(yaw)<=deg2rad(85)
%         flag = 2;
%     end
% end

% Random q0 fixed to have coherent and repeatible results in report
q0 = [  -0.197305854517669
        -0.465940680109210
         0.616000113946663
         0.603749568952325]';

simOut             = sim(model);
pointing_error     = simOut.get('q_error');
[roll, pitch, yaw] = quaternionToEuler(pointing_error(1,:));

disp('Initial guess:')
disp('   roll      pitch    yaw')
disp(rad2deg([roll, pitch, yaw]))

%% Optimization Algorithm

options = optimoptions('fmincon', ...
                       'Algorithm', 'sqp', ...
                       'Display', 'iter-detailed', ...
                       'MaxIterations', 500, ...
                       'TolCon', 1e-10,...
                       'MaxFunctionEvaluations', 5000, ...
                       'OptimalityTolerance', 1e-8, ...
                       'StepTolerance', 1e-8, ...
                       'ConstraintTolerance', 1e-8, ...
                       'FiniteDifferenceType', 'central', ...
                       'FiniteDifferenceStepSize', 1e-8, ...
                       'Diagnostics', 'on');


set_param(model, 'FastRestart', 'on');

tic
nonlcon = @(PID) torque_constraint(PID, model);
[xstar,fxstar,k,exitflag,xsequence] = fmincon(@(PID) cost_function(PID,model),PID0,[],[],[],[],[1e-3,1e-5,1e-7],[10,0.01,1],nonlcon,options);

while fxstar>=1e8
    fprintf('I change kp_0')
    kp_new = setdiff(0:6, PID0(1));
    PID0(1) = kp_new(randi(length(kp_new)));
    [xstar,fxstar,k,exitflag,xsequence] = fmincon(@(PID) cost_function(PID,model),PID0,[],[],[],[],[1e-3,1e-5,1e-7],[10,0.01,1],nonlcon,options);
end
comput_time = toc;

set_param(model, 'FastRestart', 'off');

fprintf('Optimal parameters:\n')
fprintf('kp = %.4e \n',kp)
fprintf('kd = %.4e \n',kd)
fprintf('ki = %.4e \n\n',ki)
fprintf('Computational time: %.4f \n', comput_time)

%% plots

% FH: plot for the optimization time
tf_plot        = FH;  
set_param(model, 'StopTime', num2str(tf_plot));

Disturbances   = 0;
simOut         = sim(model);
angle_error    = simOut.get('angle_error');
omega_body     = simOut.get('w');
torque_control = simOut.get('torque_control');
pointing_error = simOut.get('q_error');

Disturbances     = 1;
simOut2          = sim(model);
angle_error_d    = simOut2.get('angle_error');
omega_body_d     = simOut2.get('w');
torque_control_d = simOut2.get('torque_control');
pointing_error_d = simOut2.get('q_error');


t_vec            = 0:step:tf_plot;


figure()
plot(t_vec,pointing_error_d,'LineWidth',1.5)
grid on
xlim([0,300])
ylim([-0.8,1.2])
title('Quaternion Error','FontSize',14,'Interpreter','latex')
xlabel('Time [s]','FontSize',14,'Interpreter','latex')
ylabel('Quaternion Error','FontSize',14,'Interpreter','latex')
legend('q1','q2','q3','q4','FontSize',14,'Interpreter','latex')


figure()
plot(t_vec,angle_error_d, 'LineWidth',1.5)
grid on
xlim([0,300])
ylim([-10,150])
title('Pointing Error','FontSize',14,'Interpreter','latex')
xlabel('Time [s]','FontSize',14,'Interpreter','latex')
ylabel('Pointing Error [deg]','FontSize',14,'Interpreter','latex')

figure()
plot(t_vec,omega_body_d, 'LineWidth',1.5)
grid on
xlim([0,300])
title('Angular velocity','FontSize',14,'Interpreter','latex')
xlabel('Time [s]','FontSize',14,'Interpreter','latex')
ylabel('$$\omega$$ [rad/s]','FontSize',14,'Interpreter','latex')


figure()
plot(t_vec,torque_control_d,'LineWidth',1.5)
grid on
xlim([0,300])
title('Control Torque $$M_{act}$$','FontSize',14,'Interpreter','latex')
xlabel('Time [s]','FontSize',14,'Interpreter','latex')
ylabel('Torque [Nm]','FontSize',14,'Interpreter','latex')



%% Simulation of whole orbit 
% tf: plot for the full orbit time

Disturbances     = 1;
set_param(model, 'StopTime', num2str(tf));
simOut3          = sim(model);
angle_error_d    = simOut3.get('angle_error');
omega_body_d     = simOut3.get('w');
torque_control_d = simOut3.get('torque_control');
pointing_error_d = simOut3.get('q_error');
trajectory_r     =  simOut3.get('R_s_e_en');
M_SRP            = simOut3.get('M_SRP');
M_GG             = simOut3.get('M_GG');
t_orb            = 0:step:tf;


figure()
plot(t_orb,pointing_error_d,'LineWidth',1.5)
grid on
title('Quaternion Error','FontSize',14,'Interpreter','latex')
xlabel('Time [s]','FontSize',14,'Interpreter','latex')
ylabel('Quaternion Error','FontSize',14,'Interpreter','latex')
ylim([-0.4,1.02])
legend('q1','q2','q3','q4','FontSize',14,'Interpreter','latex','orientation','horizontal','location','southwest')
x_zoom = [0, 300];        
y_zoom = [-0.8,1.2];        
insetPosition = [0.3 0.41 0.5 0.45]; 
axes('Position', insetPosition);    
box on
hold on
plot(t_orb,pointing_error_d,'LineWidth',1.5)
xlim(x_zoom)
ylim(y_zoom)
grid on


figure()
plot(t_orb,angle_error_d, 'LineWidth',1.5)
grid on
title('Pointing Error','FontSize',14,'Interpreter','latex')
xlabel('Time [s]','FontSize',14,'Interpreter','latex')
ylabel('Pointing Error [deg]','FontSize',14,'Interpreter','latex')
x_zoom = [0,300];        
y_zoom = [-10,150];        
insetPosition = [0.3 0.3 0.5 0.45]; 
axes('Position', insetPosition);     
box on
hold on
plot(t_orb,angle_error_d, 'LineWidth',1.5)
xlim(x_zoom)
ylim(y_zoom)
grid on


figure()
plot(t_orb,omega_body_d, 'LineWidth',1.5)
grid on
ylim([-0.2,0.6])
title('Angular velocity','FontSize',14,'Interpreter','latex')
xlabel('Time [s]','FontSize',14,'Interpreter','latex')
ylabel('$$\omega$$ [rad/s]','FontSize',14,'Interpreter','latex')
x_zoom = [0, 300];        
y_zoom = [-0.15,0.1];        
insetPosition = [0.3 0.4 0.5 0.45]; 
axes('Position', insetPosition);     
box on
hold on
plot(t_orb,omega_body_d, 'LineWidth',1.5)
xlim(x_zoom)
ylim(y_zoom)
grid on


figure()
plot(t_orb,torque_control_d,'LineWidth',1.5)
grid on
ylim([-10.5e-3,4e-3])
title('Control Torque $$M_{act}$$','FontSize',14,'Interpreter','latex')
xlabel('Time [s]','FontSize',14,'Interpreter','latex')
ylabel('Torque [Nm]','FontSize',14,'Interpreter','latex')
x_zoom = [0,300];        
y_zoom = [-7e-3, 4e-3];        
insetPosition = [0.3 0.2 0.45 0.45]; 
axes('Position', insetPosition);     
box on
hold on
plot(t_orb,torque_control_d,'LineWidth',1.5)
xlim(x_zoom)
ylim(y_zoom)
grid on

%% orbit plot

% figure
% orbit_plot=plot3(trajectory_r(:,1),trajectory_r(:,2),trajectory_r(:,3),'LineWidth',2);
% hold on
% grid on
% xlabel('x [km]')
% ylabel('y [km]')
% zlabel('z [km]')
% 
% earth

