close all;
clear;

%% General Flags
run_mpc_cbf_one = false;
display_mpc_cbf_one = false;
run_mpc_cbf_multiple = true;
display_mpc_cbf_multiple = true;

%% Setup and Parameters
x0 = [-5; -5; 0; 0];  % Initial state
time_total = 30.0;
dt = 0.2;
P = 100 * eye(4);
Q = 10 * eye(4);
R = eye(2);
N = 8;
xmin = [-5; -5; -5; -5];
xmax = [5; 5; 5; 5];
umin = [-1; -1];
umax = [1; 1];

%% Discrete-time double integrator 2D (System Model)
system.dt = dt;
system.A = [1 0 dt 0;
            0 1 0 dt;
            0 0 1 0;
            0 0 0 1];
system.B = [0.5 * dt^2 0;
            0 0.5 * dt^2;
            dt 0;
            0 dt];
system.xl = xmin;
system.xu = xmax;
system.ul = umin;
system.uu = umax;

%% MPC-CBF Parameters
params_mpc_cbf.Q = Q;
params_mpc_cbf.R = R;
params_mpc_cbf.P = P;
params_mpc_cbf.N = N;
params_mpc_cbf.gamma = 0.4;

%% Obstacle Definition
obs.pos = [-2; -2.25];
obs.r = 1.5;

%% Simulate MPC-CBF with N=8
params_mpc_cbf.N = 8;
if run_mpc_cbf_multiple
    fprintf('Running MPC-CBF with N=8...\n');
    controller_mpc_cbf_multiple = MPCCBF(x0, system, params_mpc_cbf);
    controller_mpc_cbf_multiple.obs = obs;
    controller_mpc_cbf_multiple.sim(time_total);
    
    % ? Print solver time
    avg_solver_time = mean(controller_mpc_cbf_multiple.solvertime);
    fprintf('Average Solver Time (N=8): %.6f seconds\n', avg_solver_time);
end

%% Display MPC-CBF Simulation with N=8
if display_mpc_cbf_multiple
    figure;
    hold on;

    % ? Ensure trajectory is being plotted
    if isempty(controller_mpc_cbf_multiple.xlog)
        fprintf('Error: No trajectory data found. Ensure optimizer is solving correctly.\n');
    else
        plot(controller_mpc_cbf_multiple.xlog(1,:), controller_mpc_cbf_multiple.xlog(2,:), 'ko-', 'LineWidth', 1.0, 'MarkerSize', 4);
    end
    
    % ? Plot Obstacle
    pos = controller_mpc_cbf_multiple.obs.pos;
    r = controller_mpc_cbf_multiple.obs.r;
    theta = linspace(0, 2*pi, 100);
    plot(pos(1) + r * cos(theta), pos(2) + r * sin(theta), 'r', 'LineWidth', 2);
    
    % ? Plot Start & Goal Position
    plot(controller_mpc_cbf_multiple.x0(1), controller_mpc_cbf_multiple.x0(2), 'db', 'LineWidth', 1);
    plot(0.0, 0.0, 'dr', 'LineWidth', 1);
    
    title('MPC-CBF Obstacle Avoidance (N=8)');
    xlabel('x (m)');
    ylabel('y (m)');
    grid on;
    legend('Trajectory', 'Obstacle', 'Start Position', 'Goal Position');
end
