clc;
close all;
clear;
format compact;

global cmd_args

%% initialize cmd params
cmd_args = get_cmd_args();

%% Set the matlab utils paths to use custom built utility functions
set_matlab_utils_path();

USE_2nd_order_can_sys = false;

%% Load demos and process demos
load data.mat data Ts

% calculate numerically the 1st and 2nd derivatives
[yd_data, dyd_data, ddyd_data] = process_demos(data, Ts, cmd_args.add_points_percent, cmd_args.smooth_points_percent);

n_data = size(yd_data,2); % number of points in each dimension
Time_demo = ((1:n_data)-1)*Ts;


%% Set up DMP params
tau = (n_data-1)*Ts;

D = size(yd_data,1); % dimensionality of training data
n_data = size(yd_data,2); % number of training points

number_of_kernels = cmd_args.N_kernels
n_data

if (strcmpi(cmd_args.CAN_SYS_TYPE,'lin'))
    can_sys_ptr = LinCanonicalSystem(cmd_args.x_end, tau);
elseif (strcmpi(cmd_args.CAN_SYS_TYPE,'exp'))
    can_sys_ptr = ExpCanonicalSystem(cmd_args.x_end, tau);
elseif (strcmpi(cmd_args.CAN_SYS_TYPE,'spring-damper'))
    can_sys_ptr = SpringDamperCanonicalSystem(cmd_args.x_end, tau);
    USE_2nd_order_can_sys = true;
else
    error('Unsupported canonical system type ''%s''',cmd_args.CAN_SYS_TYPE);
end


dmp = cell(D,1);
for i=1:D
    
    if (strcmpi(cmd_args.DMP_TYPE,'DMP'))
       dmp{i} = DMP(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K);
    elseif (strcmpi(cmd_args.DMP_TYPE,'DMP-bio'))
        dmp{i} = DMP_bio(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K);
    elseif (strcmpi(cmd_args.DMP_TYPE,'DMP-plus'))
        dmp{i} = DMP_plus(cmd_args.N_kernels, cmd_args.a_z, cmd_args.b_z, can_sys_ptr, cmd_args.std_K);
    else
        error('Unsupported DMP type ''%s''', cmd_args.DMP_TYPE);
    end
    
end


F_train_data = [];
Fd_train_data = [];

%% Train the DMP
disp('DMP training...')
tic
train_mse = zeros(D,1); 
n_data = length(yd_data(i,:));
Time = (0:n_data-1)*Ts;
for i=1:D
    
    y0 = yd_data(i,1);
    g0 = yd_data(i,end);
    
    ind = 1:n_data;
%     ind = randperm(n_data);
    T = Time(ind);
    yd = yd_data(i,ind);
    dyd = dyd_data(i,ind);
    ddyd = ddyd_data(i,ind);

    [train_mse(i), F_train, Fd_train] = dmp{i}.train(T, yd, dyd, ddyd, y0, g0, cmd_args.train_method, cmd_args.USE_GOAL_FILT, cmd_args.a_g);      

    F_train_data = [F_train_data; F_train];
    Fd_train_data = [Fd_train_data; Fd_train];
    
end
Time_train = (0:(size(F_train_data,2)-1))*Ts;

toc

%% DMP simulation
% set initial values
y0 = yd_data(:,1);
g0 = yd_data(:,end); 
g = g0;
dg = zeros(D,1);
if (cmd_args.USE_GOAL_FILT), g = y0; end
x = cmd_args.x0;
dx = 0;
if (USE_2nd_order_can_sys)
    u = 0;
else
    u = x;
end
du = 0;
ddy = zeros(D,1);
dy = zeros(D,1);
y = y0;
t = 0;
y_robot = y0;
dy_robot = zeros(D,1);
dz = zeros(D,1);
z = tau*dy + cmd_args.a_py*(y_robot-y);
scaled_forcing_term = zeros(D,1);
shape_attr = zeros(D,1);
goal_attr = zeros(D,1);

Fdist = 0;

log_data = get_logData_struct();   

log_data.dmp = dmp;
log_data.Time_demo = Time_demo;
log_data.yd_data = yd_data;
log_data.dyd_data = dyd_data;
log_data.ddyd_data = ddyd_data;
log_data.D = D;
log_data.Ts = Ts;
log_data.g0 = g0;
log_data.Time_train = Time_train;
log_data.F_train_data = F_train_data;
log_data.Fd_train_data = Fd_train_data;
log_data.Psi_data = cell(D,1);

% for some strange reason I have to pass cell array explicitly here. In the
% initialization of the structure above, cell array are rendered as []...?
log_data.shape_attr_data = cell(D,1);
log_data.goal_attr_data = cell(D,1);


tau = cmd_args.tau_sim_scale*tau;
can_sys_ptr.set_tau(tau);

iters = 0;
dt = cmd_args.dt;

disp('DMP simulation...')
tic
while (true)

    %% data logging

    log_data.Time = [log_data.Time t];
    
    log_data.dy_data = [log_data.dy_data dy];
    log_data.y_data = [log_data.y_data y];

    log_data.dy_robot_data = [log_data.dy_robot_data dy_robot];
    log_data.y_robot_data = [log_data.y_robot_data y_robot];

    log_data.z_data = [log_data.z_data z];
    log_data.dz_data = [log_data.dz_data dz];
        
    log_data.x_data = [log_data.x_data x];
    log_data.u_data = [log_data.u_data u];
    
    log_data.Fdist_data = [log_data.Fdist_data Fdist];
    
    log_data.Force_term_data = [log_data.Force_term_data scaled_forcing_term];
    
    log_data.g_data = [log_data.g_data g];
    
    %log_data.shape_attr_data = [log_data.shape_attr_data shape_attr];
    %log_data.goal_attr_data = [log_data.goal_attr_data goal_attr];
    
    %% DMP simulation
    
    for i=1:D
        
        v_scale = dmp{i}.get_v_scale();% 1 / (tau*dmp{i}.a_s);
        
        Psi = dmp{i}.activation_function(x);
        log_data.Psi_data{i} = [log_data.Psi_data{i} Psi(:)];

        %shape_attr(i) = dmp{i}.shape_attractor(x,u,g0(i),y0(i));
        %goal_attr(i) = dmp{i}.goal_attractor(y(i),dy(i),g(i));

        scaled_forcing_term(i) = dmp{i}.forcing_term(x)*dmp{i}.forcing_term_scaling(u, y0(i), g0(i));
        
        %dz(i) = ( dmp{i}.a_z*(dmp{i}.b_z*(g(i)-y(i))-z(i)) + force_term(i) ) / v_scale;
        %dy(i) = ( z(i) - cmd_args.a_py*(y_robot(i)-y(i)) ) / v_scale;
        
        y_c = - cmd_args.a_py*(y_robot(i)-y(i));
        z_c = 0;
        
        [dy(i), dz(i)] = dmp{i}.get_states_dot(y(i), z(i), x(i), u(i), y0(i), g0(i), g(i), y_c, z_c);
        
        dy_robot(i) = dy(i) - (cmd_args.Kd/cmd_args.Dd)*(y_robot(i)-y(i)) + Fdist/cmd_args.Dd; 
      
    end
    
    if (cmd_args.USE_GOAL_FILT)
        dg = -cmd_args.a_g*(g-g0)/can_sys_ptr.tau;
    else
        dg = zeros(size(g));
    end
    
    %% Update phase variable
    
    X_in = x;
    if (USE_2nd_order_can_sys)
        X_in = [X_in; u];
    end
        
    X_out = can_sys_ptr.get_derivative(X_in);
    
%     x = X_out(1);
    dx = X_out(1);
    if (length(X_out) > 1)
%         u = X_out(2);
        du = X_out(2);
    else
        du = dx;
    end
    
    %% Update disturbance force
    if (cmd_args.APPLY_DISTURBANCE)
        Fdist = Fdist_fun(t);
    end
     
    if (cmd_args.USE_PHASE_STOP)
        stop_coeff = 1/(1+cmd_args.a_px*norm(y-y_robot)^2);
        
        dx = dx*stop_coeff;
        du = du*stop_coeff;
        dg = dg*stop_coeff;
    end
    
    err = max(abs(g0-y_robot));
    if (err <= cmd_args.tol_stop), break; end
    
    iters = iters + 1;
    if (iters >= cmd_args.max_iters), break; end
    
    %% Numerical integration
    t = t + dt;
    
    y = y + dy*dt;
    
    z = z + dz*dt;
    
    g = g + dg*dt;
    
    x = x + dx*dt;
    if (x<0), x=0; end % zero crossing can occur due to numberical integration
    
    if (USE_2nd_order_can_sys)
        u = u + du*dt;
    else
        u = x;
    end
    
    y_robot = y_robot + dy_robot*dt;

end
toc

save dmp_results.mat log_data cmd_args;
...
%     Time_demo yd_data dyd_data ddyd_data ...
%     D Ts ...
%     Time y_data dy_data z_data dz_data x_data u_data Force_term_data ...
%     Fdist_data ...
%     g0 g_data ...
%     y_robot_data dy_robot_data ...
%     Time_train F_train_data Fd_train_data
    

%% Find mean square error between the signals

y_data = log_data.y_data;
yd_data = log_data.yd_data;

dist_f = @(s1,s2) norm(s1-s2);
dtw_win = floor(max([size(y_data,2), size(yd_data,2)])/3);
[dtw_dist, ind_y, ind_yd, C] = dtw(y_data, yd_data, dtw_win, dist_f);
sim_mse = sum(C)/length(C);


train_mse
sim_mse
dtw_dist/length(C)
