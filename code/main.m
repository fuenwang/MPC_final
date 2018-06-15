% EE551000 System Theory
% Model Predictive Control Using FPGA
% 2018/06/15 106061531 Fu-En Wang

clear;
%% state-space model
Am = [-0.0001 -0.0000; 3.3864 0.9974];
Bm = [0.0025; 0.2594];
Cm = [0 1];
%% hyper parameters
Np = 10;
Nc = 3;
delta_t = 0.1;
Nsim = 250;

%% augmented state-space model
[F, Phi,A,B,C] = mpcgain(Am,Bm,Cm,Nc,Np);


%% Model input constraint M * dU <= gamma
% M x dU <= 24 (gamma)
M1 = tril(ones(Nc));
gamma1 = 24 * ones(Nc, 1);
% -(M x dU) <= -24 (gamma)
M2 = -tril(ones(Nc));
gamma2 = -24 * ones(Nc, 1);

I = [-ones(Nc, 1); ones(Nc, 1)];
M = [M1; M2];
gamma = [gamma1; gamma2];
%options = optimoptions('quadprog', 'Display', 'off');

%% create data buffer
all_u = zeros(1, Nsim);
all_u(1, 1)= 11;
all_x = zeros(3, Nsim);
all_x(:, 1) = [0.3 0.2 -0];
all_y = zeros(1, Nsim);

%% Set H for QP
R_bar = 1*eye(Nc, Nc);
Rs_bar = ones(Np, 1);
H = Phi' * Phi + R_bar;
Rs = ones(Np, 1); % one * r set poiint signal
%f = -2 * Phi' * (Rs - F * zeros(3, 1));

for i = 2:Nsim
    prev_u = all_u(1, i-1);
    now_M = M;
    now_gamma = gamma + prev_u * I;
    f = -2 * Phi' * (Rs - F * all_x(:, i-1)); % set f for QP according to previous x and set-point signal
    %f = f';
    %problem = struct('H', H, 'f', f, 'Aineq', now_M, 'bineq', now_gamma, 'options', options, 'solver', 'quadprog');
    %delta_u = quadprog(problem);
    delta_u = QPhild(H, f, now_M, now_gamma); % solve QP
    delta_u = delta_u(1, 1); % receding horizon control
    
    all_u(1, i) = prev_u + delta_u;  % update u
    all_x(:, i) = A * all_x(:, i-1) + B * delta_u; % update state
end

%% plot result
fig = figure();
plot((1:size(all_x, 2))*delta_t, all_x(3, :), 'b', 'linewidth', 2);
ylim([0, 1.4])
xlabel('Duration Time');
ylabel('Motor Speed');
title('Output')
%saveFig(fig, '/media/external/Fu-En.Wang/MPC_final/doc/src/output.pdf');
          
fig = figure();
plot((1:size(all_u(1:end), 2))*delta_t, all_u(1:end), 'b', 'linewidth', 2);
ylim([5 11.5])
xlabel('Duration Time');
ylabel('Applied Voltage');
title('Input')
%saveFig(fig, '/media/external/Fu-En.Wang/MPC_final/doc/src/input.pdf');

    
   
    














