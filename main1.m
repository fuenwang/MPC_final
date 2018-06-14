% Model Predictive Control Using FPGA

clear;

Am = [-0.0001 -0.0000; 3.3864 0.9974];
Bm = [0.0025; 0.2594];
Cm = [0 1];

Np = 10;
Nc = 3;

delta_t = 0.1;
Nsim = 250;

[F, Phi,A,B,C] = mpcgain(Am,Bm,Cm,Nc,Np);


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
options = optimoptions('quadprog');
%{
H = eye(Nc);
f = ones(1, Nc);
options = optimoptions('quadprog', 'Display', 'off');
problem = struct('H', H, 'f', f, 'Aineq', M, 'bineq', gamma, 'options', options, 'solver', 'quadprog');
quadprog(problem)
%}
all_u = zeros(1, Nsim);
all_u(1, 1)= 11;

all_x = zeros(3, Nsim);
all_x(:, 1) = [0.3 0.2 -0];

all_y = zeros(1, Nsim);



R_bar = 1*eye(Nc, Nc);
Rs_bar = ones(Np, 1);
H = Phi' * Phi + R_bar;
Rs = ones(Np, 1); % one * r set poiint signal
%f = -2 * Phi' * (Rs - F * zeros(3, 1));

for i = 2:Nsim
    prev_u = all_u(1, i-1);
    now_M = M;
    now_gamma = gamma + prev_u * I;
    f = -2 * Phi' * (Rs - F * all_x(:, i-1));
    %f = f';
    %H = 
    
    
    %problem = struct('H', H, 'f', f, 'Aineq', now_M, 'bineq', now_gamma, 'options', options, 'solver', 'quadprog');
    %delta_u = quadprog(problem);
    delta_u = QPhild(H, f, now_M, now_gamma);
    delta_u = delta_u(1, 1);
    
    all_u(1, i) = prev_u + delta_u;
    all_x(:, i) = A * all_x(:, i-1) + B * delta_u;
end

figure
plot((1:size(all_x, 2))*delta_t, all_x(3, :));
ylim([0, 1.4])
xlabel('Duration Time');
ylabel('Motor Speed');

figure
plot((1:size(all_u(1:end), 2))*delta_t, all_u(1:end));
ylim([5 11.5])
xlabel('Duration Time');
ylabel('Applied Voltage');


















