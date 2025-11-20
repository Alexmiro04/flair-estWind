clc; clear; close all;

% =======================================================================
% PARAMETERS
% =======================================================================
g = 9.81;
m = 1.0;
I = diag([0.02,0.02,0.04]);

ts = 0.01; Tsim = 30;
t = (0:ts:Tsim)';   
N = numel(t);

% Reference trajectory (circle)
Rref = 0.5; w = 0.3;
xd   = Rref*cos(w*t);       yd   = Rref*sin(w*t);     zd   = ones(N,1);
xdp  = -Rref*w*sin(w*t);    ydp  = Rref*w*cos(w*t);   zdp  = zeros(N,1);
xd2p = -Rref*w^2*cos(w*t);  yd2p = -Rref*w^2*sin(w*t);zd2p= zeros(N,1);

% Gains
Kp = diag([1.8, 1.8, 4.5]);
Kd = diag([1.1, 1.1, 2.2]);

Kp_att = diag([6.0, 6.0, 6.0]);
Kd_att = diag([1.2, 1.2, 1.2]);

% Limits
T_min = 0.0;  T_max = 30.0;
phi_max = deg2rad(35); theta_max = deg2rad(35);

% Initial state
X0 = zeros(12,1);
X = zeros(N,12); 
X(1,:) = X0';

% =======================================================================
% WIND DISTURBANCE (TRUE DISTURBANCE, UNKNOWN TO NN)
% =======================================================================
wind.enable   = true;
wind.t_start  = 5.0;
wind.t_end    = wind.t_start + 18.0;

wind.mu       = 0.11;
wind.sigma    = 0.92;

rng(0);
v_w = wind.mu + wind.sigma*randn(N,1);

rho = 1.225; Cd = 1.5; A = 0.2;  % aerodynamic params

% =======================================================================
% NEURAL NETWORK INITIALIZATION (1 hidden layer, scalar output)
% =======================================================================
nx  = 4;     % input dimension
nh  = 8;     % hidden neurons
eta = 0.1;  % learning rate

W1 = 0.1*randn(nh, nx);
b1 = zeros(nh,1);
W2 = 0.1*randn(1, nh);
b2 = 0;

% Storage
d_hat_hist = zeros(N,1);
r_x_hist   = zeros(N,1);
u_des_hist = zeros(N,3);
T_cmd_hist = zeros(N,1);
Fdist_hist = zeros(N,1);

acc_prev = [0;0;0];   % initial acceleration estimate

% =======================================================================
% SIMULATION
% =======================================================================
for k = 1:N-1
    
    xk = X(k,:)';
    pos   = xk(1:3);
    vel   = xk(4:6);
    ang   = xk(7:9);
    omega = xk(10:12);

    pd = [xd(k); yd(k); zd(k)];
    vd = [xdp(k); ydp(k); zdp(k)];
    ad = [xd2p(k); yd2p(k); zd2p(k)];

    % =====================================================
    % TRUE WIND FORCE
    % =====================================================
    v_air = vel(1) - v_w(k);
    if wind.enable && t(k)>=wind.t_start && t(k)<=wind.t_end
        Fdist_x = 0.5*rho*Cd*A*v_air*abs(v_air);
    else
        Fdist_x = 0;
    end
    Fdist = [Fdist_x; 0; 0];
    Fdist_hist(k) = Fdist_x;

    % =====================================================
    % NN INPUT VECTOR (Version 1)
    % =====================================================
    e  = pos - pd;
    ed = vel - vd;

    s_x = [ e(1);
            ed(1);
            vel(1);
            acc_prev(1) ];     % placeholder (could use previous u_nom_x)

    % --------------------------- NN FORWARD ----------------------------
    z1 = W1*s_x + b1;
    h  = tanh(z1);
    d_hat_x = W2*h + b2;

    d_hat_hist(k) = d_hat_x;

    % =====================================================
    % NOMINAL CONTROL (PD + FF)
    % =====================================================
    u_nom = m*(ad - Kp*e - Kd*ed + [0;0;g]);
    
    u_nom_x = u_nom(1);

    % Inject NN compensation only in x
    u_ctrl = u_nom;
    u_ctrl(1) = u_nom_x - d_hat_x;

    u_des_hist(k,:) = u_ctrl';

    % =====================================================
    % ORIENTATION + THRUST FROM u_ctrl
    % =====================================================
    T_des = norm(u_ctrl);
    if T_des < 1e-9
        zB_des = [0;0;1];
    else
        zB_des = u_ctrl / T_des;
    end

    theta_d = asin(-zB_des(1));
    phi_d   = atan2(zB_des(2), zB_des(3));
    psi_d   = 0;

    % saturación
    phi_d   = max(min(phi_d, phi_max), -phi_max);
    theta_d = max(min(theta_d, theta_max), -theta_max);

    eta_d = [phi_d; theta_d; psi_d];

    tau = -Kp_att*(ang - eta_d) - Kd_att*omega;

    T_cmd = max(min(T_des, T_max), T_min);
    T_cmd_hist(k) = T_cmd;

    % =====================================================
    % MODEL INTEGRATION
    % =====================================================
    [~, xx] = ode45(@(tt,xx) quad_model_only(tt, xx, T_cmd, tau, m, I, g, Fdist), ...
                    [t(k) t(k+1)], xk);

    X(k+1,:) = xx(end,:)';

    % numerical acceleration for next step
    acc_prev = (X(k+1,4:6)' - X(k,4:6)')/ts;

    % =====================================================
    % NN TRAINING (BACKPROP RESIDUAL)
    % =====================================================
    xdd_meas = acc_prev(1);

    % real residual: Fdist = m*xdd - u_nom_x - u_NN
    u_NN_x = -d_hat_x;
    r_x = m*xdd_meas - u_nom_x - u_NN_x;

    r_x_hist(k) = r_x;

    % estimation error
    e_d = d_hat_x - r_x;

    % ============== Backprop ==============
    dh_dz1 = 1 - tanh(z1).^2;

    % Output layer
    dJ_dW2 = e_d * h';
    dJ_db2 = e_d;

    % Hidden layer
    dJ_dh  = W2' * e_d;
    dJ_dz1 = dJ_dh .* dh_dz1;

    dJ_dW1 = dJ_dz1 * s_x';
    dJ_db1 = dJ_dz1;

    % gradient update
    W2 = W2 - eta*dJ_dW2;
    b2 = b2 - eta*dJ_db2;
    W1 = W1 - eta*dJ_dW1;
    b1 = b1 - eta*dJ_db1;
end

% =======================================================================
% PLOTS
% =======================================================================

figure;
plot3(X(:,1),X(:,2),X(:,3),'b','LineWidth',1.5); hold on;
plot3(xd,yd,zd,'r--','LineWidth',1.5);
grid on; axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
title('Trajectory');

figure;
subplot(3,1,1); plot(t, X(:,1)-xd,'r'); ylabel('e_x'); grid on;
subplot(3,1,2); plot(t, X(:,2)-yd,'g'); ylabel('e_y'); grid on;
subplot(3,1,3); plot(t, X(:,3)-zd,'b'); ylabel('e_z'); xlabel('t'); grid on;

figure;
plot(t, d_hat_hist,'LineWidth',1.3); hold on;
plot(t, r_x_hist,'--','LineWidth',1.3);
legend('d\_hat','residual');
xlabel('t'); ylabel('force');
title('NN disturbance estimation');

figure;
plot(t, T_cmd_hist);
xlabel('t'); ylabel('T');
title('Thrust');

figure;
plot(t, Fdist_hist,'k','LineWidth',1.5); hold on;
plot(t, d_hat_hist,'r','LineWidth',1.5);
plot(t, r_x_hist,'b--','LineWidth',1.2);
legend('True disturbance','NN estimate','Residual');
xlabel('t [s]'); ylabel('Force [N]');
title('Wind disturbance and NN estimation');
grid on;

% =======================================================================
% MODEL (unchanged)
% =======================================================================
function dX = quad_model_only(~, X, T, tau, m, I, g, Fdist)

    pos   = X(1:3);
    vel   = X(4:6);
    ang   = X(7:9);       phi=ang(1); theta=ang(2); psi=ang(3);
    omega = X(10:12);

    cps = cos(psi); sps = sin(psi);
    cth = cos(theta); sth = sin(theta);
    cph = cos(phi);   sph = sin(phi);

    R = [ cps*cth,  cps*sth*sph - sps*cph,  cps*sth*cph + sps*sph;
          sps*cth,  sps*sth*sph + cps*cph,  sps*sth*cph - cps*sph;
          -sth   ,  cth*sph              ,  cth*cph               ];

    F_thrust_world = (R') * [0;0;T];

    acc = (1/m)*(F_thrust_world + Fdist) - [0;0;g];

    Wmat = [1 sph*tan(theta) cph*tan(theta);
            0 cph            -sph;
            0 sph/cth        cph/cth];
    ang_dot = Wmat*omega;

    omega_dot = I \ (tau - cross(omega, I*omega));

    dX = zeros(12,1);
    dX(1:3)   = vel;
    dX(4:6)   = acc;
    dX(7:9)   = ang_dot;
    dX(10:12) = omega_dot;
end
