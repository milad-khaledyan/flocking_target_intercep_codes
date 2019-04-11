%% Flocking Control (5 Agents)
% Initially created on 03/01/19

% Milad Khaledyan
% Tairan Liu
% Victor Fernandez-Kim

% Source: https://github.com/milad-khaledyan/flocking_target_intercep_codes
% 02/09/19

%% Control Parameters
% Define controller specific parameters

N = length(IDs);

% Define timing variables
dt = ts.desiredStepTime(N);  % step time (Specific to TIGERSquare, do not modify)
T = 0:dt:100;                 % define total run time
dt_rate = 20;
iterations = length(T);

safety = 0.08;
lambda = 0.03;
unicycle_barrier_certificate = create_uni_barrier_certificate('SafetyRadius', safety, ... 
    'ProjectionDistance', lambda);
% Ka = diag([6 6 6 6 6 6 6 6 6 6]);
Ka = 1*eye(2*N);
alpha = 0.028;
% k1=10; k2=k1;k3=k1;k5=k1;k6=k1;
% C = diag([10 10 10 10 10]);
C = 2*eye(N);
tol = 1e-3;
eps = 1e-5;
theta_d_dot_bound = 0.15;

% Limit
v_limit = 0.035;
v_limit_per = 1;
w_limit = pi/4;
w_limit_per = 1;

%% Desired formation settings
ang_ind = 1:5;
ang = 2*pi/N*ang_ind;
rd = 0.20;%0.4;
pd = rd*[cos(ang);sin(ang)];
d = zeros(N,N);
for i = 1:N
    for j = 1:N
        d(i,j) = norm(pd(:,i)-pd(:,j));
    end
end

wrap = @(x) atan2(sin(x), cos(x));
% initial_conditions = [0.0188   -0.0769    0.0100    0.2000    0.3147; ...
%                       0.2770    0.1824   -0.0283   -0.2023    0.0396 ; ...
%                       0.0520    0.0061    0.1052    0.1695    0.1242]
% -[zeros(1,5);ones(1,5)*0.1;zeros(1,5)];
% Adj = [0 1 1 1 1;
scalem = diag([0.3 0.3]);
q_init = scalem*[1.2 0.5 -0.5 -0.5  0.5;
                   0 0.6  0.6 -0.6 -0.6]+[-0.5;0.5]*ones(1,N)+[1.4;0.35]*ones(1,N);
initial_conditions = [q_init; zeros(1,N)];
%        1 0 1 0 0;
%        1 1 0 1 0;
%        1 0 1 0 1;
%        1 0 0 1 0]; 
Adj = [0 1 0 0 1;
       1 0 1 0 1;
       0 1 0 1 1;
       0 0 1 0 1;
       1 1 1 1 0];
% defined who has access to the information of vd
%     Aio = kron([1,0,0,0,0],[1,1])';
    Aio = kron([1,1,0,0,1],[1,1])';
args = {'PositionError', 0.02, 'RotationError', 0.1};
init_checker = create_is_initialized(args{:});
automatic_parker = create_automatic_parking_controller2(args{:});

x = zeros(3, N);

while(~init_checker(x, initial_conditions))
    x = ts.getPoses(0);
    dxu = automatic_parker(x, initial_conditions);
    dxu = unicycle_barrier_certificate(dxu, x);     
    ts.setAllVelocities(dxu);
    ts.stepTS(0,toc,x,dxu);
end

%% Velocity Estimator 
radius = 0.25;%0.35;%0.15;
omega0 = 0.1;%0.06;%0.02;
paras.Adj = Adj;
paras.Aio = Aio;
paras.alpha = alpha;
paras.radius = radius;
paras.omega0 = omega0;

% opts = odeset('RelTol',1e-6,'AbsTol',1e-6);
% [t,v_hat] = ode45(@VD_GENE, T, zeros(10,1), opts,paras);

%% Output Values
    
E_output = zeros(iterations, 2*N-3);
V_output = zeros(iterations, N);
V_hat = zeros(iterations,2*N);
V_hat_output = zeros(iterations, 2*N);
V_tilde_output = zeros(iterations, 2*N);
V_tilde_error = zeros(iterations, N);
Omega_output = zeros(iterations, N);
Theta_d_output = zeros(iterations, N);
Theta_d_dot_output = zeros(iterations, N);
Theta_tilde_output = zeros(iterations, N);
Theta_output = zeros(iterations, N);
X_output = zeros(iterations, N);
Y_output = zeros(iterations, N);
U_output = zeros(iterations, 2*N);
U_norm_output = zeros(iterations, N);

%% Iterations
for t = 1:iterations
    
    q = ts.getPoses(0);
    
    %% Insert your code here!
    X_output(t, 1:N) = q(1:3:3*N);
    Y_output(t, 1:N) = q(2:3:3*N);
    Theta_output(t,1:N) = q(3:3:3*N);
    theta = Theta_output(t,:);
    p = [X_output(t,:); Y_output(t,:)];
    
    e = zeros(N,N);
    z = zeros(2*N-3,1);
    R = zeros(2*N-3,2*N);
    ord = 0;
    for i = 1:N
        for j = i+1:N
            e(i,j) = norm(p(:,i)-p(:,j)) - d(i,j);
            if Adj(i,j) == 1
                ord = ord + 1;
                z(ord) = e(i,j)*(e(i,j)+2*d(i,j));
                R(ord, 2*i-1:2*i) = (p(:,i)-p(:,j))';
                R(ord, 2*j-1:2*j) = (p(:,j)-p(:,i))';
                E_output(t, ord) = e(i,j);
            end
        end
    end
   
    %% Velocity estimate   
    D = diag(sum(Adj));
    L1 = D - Adj;
    L = kron(L1,eye(2));
    
%     v_hat_vec = v_hat(t,1:10)'; % Velocity estimate
    
    vd1 = [-radius*omega0*sin(omega0*T(t));radius*omega0*cos(omega0*T(t))];
    vd = kron(ones(N,1),vd1);
    h = dt;
    if t == 1
        v_hat = zeros(2*N,1);
        V_hat(t,:) = v_hat';
    else
%         v_hat = RK4(T(t-1), h, alpha, L, Aio, V_hat(t-1,:)');
        v_hat = RK4_interp(T(t-1), h, dt_rate, alpha, L, Aio, V_hat(t-1,:)');
        V_hat(t,:) = v_hat';
    end

    v_hat_vec = V_hat(t,:)';
    V_tilde_output(t,:) = (v_hat_vec-vd)';
    %v_hat_vec_dot = -alpha*sign(L*v_hat_vec + Aio.*(v_hat_vec - vd));
    
    v_hat_vec_dot = -alpha*tanh(L*v_hat_vec + Aio.*(v_hat_vec - vd));
    
    V_hat_output(t,:) = v_hat_vec;
    V_tilde_output(t,:) = v_hat_vec - vd;
    
    u = -Ka*R'*z + v_hat_vec;
    U_output(t,:) = u';

    theta_tilde = zeros(N,1);
    v = zeros(N,1);
    for i = 1:N
        U_norm_output(t, i) = norm(u(2*i-1:2*i));
        if norm(u(2*i-1:2*i)) > eps
            ang_form = atan2(u(2*i),u(2*i-1));
        else
            ang_form = 0;
        end
        Theta_d_output(t,i) = ang_form;
        theta_tilde(i) = mod(theta(i)-ang_form, 2*pi);
        if theta_tilde(i) > pi
            theta_tilde(i) = theta_tilde(i) - 2*pi;
        end
        v(i) = norm(u(2*i-1:2*i))*cos(theta_tilde(i));
        % Saturation
        if v(i) >= v_limit*v_limit_per
            v(i) = v_limit*v_limit_per;
        elseif v(i) <= -v_limit*v_limit_per
            v(i) = -v_limit*v_limit_per;
        end
    end
    Theta_tilde_output(t,:) = theta_tilde';

    R_dot = zeros(2*N-3,2*N);
    B = zeros(2*N,2*N);
    for i = 1:N
        B(2*i-1:2*i,2*i-1:2*i) = [(cos(theta_tilde(i)))^2 -(sin(2*theta_tilde(i)))/2; 
                                  (sin(2*theta_tilde(i)))/2 (cos(theta_tilde(i)))^2];
    end
    ord = 0;
    for i = 1:N
        for j = i+1:N
            if Adj(i,j) == 1
                ord = ord+1;
                Bi = [(cos(theta_tilde(i)))^2 -(sin(2*theta_tilde(i)))/2; 
                      (sin(2*theta_tilde(i)))/2 (cos(theta_tilde(i)))^2];
                Bj = [(cos(theta_tilde(j)))^2 -(sin(2*theta_tilde(j)))/2; 
                      (sin(2*theta_tilde(j)))/2 (cos(theta_tilde(j)))^2];
                R_dot(ord,2*i-1:2*i) = (Bi*u(2*i-1:2*i)-Bj*u(2*j-1:2*j))'; 
                R_dot(ord,2*j-1:2*j) = -(Bi*u(2*i-1:2*i)-Bj*u(2*j-1:2*j))'; 
            end
        end
    end

    z_dot = 2*R*B*u;
    u_dot = -Ka*R_dot'*z -Ka*R'*z_dot + v_hat_vec_dot;
    H = [0 -1; 1 0];
    theta_d_dot = zeros(N,1);
    for i = 1:N
        if norm(u(2*i-1:2*i)) > eps
            theta_d_dot(i) = u(2*i-1:2*i)'*H'*u_dot(2*i-1:2*i)/norm(u(2*i-1:2*i))^2;
        end
        
        if theta_d_dot(i) > theta_d_dot_bound
            
            theta_d_dot(i) = theta_d_dot_bound;
            
        elseif theta_d_dot(i) < -theta_d_dot_bound
            
            theta_d_dot(i) = -theta_d_dot_bound;
            
        end
    end
    Theta_d_dot_output(t,:) = theta_d_dot';
    
    omega = -C*theta_tilde + theta_d_dot;
    for j = 1:N
        if omega(j) >= w_limit*w_limit_per
            omega(j) = w_limit*w_limit_per;
        elseif omega(j) <= -w_limit*w_limit_per
            omega(j) = -w_limit*w_limit_per;
        end
    end
    Omega_output(t,:) = omega';

    V_output(t,:) = v';
    dq = [v'; omega']; 
    dq = unicycle_barrier_certificate(dq, q);
    
    
   %% Send velocities to agents
    ts.setAllVelocities(dq);
    ts.stepTS(1,toc,q,dq);
end

% save('DATA_Flocking','T','X_output','Y_output','Theta_output', ...
%      'E_output','Theta_d_output','Theta_tilde_output', ...
%      'V_output','Omega_output','V_hat_output','V_tilde_output','U_output','U_norm_output');

figure;
subplot(3,1,1); plot(T,E_output); grid on;
ylabel('Distance Error, $e_{ij}$ [m]','Interpreter','latex');
subplot(3,1,2); plot(T,Theta_tilde_output); grid on;
ylabel('Heading Angle Error, $\tilde{\theta_{i}}$ [rad]','Interpreter','latex');
subplot(3,1,3); plot(T,V_tilde_output); grid on;
ylabel('Velocity Estimation Error, $\tilde{v_{fi}}$ [m]','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');


function [value] = RK4_interp(t, h, rate, alpha, L, B, value_last)
    temp_value = zeros(10,rate+1);
    temp_value(:,1) = value_last;
    t_span = t + h/rate*[0:1:rate];
    for i = 1:rate
        temp_value(:,i+1) = RK4(t_span(i), h/rate, alpha, L, B, temp_value(:,i));
    end
    value = temp_value(:,rate+1);
end

function [value] = RK4(t, h, alpha, L, B, value_last)
    k1 = h*VelocityEstimator(t, alpha, L, B, value_last);
    k2 = h*VelocityEstimator(t+h/2, alpha, L, B, value_last+k1/2);
    k3 = h*VelocityEstimator(t+h/2, alpha, L, B, value_last+k2/2);
    k4 = h*VelocityEstimator(t+h, alpha, L, B, value_last+k3);
    value = value_last + (k1+2*k2+2*k3+k4)/6;
end

function [v_hat_dot] = VelocityEstimator(t, alpha, L, B, v_hat)
    radius = 0.25;
    omega0 = 0.1;
    N = 5;
    vd1 = [-radius*omega0*sin(omega0*t);radius*omega0*cos(omega0*t)];
    vd = kron(ones(N,1),vd1);
    v_hat_dot = -alpha*sign(L*v_hat + B.*(v_hat-vd));
end

function [dv_hat] = VD_GENE(t,v_hat, paras)
%     Adj = [0 1 1 1 1;                   % n x n adjacency matrix for the graph
%            1 0 1 0 0;                   % Adj(i,j) == 0, if ith agent and jth
%            1 1 0 1 0;                   % agent are not connected; Adj(i,j) ==
%            1 0 1 0 1;                   % 1 if ith and jth agent are connected
%            1 0 0 1 0];                  % Change Adj if the connecting 
%                                         % topology needs to be changed
%     Adj = [0 1 0 0 1;
%            1 0 1 0 1;
%            0 1 0 1 1;
%            0 0 1 0 1;
%            1 1 1 1 0];
    Adj = paras.Adj;
    Aio = paras.Aio;
    alpha = paras.alpha;
    radius = paras.radius;
    omega0 = paras.omega0;
    L1 = diag(sum(Adj)) - Adj;
    L = kron(L1,eye(2));
%     Aio = kron([1,0,0,0,0],[1,1])';
%     Aio = kron([1,1,0,0,1],[1,1])';
%     radius = 0.15;
%     omega0 = 0.3;
%     alpha = 0.1;
    vd1=[-radius*omega0*sin(omega0*t);
          radius*omega0*cos(omega0*t)];
    vd = kron(ones(5,1),vd1);
    dv_hat = -alpha*sign(L*v_hat + Aio.*(v_hat - vd));
end
