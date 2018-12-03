%% Simulator Skeleton File
% Paul Glotfelter
% 10/04/2016
% This file provides the bare-bones requirements for interacting with the
% Robotarium.  Note that this code won't actually run.  You'll have to
% insert your own algorithm!  If you want to see some working code, check
% out the 'examples' folder.

%% Get Robotarium object used to communicate with the robots/simulator
T = 0:.033:80;
opts = odeset('RelTol',1e-6,'AbsTol',1e-6);
[t,v_hat] = ode45(@VD_GENE, T, zeros(10,1), opts);

% rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = 5;%rb.get_available_agents();

% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
% r = rb.set_number_of_agents(N).set_save_data(false).build();%r = rb.set_number_of_agents(N).set_save_data(false).set_show_figure(false).build();
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);
% Select the number of iterations for the experiment.  This value is
% arbitrary

% Iterate for the previously specified number of iterations

% (PAUL) %


safety = 0.03;
lambda = 0.03;
unicycle_barrier_certificate = create_uni_barrier_certificate('SafetyRadius', safety, ... 
    'ProjectionDistance', lambda);
Ka = diag([6 6 6 6 5 5 6 6 6 6]);
alpha = 0.1;
% k1=10; k2=k1;k3=k1;k5=k1;k6=k1;
C = diag([10 10 10 10 10]);
tol = 1e-3;
eps = 1e-7;
%d12 = 0.2351; d13=0.3804;d23=d12;d14=d13;d15=d12;d34=d12;d45=d12;
s1=sin(2*pi/5);c1=cos(2*pi/5);s2=sin(4*pi/5);c2=cos(pi/5);

a = 0.25;
d12 = a*sqrt(2*(1-c1)); 
d13 = d12*sqrt(2*(1-cos(3*pi/5)));
d23 = d12;
d14 = d13;
d15 = d12;
d34 = d12;
d45 = d12;
d_u = zeros(N,N);
d_u(1,2) = d12;
d_u(1,3) = d13;
d_u(2,3) = d12;
d_u(1,4) = d13;
d_u(1,5) = d12;
d_u(3,4) = d12;
d_u(4,5) = d12;
d = d_u + d_u';

T=0:.033:80;
iterations = length(T);
wrap = @(x) atan2(sin(x), cos(x));
initial_conditions = [0.0188   -0.0769    0.0100    0.2000    0.3147; ...
                      0.2770    0.1824   -0.0283   -0.2023    0.0396 ; ...
                      0.0520    0.0061    0.1052    0.1695    0.1242]-[zeros(1,5);ones(1,5)*0.1;zeros(1,5)];

Adj = [0 1 1 1 1;                   % n x n adjacency matrix for the graph
       1 0 1 0 0;                   % Adj(i,j) == 0, if ith agent and jth
       1 1 0 1 0;                   % agent are not connected; Adj(i,j) ==
       1 0 1 0 1;                   % 1 if ith and jth agent are connected
       1 0 0 1 0]; 

args = {'PositionError', 0.03, 'RotationError', 0.01};
init_checker = create_is_initialized(args{:});
automatic_parker = create_automatic_parking_controller2(args{:});

x = zeros(3, N);

while(~init_checker(x, initial_conditions))

    x = r.get_poses();
    dxu = automatic_parker(x, initial_conditions);
    dxu = unicycle_barrier_certificate(dxu, x);     
    r.set_velocities(1:N, dxu);
    r.step();   
end

% (END PAUL) % 

%% Velocity Estimator 



% load 'Velocity_Estimation'

%% Output Values
    
E_output = zeros(iterations, 2*N-3);
V_output = zeros(iterations, N);
V_hat_output = zeros(iterations, 2*N);
V_tilde_output = zeros(iterations, 2*N);
Omega_output = zeros(iterations, N);
Theta_d_output = zeros(iterations, N);
Theta_tilde_output = zeros(iterations, N);
Theta_output = zeros(iterations, N);
X_output = zeros(iterations, N);
Y_output = zeros(iterations, N);
U_output = zeros(iterations, 2*N);
U_norm_output = zeros(iterations, N);

%% Iterations
for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds

    q = r.get_poses();
    
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
    % defined who has access to the information of vd
    Aio = kron([1,0,0,0,0],[1,1])';
    
    v_D = v_hat(t,1:10)'; % Velocity estimate
    
    radius = 0.15;
    omega0 = 0.3;
    vd1 = [-radius*omega0*sin(omega0*T(t));radius*omega0*cos(omega0*T(t))];
    vd = kron(ones(N,1),vd1);
    vddot = -alpha*sign(L*v_D + Aio.*(v_D - vd));
    
    V_hat_output(t,:) = v_D;
    V_tilde_output(t,:) = v_D - vd;
    
    u = -Ka*R'*z + v_D;
    U_output(t,:) = u';

    theta_tilde = zeros(N,1);
    u_output = zeros(3*N,1);
    v = zeros(N,1);
    for i = 1:N
        U_norm_output(t, i) = norm(u(2*i-1:2*i));
        if norm(u(2*i-1:2*i)) > eps
            ang_form = atan2(u(2*i),u(2*i-1));
        else
            ang_form = 0;
        end
        theta_tilde(i) = mod(theta(i)-ang_form, 2*pi);
        if theta_tilde(i) > pi
            theta_tilde(i) = theta_tilde(i) - 2*pi;
        end
        v(i) = norm(u(2*i-1:2*i))*cos(theta_tilde(i));
        % Saturation
        if v(i) >= 0.1
            v(i) = 0.1;
        elseif v(i) <= -0.1
            v(i) = -0.1;
        end
    end
    Theta_tilde_output(t,:) = theta_tilde';

    R_dot = zeros(2*N-3,2*N);
    B = zeros(2*N,2*N);
    for i = 1:N
        B(2*i-1:2*i,2*i-1:2*i) = [(cos(theta_tilde(i)))^2 -(sin(2*theta_tilde(i)))/2; (sin(2*theta_tilde(i)))/2 (cos(theta_tilde(i)))^2];
    end
    ord = 0;
    for i = 1:N
        for j = i+1:N
            if Adj(i,j) == 1
                ord = ord+1;
                Bi = [(cos(theta_tilde(i)))^2 -(sin(2*theta_tilde(i)))/2; (sin(2*theta_tilde(i)))/2 (cos(theta_tilde(i)))^2];
                Bj = [(cos(theta_tilde(j)))^2 -(sin(2*theta_tilde(j)))/2; (sin(2*theta_tilde(j)))/2 (cos(theta_tilde(j)))^2];
                R_dot(ord,2*i-1:2*i) = (Bi*u(2*i-1:2*i)-Bj*u(2*j-1:2*j))'; 
                R_dot(ord,2*j-1:2*j) = -(Bi*u(2*i-1:2*i)-Bj*u(2*j-1:2*j))'; 
            end
        end
    end
    z_dot = -2*R*B*Ka*R'*z;
    u_dot = -Ka*R_dot'*z -Ka*R'*z_dot + vddot;
    H = [0 -1; 1 0];
    theta_d_dot = zeros(N,1);
    for i = 1:N
        if norm(u(2*i-1:2*i)) > eps
            theta_d_dot(i) = u(2*i-1:2*i)'*H'*u_dot(2*i-1:2*i)/norm(u(2*i-1:2*i))^2;
        end
    end
    
    omega = -C*theta_tilde + theta_d_dot;
    for j = 1:N
        if omega(j) >= 2*pi
            omega(j) = 2*pi;
        elseif omega(j) <= -2*pi
            omega(j) = -2*pi;
        end
    end
    Omega_output(t,:) = omega';

    V_output(t,:) = v';
    u = [v'; omega'];
    dq = u; 
    dq = unicycle_barrier_certificate(dq, q);
    
    
    %% Send velocities to agents
    
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dq);
    
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();
end

save('DATA_Flocking','E_output','Theta_tilde_output','V_output','Omega_output','V_hat_output','V_tilde_output','U_output','U_norm_output');


% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.debug();
function [dv_hat] = VD_GENE(t,v_hat)
    Adj = [0 1 1 1 1;                   % n x n adjacency matrix for the graph
           1 0 1 0 0;                   % Adj(i,j) == 0, if ith agent and jth
           1 1 0 1 0;                   % agent are not connected; Adj(i,j) ==
           1 0 1 0 1;                   % 1 if ith and jth agent are connected
           1 0 0 1 0];                  % Change Adj if the connecting 
                                        % topology needs to be changed
    L1 = diag(sum(Adj)) - Adj;
    L = kron(L1,eye(2));
    Aio = kron([1,0,0,0,0],[1,1])';
    radius = 0.15;
    omega0=0.3;
    alpha = 0.1;
    vd1=[-radius*omega0*sin(omega0*t);
          radius*omega0*cos(omega0*t)];
    vd = kron(ones(5,1),vd1);
    dv_hat = -alpha*sign(L*v_hat + Aio.*(v_hat - vd));
end
