%% Simulator Skeleton File
% Paul Glotfelter
% 10/04/2016
% This file provides the bare-bones requirements for interacting with the
% Robotarium.  Note that this code won't actually run.  You'll have to
% insert your own algorithm!  If you want to see some working code, check
% out the 'examples' folder.
T = 0:0.033:160;
[t,vT_hat] = ode45(@VD_GENE, T, [zeros(10,1);0.1;0.1]);
%% Get Robotarium object used to communicate with the robots/simulator
% rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = 7;%rb.get_available_agents();

% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
% r = rb.set_number_of_agents(N).set_save_data(false).build();%r = rb.set_number_of_agents(N).set_save_data(false).set_show_figure(false).build();
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);
% Select the number of iterations for the experiment.  This value is
% arbitrary

% Iterate for the previously specified number of iterations

% (PAUL) %


linearVelocityGain = 1; 
angularVelocityGain = pi/2;
formationControlGain = 4;
si_barrier_cert = create_si_barrier_certificate('SafetyRadius', 0.03);
si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', linearVelocityGain, ... 
    'AngularVelocityLimit', angularVelocityGain);

safety = 0.03;
lambda = 0.03;
unicycle_barrier_certificate = create_uni_barrier_certificate('SafetyRadius', safety, ... 
    'ProjectionDistance', lambda);

Kt = 0.4;
Ka = 10;
c = 10;
C = c*eye(N-1);
alpha = 0.1; % Velocity observer gain
beta = 0.5; % Target observer gain
tol = 1e-3;
eps = 1e-7;
iterations = length(T);
wrap = @(x) atan2(sin(x), cos(x));

fradius = 0.15;
radius = 0.05;
omega0 = 1;
s1=sin(2*pi/5);c1=cos(2*pi/5);s2=sin(4*pi/5);c2=cos(pi/5);
a=0.4;d12 = a*sqrt(2*(1-c1));

d_u = zeros(N-1,N-1);
d_u(1,2) = d12;
d_u(2,3) = d12;
d_u(1,5) = d12;
d_u(1,6) = a;
d_u(2,6) = a;
d_u(3,6) = a;
d_u(3,4) = d12;
d_u(4,6) = a;
d_u(4,5) = d12;
d = d_u + d_u';

q_star = [fradius -a*s1+fradius -a*s2+fradius a*s2+fradius a*s1+fradius;a a*c1 -a*c2 -a*c2 a*c1;pi/2*ones(1,5)];
sigma = 0.5;
% ini = q_star+sigma*(rand(3,5));
%initial_conditions = [ini [sum(ini(1,:))/5;sum(ini(2,:))/5;sum(ini(3,:))/5] [radius;0;pi/2]];
initial_conditions= 0.9*[0.2140   -0.3092    0.1907    0.6743    .6533    0.1646    0.4
    0.9995    0.4351   -0.3093    0.0863    0.5769    0.3377         0.3377
    0.8727    0.8727    0.8727    0.8727    0.8727    0.8727   0.8727]-[1.1*ones(1,7);0.4*ones(1,7);zeros(1,7)];

Adj = [0 1 0 0 1 1;                   % n x n adjacency matrix for the graph
       1 0 1 0 0 1;                   % Adj(i,j) == 0, if ith agent and jth
       0 1 0 1 0 1;                   % agent are not connected; Adj(i,j) ==
       0 0 1 0 1 1;                   % 1 if ith and jth agent are connected
       1 0 0 1 0 0;
       1 1 1 1 0 0];                  % Change Adj if the connecting 
                                    % topology needs to be changed


args = {'PositionError', 0.01, 'RotationError', 0.01};
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

%% 
E_output = zeros(iterations, 2*(N-1)-3);
V_output = zeros(iterations, (N-1));
V_hat_output = zeros(iterations, 2*(N-1));
V_tilde_output = zeros(iterations, 2*(N-1));
Omega_output = zeros(iterations, (N-1));
Theta_d_output = zeros(iterations, (N-1));
Theta_tilde_output = zeros(iterations, (N-1));
Theta_output = zeros(iterations, N);
X_output = zeros(iterations, N);
Y_output = zeros(iterations, N);
U_output = zeros(iterations, 2*N);
U_norm_output = zeros(iterations, N);

%load 'Velocity_Estimation'
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

    e = zeros(N-1,N-1);
    z = zeros(2*(N-1)-3,1);
    R = zeros(2*(N-1)-3,2*(N-1));
    ord = 0;
    for i = 1:N-1
        for j = i+1:N-1
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
    R0 = R;
    % Since N agent includes the target, so the size of rigidity matrix is
    % actually 2*(N-1)-3 by 2*(N-1).
    R0(:,2*(N-1)-1:2*(N-1)) = zeros(size(R,1),2);

    %%  
    D = diag(sum(Adj));
    L1 = D - Adj;
    L = kron(L1,eye(2));
    Aio = kron([0,0,0,0,0,1],[1,1])';

    pn = p(:,N-1);
    pT_init = [X_output(1,N);Y_output(1,N)];

    pT = pT_init+[radius*T(t);radius*sin(omega0*T(t))];
    vT1 = [radius;radius*omega0*cos(omega0*T(t))];

    vT = kron(ones(N-1,1),vT1);
    
    eT = pT - pn - [0.1;0];
    
    %% Observer Design for eT
    if t == 1
        et1x(t) = eT(1);et1y(t) = eT(2);et2x(t) = eT(1);et2y(t) = eT(2);et3x(t) = eT(1);et3y(t) = eT(2);et4x(t) = eT(1);et4y(t) = eT(2);
        et5x(t) = eT(1);et5y(t) = eT(2);et6x(t) = eT(1);et6y(t) = eT(2);
        E_hat = [zeros(10,1);eT];
        dE_hat = -beta*sign(L*E_hat + Aio.*(E_hat - kron(ones(6,1),eT)));
    else
        
        f1 = @(t,et1x,et2x,et3x,et4x,et5x,et6x) (-beta*sign(3*et1x-et2x-et5x-et6x));
        f2 = @(t,et1x,et2x,et3x,et4x,et5x,et6x) (-beta*sign(3*et2x-et1x-et3x-et6x));
        f3 = @(t,et1x,et2x,et3x,et4x,et5x,et6x) (-beta*sign(3*et3x-et2x-et4x-et6x));
        f4 = @(t,et1x,et2x,et3x,et4x,et5x,et6x) (-beta*sign(3*et4x-et3x-et5x-et6x));
        f5 = @(t,et1x,et2x,et3x,et4x,et5x,et6x) (-beta*sign(2*et5x-et1x-et4x));
        f6 = @(t,et1x,et2x,et3x,et4x,et5x,et6x) (-beta*sign(5*et6x-et1x-et2x-et3x-et4x-eT(1)));
        
        h = 0.01;
        k1f1 = h*f1(T(t-1),et1x(t-1),et2x(t-1),et3x(t-1),et4x(t-1),et5x(t-1),et6x(t-1));
        k1f2 = h*f2(T(t-1),et1x(t-1),et2x(t-1),et3x(t-1),et4x(t-1),et5x(t-1),et6x(t-1));
        k1f3 = h*f3(T(t-1),et1x(t-1),et2x(t-1),et3x(t-1),et4x(t-1),et5x(t-1),et6x(t-1));
        k1f4 = h*f4(T(t-1),et1x(t-1),et2x(t-1),et3x(t-1),et4x(t-1),et5x(t-1),et6x(t-1));
        k1f5 = h*f5(T(t-1),et1x(t-1),et2x(t-1),et3x(t-1),et4x(t-1),et5x(t-1),et6x(t-1));
        k1f6 = h*f6(T(t-1),et1x(t-1),et2x(t-1),et3x(t-1),et4x(t-1),et5x(t-1),et6x(t-1));
        k2f1 = h*f1(T(t-1)+h/2, et1x(t-1)+k1f1/2,et2x(t-1)+k1f2/2,et3x(t-1)+k1f3/2,et4x(t-1)+k1f4/2,et5x(t-1)+k1f5/2,et6x(t-1)+k1f6/2);
        k2f2 = h*f2(T(t-1)+h/2, et1x(t-1)+k1f1/2,et2x(t-1)+k1f2/2,et3x(t-1)+k1f3/2,et4x(t-1)+k1f4/2,et5x(t-1)+k1f5/2,et6x(t-1)+k1f6/2);
        k2f3 = h*f3(T(t-1)+h/2, et1x(t-1)+k1f1/2,et2x(t-1)+k1f2/2,et3x(t-1)+k1f3/2,et4x(t-1)+k1f4/2,et5x(t-1)+k1f5/2,et6x(t-1)+k1f6/2);
        k2f4 = h*f4(T(t-1)+h/2, et1x(t-1)+k1f1/2,et2x(t-1)+k1f2/2,et3x(t-1)+k1f3/2,et4x(t-1)+k1f4/2,et5x(t-1)+k1f5/2,et6x(t-1)+k1f6/2);
        k2f5 = h*f5(T(t-1)+h/2, et1x(t-1)+k1f1/2,et2x(t-1)+k1f2/2,et3x(t-1)+k1f3/2,et4x(t-1)+k1f4/2,et5x(t-1)+k1f5/2,et6x(t-1)+k1f6/2);
        k2f6 = h*f6(T(t-1)+h/2, et1x(t-1)+k1f1/2,et2x(t-1)+k1f2/2,et3x(t-1)+k1f3/2,et4x(t-1)+k1f4/2,et5x(t-1)+k1f5/2,et6x(t-1)+k1f6/2);
        k3f1 = h*f1(T(t-1)+h/2, et1x(t-1)+k2f1/2,et2x(t-1)+k2f2/2,et3x(t-1)+k2f3/2,et4x(t-1)+k2f4/2,et5x(t-1)+k2f5/2,et6x(t-1)+k2f6/2);
        k3f2 = h*f2(T(t-1)+h/2, et1x(t-1)+k2f1/2,et2x(t-1)+k2f2/2,et3x(t-1)+k2f3/2,et4x(t-1)+k2f4/2,et5x(t-1)+k2f5/2,et6x(t-1)+k2f6/2);
        k3f3 = h*f3(T(t-1)+h/2, et1x(t-1)+k2f1/2,et2x(t-1)+k2f2/2,et3x(t-1)+k2f3/2,et4x(t-1)+k2f4/2,et5x(t-1)+k2f5/2,et6x(t-1)+k2f6/2);
        k3f4 = h*f4(T(t-1)+h/2, et1x(t-1)+k2f1/2,et2x(t-1)+k2f2/2,et3x(t-1)+k2f3/2,et4x(t-1)+k2f4/2,et5x(t-1)+k2f5/2,et6x(t-1)+k2f6/2);
        k3f5 = h*f5(T(t-1)+h/2, et1x(t-1)+k2f1/2,et2x(t-1)+k2f2/2,et3x(t-1)+k2f3/2,et4x(t-1)+k2f4/2,et5x(t-1)+k2f5/2,et6x(t-1)+k2f6/2);
        k3f6 = h*f6(T(t-1)+h/2, et1x(t-1)+k2f1/2,et2x(t-1)+k2f2/2,et3x(t-1)+k2f3/2,et4x(t-1)+k2f4/2,et5x(t-1)+k2f5/2,et6x(t-1)+k2f6/2);
        k4f1 = h*f1(T(t-1)+h, et1x(t-1)+k3f1,et2x(t-1)+k3f2,et3x(t-1)+k3f3,et4x(t-1)+k3f4,et5x(t-1)+k3f5,et6x(t-1)+k3f6);
        k4f2 = h*f2(T(t-1)+h, et1x(t-1)+k3f1,et2x(t-1)+k3f2,et3x(t-1)+k3f3,et4x(t-1)+k3f4,et5x(t-1)+k3f5,et6x(t-1)+k3f6);
        k4f3 = h*f3(T(t-1)+h, et1x(t-1)+k3f1,et2x(t-1)+k3f2,et3x(t-1)+k3f3,et4x(t-1)+k3f4,et5x(t-1)+k3f5,et6x(t-1)+k3f6);
        k4f4 = h*f4(T(t-1)+h, et1x(t-1)+k3f1,et2x(t-1)+k3f2,et3x(t-1)+k3f3,et4x(t-1)+k3f4,et5x(t-1)+k3f5,et6x(t-1)+k3f6);
        k4f5 = h*f5(T(t-1)+h, et1x(t-1)+k3f1,et2x(t-1)+k3f2,et3x(t-1)+k3f3,et4x(t-1)+k3f4,et5x(t-1)+k3f5,et6x(t-1)+k3f6);
        k4f6 = h*f6(T(t-1)+h, et1x(t-1)+k3f1,et2x(t-1)+k3f2,et3x(t-1)+k3f3,et4x(t-1)+k3f4,et5x(t-1)+k3f5,et6x(t-1)+k3f6);
        
        et1x(t) = et1x(t-1) + (k1f1+2*k2f1+2*k3f1+k4f1)/6;
        et2x(t) = et2x(t-1) + (k1f2+2*k2f2+2*k3f2+k4f2)/6;
        et3x(t) = et3x(t-1) + (k1f3+2*k2f3+2*k3f3+k4f3)/6;
        et4x(t) = et4x(t-1) + (k1f4+2*k2f4+2*k3f4+k4f4)/6;
        et5x(t) = et5x(t-1) + (k1f5+2*k2f5+2*k3f5+k4f5)/6;
        et6x(t) = et6x(t-1) + (k1f6+2*k2f6+2*k3f6+k4f6)/6;
        
        g1 = @(t,et1y,et2y,et3y,et4y,et5y,et6y) (-beta*sign(3*et1y-et2y-et5y-et6y));
        g2 = @(t,et1y,et2y,et3y,et4y,et5y,et6y) (-beta*sign(3*et2y-et1y-et3y-et6y));
        g3 = @(t,et1y,et2y,et3y,et4y,et5y,et6y) (-beta*sign(3*et3y-et2y-et4y-et6y));
        g4 = @(t,et1y,et2y,et3y,et4y,et5y,et6y) (-beta*sign(3*et4y-et3y-et5y-et6y));
        g5 = @(t,et1y,et2y,et3y,et4y,et5y,et6y) (-beta*sign(2*et5y-et1y-et4y));
        g6 = @(t,et1y,et2y,et3y,et4y,et5y,et6y) (-beta*sign(5*et6y-et1y-et2y-et3y-et4y-eT(2)));
        
       
        k1g1 = h*g1(T(t-1),et1y(t-1),et2y(t-1),et3y(t-1),et4y(t-1),et5y(t-1),et6y(t-1));
        k1g2 = h*g2(T(t-1),et1y(t-1),et2y(t-1),et3y(t-1),et4y(t-1),et5y(t-1),et6y(t-1));
        k1g3 = h*g3(T(t-1),et1y(t-1),et2y(t-1),et3y(t-1),et4y(t-1),et5y(t-1),et6y(t-1));
        k1g4 = h*g4(T(t-1),et1y(t-1),et2y(t-1),et3y(t-1),et4y(t-1),et5y(t-1),et6y(t-1));
        k1g5 = h*g5(T(t-1),et1y(t-1),et2y(t-1),et3y(t-1),et4y(t-1),et5y(t-1),et6y(t-1));
        k1g6 = h*g6(T(t-1),et1y(t-1),et2y(t-1),et3y(t-1),et4y(t-1),et5y(t-1),et6y(t-1));
        k2g1 = h*g1(T(t-1)+h/2, et1y(t-1)+k1g1/2,et2y(t-1)+k1g2/2,et3y(t-1)+k1g3/2,et4y(t-1)+k1g4/2,et5y(t-1)+k1g5/2,et6y(t-1)+k1g6/2);
        k2g2 = h*g2(T(t-1)+h/2, et1y(t-1)+k1g1/2,et2y(t-1)+k1g2/2,et3y(t-1)+k1g3/2,et4y(t-1)+k1g4/2,et5y(t-1)+k1g5/2,et6y(t-1)+k1g6/2);
        k2g3 = h*g3(T(t-1)+h/2, et1y(t-1)+k1g1/2,et2y(t-1)+k1g2/2,et3y(t-1)+k1g3/2,et4y(t-1)+k1g4/2,et5y(t-1)+k1g5/2,et6y(t-1)+k1g6/2);
        k2g4 = h*g4(T(t-1)+h/2, et1y(t-1)+k1g1/2,et2y(t-1)+k1g2/2,et3y(t-1)+k1g3/2,et4y(t-1)+k1g4/2,et5y(t-1)+k1g5/2,et6y(t-1)+k1g6/2);
        k2g5 = h*g5(T(t-1)+h/2, et1y(t-1)+k1g1/2,et2y(t-1)+k1g2/2,et3y(t-1)+k1g3/2,et4y(t-1)+k1g4/2,et5y(t-1)+k1g5/2,et6y(t-1)+k1g6/2);
        k2g6 = h*g6(T(t-1)+h/2, et1y(t-1)+k1g1/2,et2y(t-1)+k1g2/2,et3y(t-1)+k1g3/2,et4y(t-1)+k1g4/2,et5y(t-1)+k1g5/2,et6y(t-1)+k1g6/2);
        k3g1 = h*g1(T(t-1)+h/2, et1y(t-1)+k2g1/2,et2y(t-1)+k2g2/2,et3y(t-1)+k2g3/2,et4y(t-1)+k2g4/2,et5y(t-1)+k2g5/2,et6y(t-1)+k2g6/2);
        k3g2 = h*g2(T(t-1)+h/2, et1y(t-1)+k2g1/2,et2y(t-1)+k2g2/2,et3y(t-1)+k2g3/2,et4y(t-1)+k2g4/2,et5y(t-1)+k2g5/2,et6y(t-1)+k2g6/2);
        k3g3 = h*g3(T(t-1)+h/2, et1y(t-1)+k2g1/2,et2y(t-1)+k2g2/2,et3y(t-1)+k2g3/2,et4y(t-1)+k2g4/2,et5y(t-1)+k2g5/2,et6y(t-1)+k2g6/2);
        k3g4 = h*g4(T(t-1)+h/2, et1y(t-1)+k2g1/2,et2y(t-1)+k2g2/2,et3y(t-1)+k2g3/2,et4y(t-1)+k2g4/2,et5y(t-1)+k2g5/2,et6y(t-1)+k2g6/2);
        k3g5 = h*g5(T(t-1)+h/2, et1y(t-1)+k2g1/2,et2y(t-1)+k2g2/2,et3y(t-1)+k2g3/2,et4y(t-1)+k2g4/2,et5y(t-1)+k2g5/2,et6y(t-1)+k2g6/2);
        k3g6 = h*g6(T(t-1)+h/2, et1y(t-1)+k2g1/2,et2y(t-1)+k2g2/2,et3y(t-1)+k2g3/2,et4y(t-1)+k2g4/2,et5y(t-1)+k2g5/2,et6y(t-1)+k2g6/2);
        k4g1 = h*g1(T(t-1)+h, et1y(t-1)+k3g1,et2y(t-1)+k3g2,et3y(t-1)+k3g3,et4y(t-1)+k3g4,et5y(t-1)+k3g5,et6y(t-1)+k3g6);
        k4g2 = h*g2(T(t-1)+h, et1y(t-1)+k3g1,et2y(t-1)+k3g2,et3y(t-1)+k3g3,et4y(t-1)+k3g4,et5y(t-1)+k3g5,et6y(t-1)+k3g6);
        k4g3 = h*g3(T(t-1)+h, et1y(t-1)+k3g1,et2y(t-1)+k3g2,et3y(t-1)+k3g3,et4y(t-1)+k3g4,et5y(t-1)+k3g5,et6y(t-1)+k3g6);
        k4g4 = h*g4(T(t-1)+h, et1y(t-1)+k3g1,et2y(t-1)+k3g2,et3y(t-1)+k3g3,et4y(t-1)+k3g4,et5y(t-1)+k3g5,et6y(t-1)+k3g6);
        k4g5 = h*g5(T(t-1)+h, et1y(t-1)+k3g1,et2y(t-1)+k3g2,et3y(t-1)+k3g3,et4y(t-1)+k3g4,et5y(t-1)+k3g5,et6y(t-1)+k3g6);
        k4g6 = h*g6(T(t-1)+h, et1y(t-1)+k3g1,et2y(t-1)+k3g2,et3y(t-1)+k3g3,et4y(t-1)+k3g4,et5y(t-1)+k3g5,et6y(t-1)+k3g6);
        
        et1y(t) = et1y(t-1) + (k1g1+2*k2g1+2*k3g1+k4g1)/6;
        et2y(t) = et2y(t-1) + (k1g2+2*k2g2+2*k3g2+k4g2)/6;
        et3y(t) = et3y(t-1) + (k1g3+2*k2g3+2*k3g3+k4g3)/6;
        et4y(t) = et4y(t-1) + (k1g4+2*k2g4+2*k3g4+k4g4)/6;
        et5y(t) = et5y(t-1) + (k1g5+2*k2g5+2*k3g5+k4g5)/6;
        et6y(t) = et6y(t-1) + (k1g6+2*k2g6+2*k3g6+k4g6)/6;
       
        E_hat = [et1x(t);et1y(t);et2x(t);et2y(t);et3x(t);et3y(t);et4x(t);et4y(t);et5x(t);et5y(t);eT]; 
        dE_hat = -beta*sign(L*E_hat + Aio.*(E_hat - kron(ones(6,1),eT)));
    end
    %%
%     e_T1{t} = [et1x(t);et1y(t)]-eT;e_T2{t} = [et2x(t);et2y(t)]-eT;e_T3{t} = [et3x(t);et3y(t)]-eT;e_T4{t} = [et4x(t);et4y(t)]-eT;
%     e_T5{t} = [et5x(t);et5y(t)]-eT;e_T6{t} = [et6x(t);et6y(t)]-eT;
%     en1(t)=norm(e_T1{t});en2(t)=norm(e_T2{t});en3(t)=norm(e_T3{t});en4(t)=norm(e_T4{t});en5(t)=norm(e_T5{t});en6(t)=norm(e_T6{t});
%     et(t) = norm(eT);
    
    %% 
    u_a = -Ka*R0'*z;
    %%v_D = v_hat(t,1:12)' + kron(ones(6,1),(kT)*eT);
%     v_D = v_hat(t,1:12)' + Kt*E_hat;
%     u = ua + v_D - diag([zeros(1,10) 1 1])*ua;
%     u(11:12) = qT_dot+Kt*eT;
    u = u_a + vT_hat(t,:)' + Kt*E_hat;
    u(11:12) = vT1 + Kt*eT;
    
    theta_tilde = zeros(N-1,1);
    v = zeros(N-1,1);
    for i = 1:N-1
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
    
    R_dot = zeros(2*(N-1)-3,2*(N-1));
    B = zeros(2*(N-1),2*(N-1));
    for i = 1:N-1
        B(2*i-1:2*i,2*i-1:2*i) = [(cos(theta_tilde(i)))^2 -(sin(2*theta_tilde(i)))/2; (sin(2*theta_tilde(i)))/2 (cos(theta_tilde(i)))^2];
    end
    ord = 0;
    for i = 1:N-1
        for j = i+1:N-1
            if Adj(i,j) == 1
                ord = ord+1;
                Bi = [(cos(theta_tilde(i)))^2 -(sin(2*theta_tilde(i)))/2; (sin(2*theta_tilde(i)))/2 (cos(theta_tilde(i)))^2];
                Bj = [(cos(theta_tilde(j)))^2 -(sin(2*theta_tilde(j)))/2; (sin(2*theta_tilde(j)))/2 (cos(theta_tilde(j)))^2];
                R_dot(ord,2*i-1:2*i) = (Bi*u(2*i-1:2*i)-Bj*u(2*j-1:2*j))'; 
                R_dot(ord,2*j-1:2*j) = (Bi*u(2*j-1:2*j)-Bj*u(2*i-1:2*i))'; 
            end
        end
    end
    R0_dot = R_dot;
    R0_dot(:,2*(N-1)-1:2*(N-1)) = zeros(size(R_dot,1),2);
    z_dot = 2*R*B*u;
    u_a_dot = -Ka*(R0_dot'*z+R0'*z_dot);

    vT_hat_dot = -alpha*sign(L*vT_hat(t,1:12)' + Aio.*(vT_hat(t,1:12)' - vT));
    % u_dot
    u_dot = u_a_dot + vT_hat_dot + Kt*dE_hat;
    vT1_dot = [0;-radius*omega0^2*sin(omega0*T(t))];
    eT_dot = vT1-B(11:12,11:12)*u(11:12);
    u_dot(11:12) = vT1_dot + Kt*eT_dot;
    
    H = [0 -1; 1 0];
    theta_d_dot = zeros(N-1,1);
    for i = 1:N-1
        if norm(u(2*i-1:2*i)) > eps
            theta_d_dot(i) = u(2*i-1:2*i)'*H'*u_dot(2*i-1:2*i)/norm(u(2*i-1:2*i))^2;
        end
    end
    
    omega = -C*theta_tilde + theta_d_dot;
    for j = 1:N-1
        if omega(j) >= 2*pi
            omega(j) = 2*pi;
        elseif omega(j) <= -2*pi
            omega(j) = -2*pi;
        end
    end
    Omega_output(t,:) = omega';
    
    V_output(t,:) = v';
    u_output = [v'; omega'];
    
%     v_T(t) = norm([v_hat(t,11);v_hat(t,12)]);
    %% Leader
    u_leader = vT1 + Kt*(pT-p(:,N));
    theta_d_leader = 0;
    if norm(u_leader) > eps
        theta_d_leader = atan2(u_leader(2), u_leader(1));
    end
%     theta_d_leader = atan2(radius*omega0*cos(omega0*T(t)),-radius*omega0*sin(omega0*T(t)));
    theta_tilde_leader = theta(N)-theta_d_leader;
    B_leader = [(cos(theta_tilde_leader))^2 -(sin(2*theta_tilde_leader))/2; (sin(2*theta_tilde_leader))/2 (cos(theta_tilde_leader))^2];
    eT_leader_dot = vT1 - B_leader*u_leader;
    u_leader_dot = vT1_dot + Kt*eT_leader_dot;
    theta_d_leader_dot = 0;
    if norm(u_leader) > eps
        theta_d_leader_dot = u_leader'*H'*u_leader_dot/norm(u_leader)^2;
    end
%     theta_d_leader_dot = omega0;
    w_leader = -c*theta_tilde_leader + theta_d_leader_dot;
%     if w_leader >= 2*pi
%             w_leader = 2*pi;
%     elseif w_leader <= -2*pi
%             w_leader = -2*pi;
%     end
    u_leader = [radius;radius*omega0*cos(omega0*T(t))];
    v_leader = norm(u_leader)*cos(theta_tilde_leader);
    dvT = [v_leader;w_leader];

%     dvT = [-radius*omega0*sin(omega0*T(t))*cos(q(21))+radius*omega0*cos(omega0*T(t))*sin(q(21));wd];
    

    dq = [u_output dvT];
    U_output(t,:) = reshape(dq,1,[]);
    dq = unicycle_barrier_certificate(dq, q);
    
    
    %% Send velocities to agents
    
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dq);
    
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();
end

%Though we didn't save any data, we still should call r.call_at_scripts_end() after our
%experiment is over!
% 'et','en1','en2','en3','en4','en5','en6'
save('DATA_Target_Interception_Local','E_output','Theta_tilde_output','V_output','Omega_output','V_hat_output','V_tilde_output','U_output','U_norm_output')
% r.call_at_scripts_end();
r.debug();
function [vT_hat_dot] = VD_GENE(t,vT_hat)
    alpha = 0.1;
    Adj = [0 1 0 0 1 1;                   % n x n adjacency matrix for the graph
           1 0 1 0 0 1;                   % Adj(i,j) == 0, if ith agent and jth
           0 1 0 1 0 1;                   % agent are not connected; Adj(i,j) ==
           0 0 1 0 1 1;                   % 1 if ith and jth agent are connected
           1 0 0 1 0 0;
           1 1 1 1 0 0];                  % Change Adj if the connecting 
                                          % topology needs to be changed
    D = diag(sum(Adj));
    L1 = D - Adj;
    L = kron(L1,eye(2));
    Aio = kron([0,0,0,0,0,1],[1,1])';
    radius = 0.05;
    omega0 = 1;
    N = 7;
    vT1=[radius;radius*omega0*cos(omega0*t)];
    vT = kron(ones(N-1,1),vT1);
    vT_hat_dot = -alpha*sign(L*vT_hat + Aio.*(vT_hat - vT));
end
