%% Simulator Skeleton File
% Paul Glotfelter
% 10/04/2016
% This file provides the bare-bones requirements for interacting with the
% Robotarium.  Note that this code won't actually run.  You'll have to
% insert your own algorithm!  If you want to see some working code, check
% out the 'examples' folder.
T = 0:0.033:60;
[t,v_hat] = ode45(@VD_GENE, T, [zeros(10,1);0.1;0.1]);
%% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = 7;%rb.get_available_agents();

% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
r = rb.set_number_of_agents(N).set_save_data(false).build();%r = rb.set_number_of_agents(N).set_save_data(false).set_show_figure(false).build();

% Select the number of iterations for the experiment.  This value is
% arbitrary

% Iterate for the previously specified number of iterations

% (PAUL) %


linearVelocityGain = 1; 
angularVelocityGain = pi/2;
formationControlGain = 4;
si_barrier_cert = create_si_barrier_certificate('SafetyRadius', 0.08);
si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', linearVelocityGain, ... 
    'AngularVelocityLimit', angularVelocityGain);

safety = 0.03;
lambda = 0.03;
unicycle_barrier_certificate = create_uni_barrier_certificate('SafetyRadius', safety, ... 
    'ProjectionDistance', lambda);
in=1;
k4=6;kT=0.4;
k1=10; k2=k1;k3=k1;k5=k1;k6=k1;
tol=1e-3;
iterations = length(T);
wrap = @(x) atan2(sin(x), cos(x));
%d12 = 0.2351; d13=0.3804;d23=d12;d14=d13;d15=d12;d34=d12;d45=d12;
radius = 0.15;
s1=sin(2*pi/5);c1=cos(2*pi/5);s2=sin(4*pi/5);c2=cos(pi/5);
a=0.4;d12 = a*sqrt(2*(1-c1)); d23=d12;d16=a;d15=d12;d26=a;d36=a;d34=d12;d46=a;d45=d12;
q_star = [radius -a*s1+radius -a*s2+radius a*s2+radius a*s1+radius;a a*c1 -a*c2 -a*c2 a*c1;pi/2*ones(1,5)];
sigma = 0.5;
ini = q_star+sigma*(rand(3,5));
%initial_conditions = [ini [sum(ini(1,:))/5;sum(ini(2,:))/5;sum(ini(3,:))/5] [radius;0;pi/2]];
initial_conditions=    0.9*[0.2140   -0.3092    0.1907    0.6743    .6533    0.1646    0.4
    0.9995    0.4351   -0.3093    0.0863    0.5769    0.3377         0.3377
    0.8727    0.8727    0.8727    0.8727    0.8727    0.8727   0.8727]-[1.1*ones(1,7);0.4*ones(1,7);zeros(1,7)];




args = {'PositionError', 0.03, 'RotationError', 0.1};
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

%load 'Velocity_Estimation'
 %load 'Cai_Estimation'
 %v_hat = zeros(2425,12);
for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds

    q = r.get_poses();
    
    %% Insert your code here!
    X7(t) = q(19);Y7(t)=q(20);
    Theta1(t) = q(3);Theta2(t) = q(6);Theta3(t) = q(9);Theta4(t) = q(12);Theta5(t) = q(15); Theta6(t) = q(18); 
    x1=q(1);x2=q(4);x3=q(7);x4=q(10);x5=q(13);y1=q(2);y2=q(5);y3=q(8);y4=q(11);y5=q(14);x6=q(16);y6=q(17); x7 = q(19);y7 = q(20);
    X1(t) =x1; Y1(t) = y1;X2(t) =x2; Y2(t) = y2;X3(t) =x3; Y3(t) = y3;X4(t) =x4; Y4(t) = y4;X5(t) =x5; Y5(t) = y5;X6(t)=x6;Y6(t)=y6;
    x12=x1-x2;x23=x2-x3;x15=x1-x5;x34=x3-x4;x45=x4-x5;y12=y1-y2;y23=y2-y3;y15=y1-y5;y34=y3-y4;y45=y4-y5;x16=x1-x6;y16=y1-y6;x26=x2-x6;y26=y2-y6;x36=x3-x6;y36=y3-y6;x46=x4-x6;y46=y4-y6;
    q12=[x12;y12];q23=[x23;y23];q15=[x15;y15];q34=[x34;y34];q45=[x45;y45];q16=[x16;y16];q26=[x26;y26];q36=[x36;y36];q46=[x46;y46];
    e12=norm(q12)-d12;
    e23=norm(q23)-d23;
    e15=norm(q15)-d15;
    e26=norm(q26)-d26;
    e16=norm(q16)-d16;
    e36=norm(q36)-d36;
    e46=norm(q46)-d46;
    e34=norm(q34)-d34;
    e45=norm(q45)-d45;
    E12(t) = e12; E23(t) = e23;E15(t) = e15;E34(t) = e34;E45(t) = e45;E16(t) = e16;E26(t) = e26;E36(t) = e36;E46(t) = e46;
    z12=e12*(e12+2*d12);z23=e23*(e23+2*d23);z16=e16*(e16+2*d16);z26=e26*(e26+2*d26);z36=e36*(e36+2*d36);z46=e46*(e46+2*d46);
    z15=e15*(e15+2*d15);z34=e34*(e34+2*d34);z45=e45*(e45+2*d45);
    Z=[z12;z15;z16;z23;z26;z34;z36;z45;z46];
    R=[x12 y12 -x12 -y12 0 0 0 0 0 0 0 0;x15 y15 0 0 0 0 0 0 -x15 -y15 0 0;x16 y16 0 0 0 0 0 0 0 0 -x16 -y16;0 0 x23 y23 -x23 -y23 0 0 0 0 0 0;0 0 x26 y26 0 0 0 0 0 0 -x26 -y26;...
        0 0 0 0 x34 y34 -x34 -y34 0 0 0 0;0 0 0 0 x36 y36 0 0 0 0 -x36 -y36;0 0 0 0 0 0 x45 y45 -x45 -y45 0 0;0 0 0 0 0 0 x46 y46 0 0 -x46 -y46];
    C = [cos(q(3)) 0 0 0 0 0;sin(q(3)) 0 0 0 0 0;0 cos(q(6)) 0 0 0 0;0 sin(q(6)) 0 0 0 0;0 0 cos(q(9)) 0 0 0;0 0 sin(q(9)) 0 0 0;...
        0 0 0 cos(q(12)) 0 0;0 0 0 sin(q(12)) 0 0;0 0 0 0 cos(q(15)) 0; 0 0 0 0 sin(q(15)) 0;0 0 0 0 0 cos(q(18)); 0 0 0 0 0 sin(q(18))];
    
    KKK1(t) = det(R*R');
    KKK2(t) = det(R*diag([ones(1,10) 0 0])*R');
    
        Adj = [0 1 0 0 1 1;                   % n x n adjacency matrix for the graph
       1 0 1 0 0 1;                   % Adj(i,j) == 0, if ith agent and jth
       0 1 0 1 0 1;                   % agent are not connected; Adj(i,j) ==
       0 0 1 0 1 1;                   % 1 if ith and jth agent are connected
       1 0 0 1 0 0;
       1 1 1 1 0 0];                  % Change Adj if the connecting 
                                    % topology needs to be changed
    L1 = diag([3,3,3,3,2,4]) - Adj;
    L = kron(L1,eye(2));
    Aio = kron([0,0,0,0,0,1],[1,1])';
    
    radius = 0.15;bb=0.3;
    qn=[x6;y6];
    qtx0(t) = q(19);qty0(t) = q(20);
    qT=[0.05*T(t)+qtx0(1);0.05*sin(T(t))+qty0(1)];%[x7;y7];%[radius*cos(bb*T(t));radius*sin(bb*T(t))];
    %q(19)=0.05*T(t)+qtx0(1);q(20) = 0.05*sin(T(t))+qty0(1);
    qdotT=[0.05;0.05*cos(T(t))];%[-radius*bb*sin(bb*T(t));radius*bb*cos(bb*T(t))];
    %qT=[0.05*cos(T(t));0.001*t];qdotT=[-0.05*sin(T(t));0.001];
    vd=kron(ones(6,1),qdotT);%qdotT-kT*(qn-qT);
    %v_hat(t,1:12) = kron(ones(1,6),[VThatx(t) VThaty(t)]);
    
    ua = -k4*R'*Z;
    
    
%     for i = 1:12
%         if ua(i)>=-10^(-2) && ua(i)<=10^(-2)
%             ua(i) = 0;
%         end
%     end
    %dvT = [0.05*(cos(atan2(0.05*cos(T(t)),0.05))+cos(T(t))*sin(atan2(0.05*cos(T(t)),0.05)));-sin(T(t))/(1+cos(T(t))^2)];
    %dvT = [0.05*cos(T(t))/sin(q(21));-sin(T(t))/(1+cos(T(t))^2)];
    eT=[q(19);q(20)]-qn-[0.1;0];
    
    %% Observer Design for eT
    beta = 0.1;
    if t == 1
        
        Adj = [0 1 0 0 1 1;                   % n x n adjacency matrix for the graph
       1 0 1 0 0 1;                   % Adj(i,j) == 0, if ith agent and jth
       0 1 0 1 0 1;                   % agent are not connected; Adj(i,j) ==
       0 0 1 0 1 1;                   % 1 if ith and jth agent are connected
       1 0 0 1 0 0;
       1 1 1 1 0 0];                  % Change Adj if the connecting 
                                    % topology needs to be changed
        L1 = diag([3,3,3,3,2,4]) - Adj;
        L = kron(L1,eye(2));
        Aio = kron([0,0,0,0,0,1],[1,1])';
        et1x(t) = eT(1);et1y(t) = eT(2);et2x(t) = eT(1);et2y(t) = eT(2);et3x(t) = eT(1);et3y(t) = eT(2);et4x(t) = eT(1);et4y(t) = eT(2);
        et5x(t) = eT(1);et5y(t) = eT(2);et6x(t) = eT(1);et6y(t) = eT(2);
        %E_hat = [eT;eT;eT;eT;eT;eT];
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
        
        
        Adj = [0 1 0 0 1 1;                   % n x n adjacency matrix for the graph
       1 0 1 0 0 1;                   % Adj(i,j) == 0, if ith agent and jth
       0 1 0 1 0 1;                   % agent are not connected; Adj(i,j) ==
       0 0 1 0 1 1;                   % 1 if ith and jth agent are connected
       1 0 0 1 0 0;
       1 1 1 1 0 0];                  % Change Adj if the connecting 
                                    % topology needs to be changed
        L1 = diag([3,3,3,3,2,4]) - Adj;
        L = kron(L1,eye(2));
        Aio = kron([0,0,0,0,0,1],[1,1])';
         
    dE_hat = -beta*sign(L*E_hat + Aio.*(E_hat - kron(ones(6,1),eT)));
    end
    %%
    e_T1{t} = [et1x(t);et1y(t)]-eT;e_T2{t} = [et2x(t);et2y(t)]-eT;e_T3{t} = [et3x(t);et3y(t)]-eT;e_T4{t} = [et4x(t);et4y(t)]-eT;
    e_T5{t} = [et5x(t);et5y(t)]-eT;e_T6{t} = [et6x(t);et6y(t)]-eT;
    en1(t)=norm(e_T1{t});en2(t)=norm(e_T2{t});en3(t)=norm(e_T3{t});en4(t)=norm(e_T4{t});en5(t)=norm(e_T5{t});en6(t)=norm(e_T6{t});
    et(t) = norm(eT);
    %%v_D = v_hat(t,1:12)' + kron(ones(6,1),(kT)*eT);
    v_D = v_hat(t,1:12)' + kT*E_hat;
    u=ua + v_D - diag([zeros(1,10) 1 1])*ua;
    u(11:12) = qdotT+kT*eT;
    
    

%     for j=1:12
%         if u(j)>=-tol && u(j)<=tol
%             u(j)=0;
%         end
%     end
    

    for i=1:6
        thetad(i)=atan2(u(2*i),u(2*i-1));
    end

    Thetad1(t) = thetad(1);Thetad2(t) = thetad(2);Thetad3(t) = thetad(3);Thetad4(t) = thetad(4);Thetad5(t) = thetad(5);Thetad6(t) = thetad(6);
    for i=1:6
        etheta(i)=wrap(q(3*i)-thetad(i));
    end
    etheta1=etheta(1);etheta2=etheta(2);etheta3=etheta(3);etheta4=etheta(4);etheta5=etheta(5);etheta6=etheta(6);

    E1(t)=etheta1;E2(t)=etheta2;E3(t)=etheta3;E4(t)=etheta4;E5(t)=etheta5;E6(t)=etheta6;
    u12=[u(1)-u(3) u(2)-u(4)];u16=[u(1)-u(11) u(2)-u(12)];u15=[u(1)-u(9) u(2)-u(10)];u26=[u(3)-u(11) u(4)-u(12)];u23=[u(3)-u(5) u(4)-u(6)];
    u34=[u(5)-u(7) u(6)-u(8)];u36=[u(5)-u(11) u(6)-u(12)];u45=[u(7)-u(9) u(8)-u(10)];u46=[u(7)-u(11) u(8)-u(12)];
    Rt_dot = [u12 -u12 0 0 0 0 0 0 0 0;u15 0 0 0 0 0 0 -u15 0 0;u16 0 0 0 0 0 0 0 0 -u16;0 0 u23 -u23 0 0 0 0 0 0;...
        0 0 u26 0 0 0 0 0 0 -u26;0 0 0 0 u34 -u34 0 0 0 0;0 0 0 0 u36 0 0 0 0 -u36;0 0 0 0 0 0 u45 -u45 0 0;...
        0 0 0 0 0 0 u46 0 0 -u46]';
    xdotn=u(11);
    ydotn=u(12);
    qdotn=[xdotn;ydotn];
    v_hat_dot = -beta*sign(L*v_hat(t,1:12)' + Aio.*(v_hat(t,1:12)' - vd));
    %v_hat_dot = kron(ones(6,1),kT*eT+K2*sign(eT));
    udota = -k4*(Rt_dot*Z+2*R'*R*u);
    %udot = udota+v_hat_dot+kron(ones(6,1),(kT)*(qdotT-qdotn));%+kT*v_hat(t,1:12)';
    udot = udota + v_hat_dot + kT*dE_hat - diag([zeros(1,10) 1 1])*udota;
    udot(11:12) = kT*(qdotT-u(11:12))+[0;-0.05*sin(T(t))];
    
    for i = 1:6
        tdd(i)=-(u(2*i)/(u(2*i-1)^2+u(2*i)^2))*udot(2*i-1)+(u(2*i-1)/(u(2*i-1)^2+u(2*i)^2))*udot(2*i);
    end
    tdd1=tdd(1);tdd2=tdd(2);tdd3=tdd(3);tdd4=tdd(4);tdd5=tdd(5);tdd6=tdd(6);
         

    v = C'*u ; 
    for j=1:6
        if v(j)>=0.1
            v(j)=0.1;
        elseif v(j)<=-0.1
            v(j)=-0.1;
        end
    end
    v1 = v(1);v2 = v(2);v3 = v(3);v4 = v(4);v5 = v(5); v6 = v(6); 

    W=[-k1*(etheta1)+tdd1;-k2*(etheta2)+tdd2;-k3*(etheta3)+tdd3;-k5*(etheta4)+tdd4;-k6*(etheta5)+tdd5;-k6*(etheta6)+tdd6];
    for j=1:6
        if W(j)>=6
            W(j)=6;
        elseif W(j)<=-6
            W(j)=-6;
        end
    end
    w1=W(1);w2=W(2);w3=W(3);w4=W(4);w5=W(5);w6=W(6);
    
    W1(t)=w1; W2(t)=w2; W3(t)=w3;W4(t)=w4;W5(t)=w5;W6(t)=w6;
    Tdd1(t)=tdd1; Tdd2(t)=tdd2; Tdd3(t)=tdd3;Tdd4(t)=tdd4;Tdd5(t)=tdd5;Tdd6(t)=tdd6;
    
    V1(t)=v1; V2(t)=v2; V3(t)=v3;V4(t)=v4;V5(t)=v5;;V6(t)=v6;
    u1=[v1;w1];u2=[v2;w2];u3=[v3;w3];u4=[v4;w4];u5=[v5;w5];u6=[v6;w6];
    %u1=[u(1);u(2)];u2=[u(3);u(4)];u3=[u(5);u(6)];u4=[u(7);u(8)];u5=[u(9);u(10)];
    Ux1(t)=ua(1);Uy1(t)=ua(2);Ux2(t)=ua(3);Uy2(t)=ua(4);Ux3(t)=ua(5);Uy3(t)=ua(6);Ux4(t)=ua(7);Uy4(t)=ua(8);Ux5(t)=ua(9);Uy5(t)=ua(10);Ux6(t)=ua(11);Uy6(t)=ua(12);
    U1(t) = norm([u(1) u(2)]);U2(t) = norm([u(3) u(4)]);U3(t) = norm([u(5) u(6)]);U4(t) = norm([u(7) u(8)]);U5(t) = norm([u(9) u(10)]);U6(t) = norm([u(11) u(12)]);
    U_error1(t) = norm([u(1)-v_hat(t,1) u(2)-v_hat(t,2)]);U_error2(t) = norm([u(3)-v_hat(t,3) u(4)-v_hat(t,4)]);
    U_error3(t) = norm([u(5)-v_hat(t,5) u(6)-v_hat(t,6)]);U_error4(t) = norm([u(7)-v_hat(t,7) u(8)-v_hat(t,8)]);
    U_error5(t) = norm([u(9)-v_hat(t,9) u(10)-v_hat(t,10)]);U_error6(t) = norm([u(11)-v_hat(t,11) u(12)-v_hat(t,12)]);
    
    v_T(t) = norm([v_hat(t,11);v_hat(t,12)]);
    
    kk = k1;
    thetadd = atan2(0.05*cos(T(t)),0.05);
    ethetad = q(21)-thetadd;
    tddd = -sin(T(t))/(1+cos(T(t))^2);
    wd = -kk*(ethetad)+tddd;
    dvT = [0.05*cos(q(21))+0.05*cos(T(t))*sin(q(21));wd];
    
    dq=[u1 u2 u3 u4 u5 u6 dvT];
    %dq=[[u(1);u(2)] [u(3);u(4)] [u(5);u(6)] [u(7);u(8)] [u(9);u(10)] [u(11);u(12)] [0.05;0.05*cos(T(t))]];
%      dq = si_barrier_cert(dq, q);
%      dq = si_to_uni_dyn(dq, q);  
    dq = unicycle_barrier_certificate(dq, q);
    
    
    %% Send velocities to agents
    
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dq);
    
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();
end
% figure
% plot(T,E12,T,E23,T,E15,T,E34,T,E45,T,E16,T,E26,T,E36,T,E46);legend('e_1_2','e_2_3','e_1_5','e_3_4','e_4_5','e_1_6','e_2_6','e_3_6','e_4_6');ylabel('e_i_j(m)');xlabel('Time(s)');grid;figure
% plot(T,E1,T,E2,T,E3,T,E4,T,E5,T,E6);legend({'$\tilde{\theta}_{1}$','$\tilde{\theta}_{2}$','$\tilde{\theta}_{3}$','$\tilde{\theta}_{4}$','$\tilde{\theta}_{5}$','$\tilde{\theta}_{6}$'},'Interpreter','latex');grid;ylabel('$$\tilde{\theta}_{i}{(rad)}$$','Interpreter','Latex');xlabel('Time(s)');figure
% plot(T,V1,T,V2,T,V3,T,V3,T,V4,T,V5,T,V6);legend('v1','v2','v3','v4','v5','v6');grid;figure
% plot(T,W1,T,W2,T,W3,T,W4,T,W5,T,W6);legend('w1','w2','w3','w4','w5','w6');grid
% figure
% subplot(2,3,1);plot(T,Ux1,T,Uy1);legend('Ux1','Uy1');grid
% subplot(2,3,2);plot(T,Ux2,T,Uy2);legend('Ux2','Uy2');grid
% subplot(2,3,2);plot(T,Ux2,T,Uy2);legend('Ux2','Uy2');grid
% subplot(2,3,3);plot(T,Ux3,T,Uy3);legend('Ux3','Uy3');grid
% subplot(2,3,4);plot(T,Ux4,T,Uy4);legend('Ux4','Uy4');grid
% subplot(2,3,5);plot(T,Ux5,T,Uy5);legend('Ux5','Uy5');grid
% subplot(2,3,6);plot(T,Ux6,T,Uy6);legend('Ux6','Uy6');grid
% figure
% plot(T,Thetad1,T,Thetad2,T,Thetad3,T,Thetad4,T,Thetad5,T,Thetad6);legend('thetad1','thetad2','thetad3','thetad4','thetad5','thetad6');grid;
% figure
% plot(T,Tdd1,T,Tdd2,T,Tdd3,T,Tdd4,T,Tdd5,T,Tdd6);legend('Tdd1','Tdd2','Tdd3','Tdd4','Tdd5','Tdd6');figure
% plot(T,Thetad1,T,Thetad2,T,Thetad3,T,Thetad4,T,Thetad5,T,Thetad6,T,Theta1,T,Theta2,T,Theta3,T,Theta4,T,Theta5,T,Theta6);legend('theta1','theta2','theta3','theta4','theta5','theta6');grid;
% figure
% plot(X1,Y1,X2,Y2,X3,Y3,X4,Y4,X5,Y5,X6,Y6,X7,Y7,'-r');axis equal;grid
% figure
% plot(T,en1,T,en2,T,en3,T,en4,T,en5,T,en6);grid;legend('et1','et2','et3','et4','et5','et6');
% figure
% plot(T,v_T);grid;
% figure; plot(T,et);grid;legend('eT');
% figure; plot(T,KKK1,T,KKK2);grid;legend('det(RRT)','det(RPRT)');
%Though we didn't save any data, we still should call r.call_at_scripts_end() after our
%experiment is over!
save('DATA_Target_Interception','et','en1','en2','en3','en4','en5','en6','E12','E23','E16','E26','E15','E34','E45','E36','E46','E1','E2','E3','E4','E5','E6','V1','V2','V3','V4','V5','V6','W1','W2','W3','W4','W5','W6')
r.call_at_scripts_end();
function [dv_hat] = VD_GENE(t,v_hat)
    beta = 0.05;
    Adj = [0 1 0 0 1 1;                   % n x n adjacency matrix for the graph
       1 0 1 0 0 1;                   % Adj(i,j) == 0, if ith agent and jth
       0 1 0 1 0 1;                   % agent are not connected; Adj(i,j) ==
       0 0 1 0 1 1;                   % 1 if ith and jth agent are connected
       1 0 0 1 0 0;
       1 1 1 1 0 0];                  % Change Adj if the connecting 
                                    % topology needs to be changed
    L1 = diag([3,3,3,3,2,4]) - Adj;
    L = kron(L1,eye(2));
    Aio = kron([0,0,0,0,0,1],[1,1])';
    vd1=[0.05;0.05*cos(t)];
    vd = @(t) kron(ones(6,1),vd1);
    dv_hat = -beta*sign(L*v_hat + Aio.*(v_hat - vd(t)));
end
