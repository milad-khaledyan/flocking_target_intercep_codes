%% Simulator Skeleton File
% Paul Glotfelter
% 10/04/2016
% This file provides the bare-bones requirements for interacting with the
% Robotarium.  Note that this code won't actually run.  You'll have to
% insert your own algorithm!  If you want to see some working code, check
% out the 'examples' folder.

%% Get Robotarium object used to communicate with the robots/simulator
T=0:.033:80;
[t,v_hat] = ode45(@VD_GENE, T, zeros(10,1));

rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = 5;%rb.get_available_agents();

% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
r = rb.set_number_of_agents(N).set_save_data(false).build();%r = rb.set_number_of_agents(N).set_save_data(false).set_show_figure(false).build();

% Select the number of iterations for the experiment.  This value is
% arbitrary

% Iterate for the previously specified number of iterations

% (PAUL) %


safety = 0.03;
lambda = 0.03;
unicycle_barrier_certificate = create_uni_barrier_certificate('SafetyRadius', safety, ... 
    'ProjectionDistance', lambda);
in=1;
k4=diag([6 6 6 6 5 5 6 6 6 6]);
k1=10; k2=k1;k3=k1;k5=k1;k6=k1;
tol=1e-3;
%d12 = 0.2351; d13=0.3804;d23=d12;d14=d13;d15=d12;d34=d12;d45=d12;
s1=sin(2*pi/5);c1=cos(2*pi/5);s2=sin(4*pi/5);c2=cos(pi/5);
a=0.1;d12 = a*sqrt(2*(1-c1)); d13=d12*sqrt(2*(1-cos(3*pi/5)));d23=d12;d14=d13;d15=d12;d34=d12;d45=d12;
T=0:.033:80;
iterations = length(T);
wrap = @(x) atan2(sin(x), cos(x));
initial_conditions = [0.0188   -0.0769    0.0100    0.2000    0.3147; ...
    0.2770    0.1824   -0.0283   -0.2023    0.0396 ; ...
    0.0520    0.0061    0.1052    0.1695    0.1242]-[zeros(1,5);ones(1,5)*0.1;zeros(1,5)];%-[ones(1,5)*0.4;zeros(2,5)];



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

%% Velocity Estimator 



% load 'Velocity_Estimation'

%%

    

  
    
for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds

    q = r.get_poses();
    
    %% Insert your code here!
    Theta1(t) = q(3);Theta2(t) = q(6);Theta3(t) = q(9);Theta4(t) = q(12);Theta5(t) = q(15);
    x1=q(1);x2=q(4);x3=q(7);x4=q(10);x5=q(13);y1=q(2);y2=q(5);y3=q(8);y4=q(11);y5=q(14);
    X1(t) =x1; Y1(t) = y1;X2(t) =x2; Y2(t) = y2;X3(t) =x3; Y3(t) = y3;X4(t) =x4; Y4(t) = y4;X5(t) =x5; Y5(t) = y5;
    x12=x1-x2;x13=x1-x3;x23=x2-x3;x14=x1-x4;x15=x1-x5;x34=x3-x4;x45=x4-x5;y12=y1-y2;y23=y2-y3;y13=y1-y3;y14=y1-y4;y15=y1-y5;y34=y3-y4;y45=y4-y5;
    q12=[x12;y12];q13=[x13;y13];q23=[x23;y23];q14=[x14;y14];q15=[x15;y15];q34=[x34;y34];q45=[x45;y45];
    e12=norm(q12)-d12;
    e23=norm(q23)-d23;
    e13=norm(q13)-d13;
    e14=norm(q14)-d14;
    e15=norm(q15)-d15;
    e34=norm(q34)-d34;
    e45=norm(q45)-d45;
    E12(t) = e12; E23(t) = e23;E13(t) = e13;E14(t) = e14;E15(t) = e15;E34(t) = e34;E45(t) = e45;
    z12=e12*(e12+2*d12);z23=e23*(e23+2*d23);z13=e13*(e13+2*d13);
    z14=e14*(e14+2*d14);z15=e15*(e15+2*d15);z34=e34*(e34+2*d34);z45=e45*(e45+2*d45);
    Z=[z12;z23;z13;z14;z15;z34;z45];
    R=[x12 y12 -x12 -y12 0 0 0 0 0 0;0 0 x23 y23 -x23 -y23 0 0 0 0;x13 y13 0 0 -x13 -y13 0 0 0 0;...
    x14 y14 0 0 0 0 -x14 -y14 0 0;x15 y15 0 0 0 0 0 0 -x15 -y15;0 0 0 0 x34 y34 -x34 -y34 0 0;0 0 0 0 0 0 x45 y45 -x45 -y45];
    C = [cos(q(3)) 0 0 0 0;sin(q(3)) 0 0 0 0;0 cos(q(6)) 0 0 0;0 sin(q(6)) 0 0 0;0 0 cos(q(9)) 0 0;0 0 sin(q(9)) 0 0;0 0 0 cos(q(12)) 0;0 0 0 sin(q(12)) 0;0 0 0 0 cos(q(15)); 0 0 0 0 sin(q(15))];
   
    
    radius = 0.15;bb=0.3;
    v_D=v_hat(t,1:10)';
    u=-k4*R'*Z+v_D;

    for j=1:10
        if u(j)>=-tol && u(j)<=tol
            u(j)=0;
        end
    end
    

    for i=1:5
        thetad(i)=atan2(u(2*i),u(2*i-1));
    end

    Thetad1(t) = thetad(1);Thetad2(t) = thetad(2);Thetad3(t) = thetad(3);Thetad4(t) = thetad(4);Thetad5(t) = thetad(5);
    for i=1:5
        etheta(i)=wrap(q(3*i)-thetad(i));
    end
    etheta1=etheta(1);etheta2=etheta(2);etheta3=etheta(3);etheta4=etheta(4);etheta5=etheta(5);

    E1(t)=etheta1;E2(t)=etheta2;E3(t)=etheta3;E4(t)=etheta4;E5(t)=etheta5;
    
    Rt_dot = [u(1)-u(3) 0 u(1)-u(5) u(1)-u(7) u(1)-u(9) 0 0;u(2)-u(4) 0 u(2)-u(6) u(2)-u(8) u(2)-u(10) 0 0;...
        u(3)-u(1) u(3)-u(5) 0 0 0 0 0;u(4)-u(2) u(4)-u(6) 0 0 0 0 0;0 u(5)-u(3) u(5)-u(1) 0 0 u(5)-u(7) 0;...
        0 u(6)-u(4) u(6)-u(2) 0 0 u(6)-u(8) 0;0 0 0 u(7)-u(1) 0 u(7)-u(5) u(9)-u(7);0 0 0 u(8)-u(2) 0 u(8)-u(6) u(8)-u(10);...
        0 0 0 0 u(9)-u(1) 0 u(9)-u(7);0 0 0 0 u(10)-u(2) 0 u(10)-u(8)];
    Adj = [0 1 1 1 1;                   % n x n adjacency matrix for the graph
       1 0 1 0 0;                   % Adj(i,j) == 0, if ith agent and jth
       1 1 0 1 0;                   % agent are not connected; Adj(i,j) ==
       1 0 1 0 1;                   % 1 if ith and jth agent are connected
       1 0 0 1 0];                  % Change Adj if the connecting 
                                    % topology needs to be changed
    L1 = diag([4,2,3,3,2]) - Adj;
    L = kron(L1,eye(2));
%% defined who has access to the information of vd
     Aio = kron([1,0,0,0,0],[1,1])';%diag([1,1,0,0,0,0,0,0,0,0]);%kron([1,0,0,0,0],[1,1])';
    
   radius = 0.15;bb=0.3;
   vd1=[-radius*bb*sin(bb*T(t));radius*bb*cos(bb*T(t))];
   vd =kron(ones(5,1),vd1);
   vddot = -0.05*sign(L*v_D + Aio.*(v_D - vd));
   udot = -k4*(Rt_dot*Z+2*R'*R*u)+vddot;
%    vddot = [-radius*bb*bb*sin(bb*T(t));radius*bb*bb*cos(bb*T(t))];
%    udot = -k4*(Rt_dot*Z+2*R'*R*u)+kron(ones(5,1),vddot);
    
    for i = 1:5
        tdd(i)=-(u(2*i)/(u(2*i-1)^2+u(2*i)^2))*udot(2*i-1)+(u(2*i-1)/(u(2*i-1)^2+u(2*i)^2))*udot(2*i);
    end
    tdd1=tdd(1);tdd2=tdd(2);tdd3=tdd(3);tdd4=tdd(4);tdd5=tdd(5);
    
         

    v = C'*u ; 
    for j=1:5
        if v(j)>=0.1
            v(j)=0.1;
        elseif v(j)<=-0.1
            v(j)=-0.1;
        end
    end
    v1 = v(1);v2 = v(2);v3 = v(3);v4 = v(4);v5 = v(5); 

    W=[-k1*(etheta1)+tdd1;-k2*(etheta2)+tdd2;-k3*(etheta3)+tdd3;-k5*(etheta4)+tdd4;-k6*(etheta5)+tdd5];
    for j=1:5
        if W(j)>=6.2832
            W(j)=6.2832;
        elseif W(j)<=-6.2832
            W(j)=-6.2832;
        end
    end
    w1=W(1);w2=W(2);w3=W(3);w4=W(4);w5=W(5);
    
    W1(t)=w1; W2(t)=w2; W3(t)=w3;W4(t)=w4;W5(t)=w5;
    Tdd1(t)=tdd1; Tdd2(t)=tdd2; Tdd3(t)=tdd3;Tdd4(t)=tdd4;Tdd5(t)=tdd5;
    
    V1(t)=v1; V2(t)=v2; V3(t)=v3;V4(t)=v4;V5(t)=v5;
    u1=[v1;w1];u2=[v2;w2];u3=[v3;w3];u4=[v4;w4];u5=[v5;w5];
    %u1=[u(1);u(2)];u2=[u(3);u(4)];u3=[u(5);u(6)];u4=[u(7);u(8)];u5=[u(9);u(10)];
    Ux1(t)=u(1);Uy1(t)=u(2);Ux2(t)=u(3);Uy2(t)=u(4);Ux3(t)=u(5);Uy3(t)=u(6);Ux4(t)=u(7);Uy4(t)=u(8);Ux5(t)=u(9);Uy5(t)=u(10);

    dq=[u1 u2 u3 u4 u5];
%     dq = si_barrier_cert(dq, q);
%     dq = si_to_uni_dyn(dq, q);  
    dq = unicycle_barrier_certificate(dq, q);
    
    
    %% Send velocities to agents
    
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dq);
    
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();
end
% figure
% plot(T,E12,T,E23,T,E13,T,E14,T,E15,T,E34,T,E45);legend('e_1_2','e_2_3','e_1_3','e_1_4','e_1_5','e_3_4','e_4_5');ylabel('e_i_j(m)');xlabel('Time(s)');figure
% plot(T,E1,T,E2,T,E3,T,E4,T,E5);legend({'$\tilde{\theta}_{1}$','$\tilde{\theta}_{2}$','$\tilde{\theta}_{3}$','$\tilde{\theta}_{4}$','$\tilde{\theta}_{5}$'},'Interpreter','latex');ylabel('$$\tilde{\theta}_{i}{(rad)}$$','Interpreter','Latex');xlabel('Time(s)');figure
% plot(T,V1,T,V2,T,V3,T,V3,T,V4,T,V5);legend('v1','v2','v3','v4','v5');figure
% plot(T,W1,T,W2,T,W3,T,W4,T,W5);legend('w1','w2','w3','w4','w5')
% figure
% subplot(2,3,1);plot(T,Ux1,T,Uy1);legend('Ux1','Uy1')
% subplot(2,3,2);plot(T,Ux2,T,Uy2);legend('Ux2','Uy2')
% subplot(2,3,3);plot(T,Ux3,T,Uy3);legend('Ux3','Uy3')
% subplot(2,3,4);plot(T,Ux4,T,Uy4);legend('Ux4','Uy4')
% subplot(2,3,5);plot(T,Ux5,T,Uy5);legend('Ux5','Uy5')
% subplot(2,3,6);plot(T,Thetad1,T,Thetad2,T,Thetad3,T,Thetad4,T,Thetad5);legend('thetad1','thetad2','thetad3','thetad4','thetad5')
% figure
% plot(T,Tdd1,T,Tdd2,T,Tdd3,T,Tdd4,T,Tdd5);legend('Tdd1','Tdd2','Tdd3','Tdd4','Tdd5','bb');figure
% plot(T,Thetad1,T,Thetad2,T,Thetad3,T,Thetad4,T,Thetad5,T,Theta1,T,Theta2,T,Theta3,T,Theta4,T,Theta5);legend('theta1','theta2','theta3','theta4','theta5');
% figure
% plot(X1,Y1,X2,Y2,X3,Y3,X4,Y4,X5,Y5);axis equal;grid
save('DATA_Flocking','E12','E23','E13','E14','E15','E34','E45','E1','E2','E3','E4','E5','V1','V2','V3','V4','V5','W1','W2','W3','W4','W5')


% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();
function [dv_hat] = VD_GENE(t,v_hat)
Adj = [0 1 1 1 1;                   % n x n adjacency matrix for the graph
       1 0 1 0 0;                   % Adj(i,j) == 0, if ith agent and jth
       1 1 0 1 0;                   % agent are not connected; Adj(i,j) ==
       1 0 1 0 1;                   % 1 if ith and jth agent are connected
       1 0 0 1 0];                  % Change Adj if the connecting 
                                    % topology needs to be changed
L1 = diag([4,2,3,3,2]) - Adj;
L = kron(L1,eye(2));
Aio = kron([1,0,0,0,0],[1,1])';
radius = 0.15;bb=0.3;
vd1=[-radius*bb*sin(bb*t);radius*bb*cos(bb*t)];
vd = @(t) kron(ones(5,1),vd1);
dv_hat = -0.05*sign(L*v_hat + Aio.*(v_hat - vd(t)));
end
