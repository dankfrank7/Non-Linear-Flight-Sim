% AERO3560 - Flight Mechanics - 1 - A3
% Author: Jasraj Chouhan 
%
% Function: PlotData(time,X,U)
%
% Info:
%   This function takes the state vector X and control vector from
%   Controls.m and plots the results against time.
%   
% Inputs:
%    X: State vector [13x1 Column vector]
%   	u   X(1)	Velocity component in x_b axis              	[m/s]    
%   	v   X(2)	Velocity component in y_b axis              	[m/s]              	 
%   	w   X(3)	Velocity component in z_b axis              	[m/s]
%   	p   X(4)	Body rate component in x_b axis (roll)      	[rad/s]
%   	q   X(5)	Body rate component in y_b axis (pitch)     	[rad/s]
%   	r   X(6)	Body rate component in z_b axis (yaw)       	[rad/s]
%   	q0  X(7)	Quaternion component 1
%   	q1  X(8)	Quaternion component 2
%   	q2  X(9)	Quaternion component 3
%   	q3  X(10)   Quaternion component 4
%   	xe   X(11)  X location in earth ref frame               	[m]
%   	ye   X(12)  Y location in earth ref frame               	[m]
%   	ze   X(13)  Z location in earth ref frame               	[m]
%
%    U: Vector that contains all aircraft control settings
%                   - de_t = U(1)           Throttle 
%                   - de_e = U(2)    (rad)  Elevator
%                   - de_a = U(3)    (rad)  Aileron
%                   - de_r = U(4)    (rad)  Rudder
%
%   Time: Time vector used for the entire simulation
%
%
% Outputs: 7 Figures
%           Figure 1: Position Figure for x,y,z vs time
%           Figure 2: Position Figure for x vs y vs z (3D)
%           Figure 3: Velocity Figure 
%           Figure 4: Body Rates Figure
%           Figure 5: Attitude Figure (Euler Angles)
%           Figure 6: Control Figure (Throttle, Elevator, Aileron, Rudder)
%           Figure 7: Quaternion Figure
% Required:
%   Input: State Vector, Control Vector, Time Vector.
%   Function: q2e - quaternions to euler angles
% Correct?: NOT TESTED
function PlotData(time,X,U) 
set(0, 'DefaultLineLineWidth', 2);
set(0,'defaultAxesFontSize',16)

%% Attitude Figure - in Euler Angles
figure(5);
Att = q2e(X(7:10,:)); %converting euler angles into degrees (q2e function already converts into degrees)
phi = Att(1,:); theta = Att(2,:); psi = Att(3,:); %Breaking apart Att vector into corresponding euler angles
plot(time,rad2deg(phi),'-k',time,rad2deg(theta),'--k',time,rad2deg(psi),'-.k');
xlabel('Time (s) ','Interpreter','Latex'); ylabel('Attitude $\circ$','Interpreter','Latex'); grid on; 
legend('\phi','\theta','\psi','location','best');
grid on
%% Create Position Figure with respect to time
figure(1);
xe = X(11,:); ye = X(12,:); ze = -X(13,:);
xe_2 = xe(1:100:end);ye_2 = ye(1:100:end);ze_2 = ze(1:100:end);
plot(time,xe,'-k',time,ye,'-.k',time,ze,'--k');
hold on 
xlabel('Time (s)','Interpreter','Latex'); ylabel('Position (m)','Interpreter','Latex');
legend('x_e','y_e','z_e')
grid minor;







V = zeros(1,length(time));
Alpha_vec = zeros(1,length(time));
Beta_vec = zeros(1,length(time));
for i = 1:length(time)
    [V_curr, Alpha, beta] = AeroAngles(X(:,i));
    V(i) = V_curr;
    Alpha_vec(i) = rad2deg(Alpha);
    Beta_vec(i) = rad2deg(beta);
end



%% Create Position Figure (3D)
figure(2);
plot3(xe,ye,ze,'-k');
hold on 
plot3(xe_2,ye_2,ze_2,'x','MarkerSize',12)
xlabel('Position x (m)','Interpreter','Latex'); ylabel('Position y (m)','Interpreter','Latex'); zlabel('Position z (m)','Interpreter','Latex');
hold on 
RaceCourse = 1;
while RaceCourse
    z = ones(1,8)*20';
    Vec = [15, 0;
    150, 0;
    400, -100;
    400, -350;
    271, -276;
    140, -200;
    -33, -100;
    -206.64512157586992, -0];
    plot3(Vec(:,1),Vec(:,2),z,'o','Linewidth',2,'Markersize',8);
    plot3([400,400,-206,400],[0,-350,0,0],[20,20,20,20],'--')
    RaceCourse = 0;
end
xer = xe+10*sin(psi);
yer = ye-10.*cos(psi);
zer = ze+10.*sin(phi);


xel = xe-10*sin(psi);
yel = ye+10.*cos(psi);
zel = ze-10.*sin(phi);
plot3(xer,yer,zer,'.r');
alpha(.5)
plot3(xel,yel,zel,'.g');
alpha(.5)
grid on 
axis equal
%% Create Velocity Figure with respect to time
figure(3);
%V = sqrt(X(1,:).^2 + X(2,:).^2 + X(3,:).^2); %Net V is the resultant vector of all components
plot(time,V,'-k');
alpha(.3)
hold on;
plot(time, X(1,:),'--k');
hold on;
plot(time, X(2,:),'-.k');
hold on;
plot(time, X(3,:),':k');
xlabel('Time (s)','Interpreter','Latex'); ylabel('Velocity (m/s)','Interpreter','Latex');
legend('Vnet','U','V','W','location','east');
grid on
%% Body Rates Figure
figure(4);
plot(time, rad2deg(X(4,:)),'-k');
hold on;
plot(time, rad2deg(X(5,:)),'--k');
hold on;
plot(time, rad2deg(X(6,:)),'-.k');
xlabel('$Time (s) $','Interpreter','Latex'); ylabel(' Body rate $(^\circ/s)$','Interpreter','Latex');
legend('p','q','r','location','best');
grid on

%% Control Figure 
figure(6);
de_t = U(1,:); de_e = U(2,:); de_a = U(3,:); de_r = U(4,:);   %Throttle, Elevator, Aileron, Rudder Control Settings
subplot(2,2,1);
plot(time,de_t,'-k');
xlabel('Time (s) ','Interpreter','Latex'); ylabel('$\delta_T$','Interpreter','Latex'); grid on; 
subplot(2,2,2);
plot(time,rad2deg(de_e),'-k');
xlabel('Time (s) ','Interpreter','Latex'); ylabel('$\delta_e$ (deg)','Interpreter','Latex'); grid on; 
subplot(2,2,3);
plot(time,rad2deg(de_a),'-k');
xlabel('Time (s) ','Interpreter','Latex'); ylabel('$\delta_a$ (deg)','Interpreter','Latex'); grid on; 
subplot(2,2,4);
plot(time,rad2deg(de_r),'-k');
xlabel('Time (s) ','Interpreter','Latex'); ylabel('$\delta_r$ (deg)','Interpreter','Latex'); grid on; 
%% Quaternion Plot just in case
figure(7);
plot(time,X(7,:),'-k');
hold on;
plot(time, X(8,:),'--k');
hold on;
plot(time, X(9,:),'-.k');
hold on;
plot(time, X(10,:),':k');
legend('q_{0}','q_{1}','q_{2}','q_{4}','location','best');
xlabel('Time (s)','Interpreter','Latex');
ylabel('Quaternion Dimensionless Value','Interpreter','Latex');
grid on
%% Aero Angles

figure()
plot(time,Alpha_vec,'-k','Linewidth',2)
hold on 
plot(time,Beta_vec,'--k','Linewidth',2)
legend('\alpha (deg)','\beta (deg)','location','best');
xlabel('Times (s)','Interpreter','Latex')
ylabel('Aero Angles (deg)','Interpreter','Latex')
grid on

end