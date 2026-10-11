close all; clear; clc;
h=figure(1);

% 初始化机械臂参数
franka = FrankaInitialization();
% 与CoppeliaSim建立连接
[clientID, vrep] = StartVrep(19999);
% 启用同步模式（可选，但通常更可靠）
vrep.simxSynchronous(clientID, true);
% 获取相关句柄
jointHandles = getHandles(clientID, vrep);

% centralData = centraldiskKinematics();
% [~, ~, ~, ~, ~] = vrep.simxCallScriptFunction(...
% clientID, 'Franka', ...
% vrep.sim_scripttype_childscript, ...
% 'setL3Pos_function', ...
% [],centralData, '', [], vrep.simx_opmode_blocking);

% 主控制循环
disp('开始控制循环...');
% 设置仿真步长和持续时间
Td = 8;        % 任务持续时间
tau = 0.01;    % 时间步长
t = 0:tau:Td;   % 时间向量
N = length(t);  % 时间步数

% 机器人参数
m = 2;          % 执行器数量 (空间二连杆)
n = 3;          % 任务空间维度 (X, Y, Z)

% 期望轨迹和实际轨迹
pd = zeros(n, N);       % 期望位置
pa = zeros(n, N);       % 实际位置
pa_dot = zeros(n, N);   % 实际速度
pd_dot = zeros(n, N);   % 实际速度
q_dot = zeros(m, N);    % 执行器速度
q = zeros(m, N);        % 执行器角度

% 初始执行器信号 (两个关节)
q0 = [0; 0]; % 单位: rad
L = 30 * 0.001; % 单位:mm

%初始化常数
q(:,1) = q0;
pa(:,1) = forwardKinematics(L,q(:,1));

J_hat = Jacobian(L,q0);

for k = 1:N
    tk = t(k);
    pd(:,k) = circularPath(tk);
end

figure;
clf;
plot3(pd(1,:),pd(2,:),pd(3,:));

k = 1;

while (vrep.simxGetConnectionId(clientID) ~= -1 && k < N)
    drawnow
    val_0=double(get(h,'CurrentCharacter'));
    if (val_0==98) % press 'b' to stop
        flag = 0;
        break
    end

    if k > 1
        pa_dot(:,k) = (pa(:,k) - pa(:,k-1)) / tau;
        pd_dot(:,k) = (pd(:,k) - pd(:,k-1)) / tau;
    else
        pa_dot(:,k) = zeros(n,1);
        pd_dot(:,k) = zeros(n,1);
    end

    q_dot(:,k) = pinv(J_hat) * pd_dot(:,k);
    q(:,k+1) = q(:,k) + tau * q_dot(:,k);
    J_hat = Jacobian(L,q(:,k+1));
    pa(:,k+1) = forwardKinematics(L,q(:,k+1));
    diskData = diskKinematics(q(:,k+1));
    [~, ~, ~, ~, ~] = vrep.simxCallScriptFunction(...
    clientID, 'Franka', ...
    vrep.sim_scripttype_childscript, ...
    'setJointPos_function', ...
    [],diskData, '', [], vrep.simx_opmode_blocking);
    k = k + 1;
end
    DeleteVrep(clientID, vrep)
    pa_dot(:,N) = (pa(:,N) - pa(:,N-1)) / tau;

    
%% 性能评估
% 跟踪误差
tracking_error = zeros(1, N);
for k = 1:N
    tracking_error(k) = norm(pd(:,k) - pa(:,k));
end

% MAE和MTE
MAE = mean(tracking_error);
MTE = max(tracking_error);

fprintf('平均绝对误差(MAE): %.6f mm\n', MAE);
fprintf('最大跟踪误差(MTE): %.6f mm\n', MTE);

%% 绘图
% 3D轨迹
figure(10)
clf
plot3(pd(1,:), pd(2,:), pd(3,:), 'r-', 'LineWidth', 1.5, 'DisplayName', '期望轨迹');
hold on;
% pzzzz = pa * 0.001;
% pzzzz = pa ;
% plot3(pzzzz(1,:), pzzzz(2,:), pzzzz(3,:), 'b--', 'LineWidth', 2, 'DisplayName', '实际轨迹');
plot3(pa(1,:), pa(2,:), pa(3,:), 'g--', 'LineWidth', 2, 'DisplayName', '实际轨迹');
% plot3(pa(:,1), pa(:,2), pa(:,3), 'bo', 'MarkerSize', 4, 'MarkerFaceColor', 'b', 'HandleVisibility','off');
% plot3(pa(1,:), pa(2,:), pa(3,:), 'bo', 'MarkerSize', 4, 'MarkerFaceColor', 'b', 'HandleVisibility','off');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D轨迹跟踪');
legend; grid on; 
axis equal; 
% xlim([-40 40])
% ylim([-40 40])
% zlim([0 80])
view(30, 30); % 设置视角

% 跟踪误差
figure(2)
clf
plot(t, tracking_error, 'k-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('跟踪误差 (m)');
title('跟踪误差随时间变化');
grid on;

% 各轴位置跟踪
figure(3)
clf
plot(t, pd(1,:), 'r-', 'LineWidth', 2); hold on;
plot(t, pa(1,:), 'b--', 'LineWidth', 1.5);
plot(t, pd(2,:), 'g-', 'LineWidth', 2);
plot(t, pa(2,:), 'm--', 'LineWidth', 1.5);
plot(t, pd(3,:), 'c-', 'LineWidth', 2);
plot(t, pa(3,:), 'k--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('位置 (m)');
title('各轴位置跟踪');
legend('X_d', 'X_a', 'Y_d', 'Y_a', 'Z_d', 'Z_a');
grid on;

% 执行器信号
figure(4)
clf
plot(t, q(1,:), 'LineWidth', 1.5, 'DisplayName', '关节 1');
hold on;
plot(t, q(2,:), 'LineWidth', 1.5, 'DisplayName', '关节 2');
xlabel('时间 (s)'); ylabel('执行器信号 (rad)');
title('执行器信号变化');
legend; grid on;

function J_hat = Jacobian(L,q)

    S = L;
    phi = q(1);
    if(q(2) == 0)
        theta = 1e-5;
    else
        theta = q(2);
    end   
    J_hat = [ (S*sin(phi)*(cos(theta) - 1))/theta, (S*cos(phi)*(cos(theta) + theta*sin(theta) - 1))/theta^2;
            -(S*cos(phi)*(cos(theta) - 1))/theta, (S*sin(phi)*(cos(theta) + theta*sin(theta) - 1))/theta^2;
                                               0,             -(S*(sin(theta) - theta*cos(theta)))/theta^2];
end

function p = forwardKinematics(L,q)
    S = L;
    phi = q(1);
    if(q(2) == 0)
        theta = 1e-5;
    else
        theta = q(2);
    end
    T = [sin(phi)^2+cos(phi)^2*cos(theta), cos(phi)*sin(phi)*(cos(theta)-1), cos(phi)*sin(theta), cos(phi)*(1-cos(theta))*S/(theta);
        cos(phi)*sin(phi)*(cos(theta)-1), cos(phi)^2+cos(theta)*sin(phi)^2, sin(phi)*sin(theta), sin(phi)*(1-cos(theta))*S/(theta);
        -cos(phi)*sin(theta), -sin(phi)*sin(theta), cos(theta), sin(theta)*S/(theta);
        0, 0, 0, 1];
    p = T(1:3,4); 
end
    
% Desired Trajectory
function p = circularPath(t)
    L = 30 *0.001;
    a = pi/4;
    b = pi/4;
    if( a*sin(t*b) == 0)
        q1 = 1e-5;
    else
        q1 = a*sin(t*b);
    end
    
    if( a*sin(t*b*2) == 0)
        q2 = 1e-5;
    else
        q2 = a*sin(t*b*2);
    end
    T = constant_curvature_model(L,q1,q2);
    p = T(1:3,4);

end

function T = constant_curvature_model(S,phi,theta)
  
    T = [sin(phi)^2+cos(phi)^2*cos(theta), cos(phi)*sin(phi)*(cos(theta)-1), cos(phi)*sin(theta), cos(phi)*(1-cos(theta))*S/(theta);
        cos(phi)*sin(phi)*(cos(theta)-1), cos(phi)^2+cos(theta)*sin(phi)^2, sin(phi)*sin(theta), sin(phi)*(1-cos(theta))*S/(theta);
        -cos(phi)*sin(theta), -sin(phi)*sin(theta), cos(theta), sin(theta)*S/(theta);
        0, 0, 0, 1];
end

function diskData = diskKinematics(q)
    numsegement = 5;
    S = 30 * 0.001;
    phi = q(1);
    if(q(2) == 0)
        theta = 1e-5;
    else
        theta = q(2);
    end   
    S_disk = S / numsegement;
    theta_disk = theta / numsegement;
    phi_disk = phi ;
    Tdisk = constant_curvature_model(S_disk,phi_disk,theta_disk);
    diskData = reshape(transpose(Tdisk),16,1);

end

function constantcurve = centraldiskKinematics ()
    numSegment = 10;
    L_central = 68*0.001;
    radius_tube =118*0.001;
    theta =L_central/radius_tube;
    L_disk = L_central / numSegment;
    theta_disk = theta / numSegment;
    if(theta_disk == 0)
        theta_disk = 1e-5;
    else
        theta_disk =theta / numSegment;
    end
    Tdisk = constant_curvature_model(L_disk,0,theta_disk);
    constantcurve = reshape(transpose(Tdisk),16,1);
end

function franka = FrankaInitialization()
% Define standard DH parameters for Franka Emika Panda
a = [0, 0, 0.0825, -0.0825, 0, 0.088, 0];
alpha = [-pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2, 0];
d = [0.333, 0, 0.316, 0, 0.384, 0, 0.107+0.3]; % tool offset
q0 = [0, 0, 0, 0, 0, 0, 0]; % Initial joint angles

% T = Link([theta d a alpha],'standard')
L(1)= Link([q0(1), d(1), a(1), alpha(1)],'standard');
L(2)= Link([q0(2), d(2), a(2), alpha(2)],'standard');
L(3)= Link([q0(3), d(3), a(3), alpha(3)],'standard');
L(4)= Link([q0(4), d(4), a(4), alpha(4)],'standard');
L(5)= Link([q0(5), d(5), a(5), alpha(5)],'standard');
L(6)= Link([q0(6), d(6), a(6), alpha(6)],'standard');
L(7)= Link([q0(7), d(7), a(7), alpha(7)],'standard');

franka = SerialLink(L);
franka.name = 'Franka Emika Panda';
% franka.plot(q0)
end
% 发送机械臂关节位置
function sendJointPosition(clientID, vrep, jointHandles, q)
% 设置每个关节的目标位置
for i = 1:length(jointHandles)
    % 注意：使用simx_opmode_streaming模式第一次发送，后续可以使用simx_opmode_oneshot或继续streaming
    vrep.simxSetJointTargetPosition(clientID, jointHandles(i), q(i), vrep.simx_opmode_streaming);
end
end
% 与CoppeliaSim建立连接
function [clientID, vrep] = StartVrep(porta)
% 初始化Remote API
disp('正在初始化CoppeliaSim Remote API...');
addpath vrep_lib/;
vrep = remApi('remoteApi');      % 使用remApi函数创建API对象
vrep.simxFinish(-1);             % 关闭任何已存在的连接
clientID = vrep.simxStart('127.0.0.1', porta, true, true, 5000, 5); % 连接CoppeliaSim服务器，默认端口19997

% 检查连接是否成
if clientID < 0
    disp('无法连接到CoppeliaSim仿真器！');
    disp('请确保CoppeliaSim正在运行，并且远程API服务已启用。');
    vrep.delete(); % 明确销毁对象
    return;
else
    disp('成功连接到CoppeliaSim仿真器！');
end
end
% 与CoppeliaSim断开连接
function DeleteVrep(clientID, vrep)
% vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait); % pause simulation
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait); % stop simulation
vrep.simxFinish(clientID);  % close the line if still open
vrep.delete(); % call the destructor!
disp('Program ended');
end
% 获取相关句柄
function jointHandles = getHandles(clientID, vrep)
% 获取Franka机械臂关节的句柄
jointNames = {'Franka_joint1', 'Franka_joint2', 'Franka_joint3', 'Franka_joint4', ...
    'Franka_joint5', 'Franka_joint6', 'Franka_joint7'};
jointHandles = zeros(1, length(jointNames));

for i = 1:length(jointNames)
    [returnCode, jointHandles(i)] = vrep.simxGetObjectHandle(clientID, jointNames{i}, vrep.simx_opmode_blocking);
    if returnCode ~= vrep.simx_return_ok
        fprintf('错误：无法获取关节 %s 的句柄！返回代码: %d\n', jointNames{i}, returnCode);
        vrep.simxFinish(clientID);
        vrep.delete();
        return;
    end
end

end

% MAE和MTE
MAE = mean(tracking_error);
MTE = max(tracking_error);

fprintf('平均绝对误差(MAE): %.6f mm\n', MAE);
fprintf('最大跟踪误差(MTE): %.6f mm\n', MTE);


