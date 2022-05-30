%设定参数
clear;clc;
dt = 0.1; %[s] 时间间隔
T = 100; %[s]最大仿真时间

%设定参考路径  圆 闭合曲线
global r;
aplha = 0:pi/100:2*pi;
r = 1;
cx = r *cos(aplha);
cy = r *sin(aplha);
global max_index;
max_index = length(cx);

%%设定初始状态
i = 1;
x = [];
y = [];
yaw = [];
global v;
v = 0.1; %这里的初始状态为什么设置为矩阵向量呢
w = 0;
%初始位置设定
x(1) = 0.8;
y(1) = 0; 
yaw(1) = 0;
global target_ind;%目标点索引。
target_ind = 1;
last_target_ind = target_ind;
ind = 1;
%运行主题
while(target_ind < max_index)
    if((abs((sqrt(x(i)-cx(1))^2 + (y(i)-cy(1)^2))) > 0.01) && (target_ind ==1))
        target_ind = 1;
    else
        [target_ind] = calc_target_index(x(i),y(i),cx,cy,last_target_ind);
    end
    [w] = pure_pursuit_control(x(i),y(i),yaw(i),v,cx,cy,target_ind);
    x(i+1) = x(i) + v*dt * cos(yaw(i));
    y(i+1) = y(i) + v*dt * sin(yaw(i));
    yaw(i+1) = yaw(i)+ w * dt;
    i = i+1;
    last_target_ind = target_ind;
end
% 绘图
figure;
plot(cx,cy,'b'); hold on;
plot(x,y,'r','Linewidth',1);
legend('ref','fact');


%
function[ind_target] = calc_target_index(x,y,cx,cy,last_target_ind)
    global max_index;
   % N = max_index - last_target_ind;
    N = max_index;
    Distance = zeros(1,N);
    for i = 1:N
        Distance(i) =  sqrt((cx(i)-x)^2 + (cy(i)-y)^2); 
    end
    [~,ind] = min(Distance); 
    % 从该点开始向轨迹前方搜索，找到与预瞄距离最相近的一个轨迹点
    L_steps = 0;           % 参考轨迹上几个相邻点的累计距离
    Ld = 0.01;       
    while L_steps < Ld && ind < max_index
        L_steps = L_steps +  sqrt((cx(ind+1)-cx(ind))^2 + (cy(ind+1)-cy(ind))^2);
        ind = ind+1;
    end
    ind_target = ind;
end


function [w] = pure_pursuit_control(x,y,yaw,v,cx,cy,target_ind)
    global r;
    %计算目标点在全局坐标系下的航向角
    th_target = atan2(cy(target_ind)-y, cx(target_ind)-x);
    
    %计算当前机器人在全局坐标系的航向角（这个可以计算也可以直接采用 yaw）
    
    %计算在机器人人坐标系下机器人距离坐标点的横向误差
    e_th =  th_target - yaw;
    L =   sqrt((cx(target_ind)-x)^2 + (cy(target_ind)-y)^2);
    x = L * sin(e_th);
    
    %计算器机器人圆周运动半径
    r = L*L/(2*x);
    w = v /r;
end

