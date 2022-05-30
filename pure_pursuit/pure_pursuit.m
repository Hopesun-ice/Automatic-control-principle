%�趨����
clear;clc;
dt = 0.1; %[s] ʱ����
T = 100; %[s]������ʱ��

%�趨�ο�·��  Բ �պ�����
global r;
aplha = 0:pi/100:2*pi;
r = 1;
cx = r *cos(aplha);
cy = r *sin(aplha);
global max_index;
max_index = length(cx);

%%�趨��ʼ״̬
i = 1;
x = [];
y = [];
yaw = [];
global v;
v = 0.1; %����ĳ�ʼ״̬Ϊʲô����Ϊ����������
w = 0;
%��ʼλ���趨
x(1) = 0.8;
y(1) = 0; 
yaw(1) = 0;
global target_ind;%Ŀ���������
target_ind = 1;
last_target_ind = target_ind;
ind = 1;
%��������
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
% ��ͼ
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
    % �Ӹõ㿪ʼ��켣ǰ���������ҵ���Ԥ������������һ���켣��
    L_steps = 0;           % �ο��켣�ϼ������ڵ���ۼƾ���
    Ld = 0.01;       
    while L_steps < Ld && ind < max_index
        L_steps = L_steps +  sqrt((cx(ind+1)-cx(ind))^2 + (cy(ind+1)-cy(ind))^2);
        ind = ind+1;
    end
    ind_target = ind;
end


function [w] = pure_pursuit_control(x,y,yaw,v,cx,cy,target_ind)
    global r;
    %����Ŀ�����ȫ������ϵ�µĺ����
    th_target = atan2(cy(target_ind)-y, cx(target_ind)-x);
    
    %���㵱ǰ��������ȫ������ϵ�ĺ���ǣ�������Լ���Ҳ����ֱ�Ӳ��� yaw��
    
    %�����ڻ�����������ϵ�»����˾��������ĺ������
    e_th =  th_target - yaw;
    L =   sqrt((cx(target_ind)-x)^2 + (cy(target_ind)-y)^2);
    x = L * sin(e_th);
    
    %������������Բ���˶��뾶
    r = L*L/(2*x);
    w = v /r;
end

