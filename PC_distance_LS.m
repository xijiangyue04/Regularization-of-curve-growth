
%输入变量：input_pnts(nx3)  空间曲线拟合参数parameter=[C1;C2;C3] (3xM)  M=TM+1  TM为多项式的项数
%如：X=C1(1)*T^3+C1(2)*T^2+C1(3)*T+C1(4);Y=C2(1)*T^3+C2(2)*T^2+C2(3)*T+C2(4);Z=C3(1)*T^3+C3(2)*T^2+C3(3)*T+C3(4) 
%输出变量：PC_dis (nx1) 为点云到曲线的投影距离

function [PC_dis] = PC_distance_LS(input_pnts,parameter)

for i=1:size(input_pnts,1)
% 构建多项式表达式
X =@(T) polyval(parameter(1,:), T);
Y =@(T) polyval(parameter(2,:), T);
Z =@(T) polyval(parameter(3,:), T);

% 定义点P的坐标
x0 = input_pnts(i,1);
y0 = input_pnts(i,2);
z0 = input_pnts(i,3);

% 定义距离函数
distance = @(T) sqrt((X(T)-x0)^2 + (Y(T)-y0)^2 + (Z(T)-z0)^2);

% T0=0;
% T = fmincon(distance, T0, [], [], [], [], 0, 5);
T = fminbnd(distance, -200, 200);
new_X=polyval(parameter(1,:), T);
new_Y=polyval(parameter(2,:), T);
new_Z=polyval(parameter(3,:), T);
PC_dis(i,:)=sqrt((x0-new_X)^2+(y0-new_Y)^2+(z0-new_Z)^2);
end




