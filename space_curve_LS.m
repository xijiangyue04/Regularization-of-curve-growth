%输入变量：input_pnts(nx3)  多项式的项数TM  如二次函数X=a(1)T^2+b(1)T+c(1);Y=a(2)T^2+b(2)T+c(2),... TM=2
%输出变量：parameter(3xTM) 第一行为X方向拟合参数，第二行为Y方向拟合参数，第三行为Z方向拟合参数; 拟合曲线上的点curvefit_pnts(nx3)
%mean_dis 对应点拟合偏差平均值
function [parameter,mean_dis] = space_curve_LS(input_pnts,TM)

for i=1:4
    if i<4
        pnts=sortrows(input_pnts,i,'descend');
    end
    if i==4
        pnts=input_pnts;
    end
X=pnts(:,1)';
Y=pnts(:,2)';
Z=pnts(:,3)';
%设置离散参数（维度与X,Y,Z相同）
n=size(pnts,1);
T=0:1:(n-1)*1;
%对X,Y,Z分别与T之间进行多项式拟合，这里多项式最高项为8次
C1=polyfit(T,X,TM);  %X方向曲线拟合参数 系数按降幂排列,如X=C1(1)*T^2+C1(2)*T+C1(3)
C2=polyfit(T,Y,TM);  %Y方向曲线拟合参数
C3=polyfit(T,Z,TM);   %Z方向曲线拟合参数
ZX=polyval(C1,T); %曲线拟合后的规则化x方向坐标(1xn)
ZY=polyval(C2,T);%曲线拟合后的规则化y方向坐标(1xn)
ZZ=polyval(C3,T);%曲线拟合后的规则化z方向坐标(1xn)
curvefit_pnts=[ZX;ZY;ZZ]';
fit_dis=sqrt(sum((curvefit_pnts-pnts).^2,2)); %nx1
mean_dis(i,:)=mean(fit_dis);
parameter_total{i}=[C1;C2;C3];
end
parameter=parameter_total{mean_dis==min(mean_dis)};






