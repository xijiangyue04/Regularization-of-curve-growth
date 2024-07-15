
%第二步： 该步骤是对于曲线型特征分割点云进行点云规则化处理
%输入的是第一步的segment，及规则化后的点云距离分辨率resolution，多项式的项数TM  如二次函数X=a(1)T^2+b(1)T+c(1);Y=a(2)T^2+b(2)T+c(2),... TM=2
% 对于segment，保留的分割的大于等于number的segment点云
function  [Curve_feature_segment] = CFPR(segment,resolution,TM,number)  % Curve Feature point cloud regularization(FPR)

for i=1:length(segment)
    segment_n(i,:)=[size(segment{i},1),i];
end
    sort_segment_n=sortrows(segment_n,1,'descend');
    
 for i=1:length(segment)
     sort_segment{i}=segment{sort_segment_n(i,2)};
 end

for i=1:length(sort_segment)
    No=size(sort_segment{i},1);
    [range_resol] = range_resolut(sort_segment{i});
    if No>=number
    [parameter,~] = space_curve_LS(sort_segment{i},TM);
    pnts=sort_segment{i};
    n=size(pnts,1);    
    interval=resolution/range_resol;
    T=0:interval:(n-1)*1;
    ZX=polyval(parameter(1,:),T); %曲线拟合后的规则化x方向坐标(1xn)
    ZY=polyval(parameter(2,:),T);%曲线拟合后的规则化y方向坐标(1xn)
    ZZ=polyval(parameter(3,:),T);%曲线拟合后的规则化z方向坐标(1xn)
    Curve_feature_segment{i}=[ZX;ZY;ZZ]';
    else
        break
    end
end