
%�ڶ����� �ò����Ƕ��������������ָ���ƽ��е��ƹ��򻯴���
%������ǵ�һ����segment�������򻯺�ĵ��ƾ���ֱ���resolution������ʽ������TM  ����κ���X=a(1)T^2+b(1)T+c(1);Y=a(2)T^2+b(2)T+c(2),... TM=2
% ����segment�������ķָ�Ĵ��ڵ���number��segment����
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
    ZX=polyval(parameter(1,:),T); %������Ϻ�Ĺ���x��������(1xn)
    ZY=polyval(parameter(2,:),T);%������Ϻ�Ĺ���y��������(1xn)
    ZZ=polyval(parameter(3,:),T);%������Ϻ�Ĺ���z��������(1xn)
    Curve_feature_segment{i}=[ZX;ZY;ZZ]';
    else
        break
    end
end