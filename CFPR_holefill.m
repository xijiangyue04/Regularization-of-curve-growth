%�������� �ò����Ƕ��ڷָ�������ߵ��ƽ������Ӳ���,resolution�޲���������������ֱ��ʣ�CP_dis��ͬ�����߶�Ӧ�������ֵ

function  [insert_pnts1,insert_pnts2] = CFPR_holefill(Curve_feature_segment,resolution,CP_dis)

for i=1:length(Curve_feature_segment)
    [endpnts] = Curve_endpnts(Curve_feature_segment{i});
    curve_start(i,:)=endpnts(1,:);
    curve_end(i,:)=endpnts(2,:);
end


for i=1:length(Curve_feature_segment)
    remain_start=curve_start(i+1:end,:); %��i=length(Curve_feature_segment)ʱ��Ϊ�ռ�
    remain_end=curve_end(i+1:end,:);
    if ~isempty(remain_start)
    st_dis1=sqrt(sum((curve_start(i,:)-remain_start).^2,2)); %�����ʣ�����ľ��� ��Сֵ
    st_dis2=sqrt(sum((curve_start(i,:)-remain_end).^2,2)); %�����ʣ���յ�ľ��� ��Сֵ
    end_dis1=sqrt(sum((curve_end(i,:)-remain_start).^2,2)); %�յ���ʣ�����ľ��� ��Сֵ
    end_dis2=sqrt(sum((curve_end(i,:)-remain_end).^2,2)); %�յ���ʣ���յ�ľ��� ��Сֵ
    dist=[st_dis1, st_dis2,end_dis1,end_dis2];
    [min_a,index]=min(dist,[],1); %����dist����ÿ�е���Сֵ����Ӧ���к�
    min_index=[min_a;index;1:4]';%��һ���Ƕ�Ӧ���룬�ڶ��ж�Ӧ���������߱�ǩ�ţ������ж�Ӧ��㣬�յ��Ӧ�����ԣ�����1�����-���:2�����-�յ㣬3���յ�-��㣬4���յ�-�յ�
    sort_min=sortrows(min_index,1);
    correspond_dist=sort_min(1:2,1);%���������������������߶˵��Ӧ��������С����
    
    if correspond_dist(1)<CP_dis %��Ӧ�ĵ�һ����������С������������
        Nearest_index1=sort_min(1,2); %��Ӧ�ĵ�һ�������߱�ǩ��
        correspond_index=sort_min(1,3);%��Ӧ�ĵ�һ���������������ֶ�Ӧ
        if correspond_index==1
             correspond_pnt=[curve_start(i,:);remain_start(Nearest_index1,:)];
              insert_pnts1{i} = interpolation_pnts(correspond_pnt(1,:),correspond_pnt(2,:),resolution);
        end
        
        if correspond_index==2
            correspond_pnt=[curve_start(i,:);remain_end(Nearest_index1,:)];
             insert_pnts1{i} = interpolation_pnts(correspond_pnt(1,:),correspond_pnt(2,:),resolution);
        end
        
        if  correspond_index==3
            correspond_pnt=[curve_end(i,:);remain_start(Nearest_index1,:)];
            insert_pnts1{i} = interpolation_pnts(correspond_pnt(1,:),correspond_pnt(2,:),resolution);
        end
        
        if  correspond_index==4
            correspond_pnt=[curve_end(i,:);remain_end(Nearest_index1,:)];
            insert_pnts1{i} = interpolation_pnts(correspond_pnt(1,:),correspond_pnt(2,:),resolution);
        end
    end
    
     if correspond_dist(2)<CP_dis %��Ӧ�ĵڶ�����������С������������
        Nearest_index2=sort_min(2,2); %��Ӧ�ĵڶ��������߱�ǩ��
        if Nearest_index2==Nearest_index1
            insert_pnts2{i}=[];
        else
        correspond_index=sort_min(2,3);%��Ӧ�ĵڶ����������������ֶ�Ӧ
        
        if correspond_index==1
             correspond_pnt=[curve_start(i,:);remain_start(Nearest_index2,:)];
              insert_pnts2{i} = interpolation_pnts(correspond_pnt(1,:),correspond_pnt(2,:),resolution);
        end
        
        if correspond_index==2
              correspond_pnt=[curve_start(i,:);remain_end(Nearest_index2,:)];
              insert_pnts2{i} = interpolation_pnts(correspond_pnt(1,:),correspond_pnt(2,:),resolution);
        end
        
        if correspond_index==3
              correspond_pnt=[curve_end(i,:);remain_start(Nearest_index2,:)];
              insert_pnts2{i} = interpolation_pnts(correspond_pnt(1,:),correspond_pnt(2,:),resolution);
        end
        if correspond_index==4
             correspond_pnt=[curve_end(i,:);remain_end(Nearest_index2,:)];
              insert_pnts2{i} = interpolation_pnts(correspond_pnt(1,:),correspond_pnt(2,:),resolution);
        end
        
        end
     end
    end
end
insert_pnts1(cellfun(@isempty, insert_pnts1))=[];   
insert_pnts2(cellfun(@isempty, insert_pnts2))=[];   
            

     
    