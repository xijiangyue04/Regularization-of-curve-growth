%第三步： 该步骤是对于分割规则化曲线点云进行连接补洞,resolution修补点的相邻两点距离分辨率，CP_dis不同轮廓线对应点距离阈值

function  [insert_pnts1,insert_pnts2] = CFPR_holefill(Curve_feature_segment,resolution,CP_dis)

for i=1:length(Curve_feature_segment)
    [endpnts] = Curve_endpnts(Curve_feature_segment{i});
    curve_start(i,:)=endpnts(1,:);
    curve_end(i,:)=endpnts(2,:);
end


for i=1:length(Curve_feature_segment)
    remain_start=curve_start(i+1:end,:); %当i=length(Curve_feature_segment)时，为空集
    remain_end=curve_end(i+1:end,:);
    if ~isempty(remain_start)
    st_dis1=sqrt(sum((curve_start(i,:)-remain_start).^2,2)); %起点与剩余起点的距离 最小值
    st_dis2=sqrt(sum((curve_start(i,:)-remain_end).^2,2)); %起点与剩余终点的距离 最小值
    end_dis1=sqrt(sum((curve_end(i,:)-remain_start).^2,2)); %终点与剩余起点的距离 最小值
    end_dis2=sqrt(sum((curve_end(i,:)-remain_end).^2,2)); %终点与剩余终点的距离 最小值
    dist=[st_dis1, st_dis2,end_dis1,end_dis2];
    [min_a,index]=min(dist,[],1); %矩阵dist距离每列的最小值及对应的行号
    min_index=[min_a;index;1:4]';%第一列是对应距离，第二列对应其他轮廓线标签号，第三列对应起点，终点对应可能性，即是1：起点-起点:2：起点-终点，3：终点-起点，4：终点-终点
    sort_min=sortrows(min_index,1);
    correspond_dist=sort_min(1:2,1);%本体轮廓线与其他轮廓线端点对应的两个最小距离
    
    if correspond_dist(1)<CP_dis %对应的第一个轮廓线最小距离满足条件
        Nearest_index1=sort_min(1,2); %对应的第一个轮廓线标签号
        correspond_index=sort_min(1,3);%对应的第一个轮廓线属于哪种对应
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
    
     if correspond_dist(2)<CP_dis %对应的第二个轮廓线最小距离满足条件
        Nearest_index2=sort_min(2,2); %对应的第二个轮廓线标签号
        if Nearest_index2==Nearest_index1
            insert_pnts2{i}=[];
        else
        correspond_index=sort_min(2,3);%对应的第二个轮廓线属于哪种对应
        
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
            

     
    