% 第一步： 实现的曲线点云轮廓区域增长算法
%该命令实现的是轮廓点的区域增长算法，在增长的过程中，利用最小二乘进行空间曲线拟合，并依据点到曲线距离进行增长的判断
%输入：轮廓点数据：input_pnts(nx3)，每次区域增长的数量number_neighbor（大概是8左右）,距离分辨率，可以用range_resolut命令得到
%TM是多项式空间曲线的项数，TM=2   则空间曲线方程为X=a(1)T^2+b(1)T+c(1);Y=a(2)T^2+b(2)T+c(2),...
%输出：得到增长后不同类点云segment（元包形式），pcData 所有数据为1显示都增长完了
function [pcData,segment] = Curve_region_growing_LS(input_pnts,number_neighbor,range_resolution,TM)



%赋值颜色信息，方便后面加入图像信息进行改进
color=zeros(size(input_pnts,1),3);
pcData=[input_pnts color];

% [pose,pose_indice]=getpointsXYZ(input_pnts,1);%pose为鼠标选取点的坐标，pose_indice为选取点的索引
 %手动选择一个初始种子点，进行区域生长分割
neighbors = transpose(knnsearch(input_pnts, input_pnts, 'k', number_neighbor+1));%neighbor为一个索引矩阵，第一行代表第几个点，后8行代表K近邻的点。记录每个点及其周围的8个点
seeds=[];%种子点序列 ,第一列元素代表种子点的索引
cluster=[];%聚类后的点云簇
pcData(:,7)=0;%pcData的第7列为是否生长过的标志位，0代表未生长过，1代表生长过
current_neighbors8=[];%该种子点的近邻点，第一列为索引，后三列为x,y,z
 
 j=1;
while any(pcData(:,7) == 0)
    [row,~]=find(pcData(:,7)==0);
    seeds(1,:)=[row(1,:), pcData(row(1,:),1:3)];
    cluster=seeds(1,2:end);%默认该种子点是属于某一类的

while(size(seeds)>0) %种子点为空时停止循环
    current_seed=seeds(1,:);%将种子点栈的第一个种子点给current_seed,current_seed带索引
    pcData(seeds(1,1),7)=1;%将种子点标记为已经生长过的点
    seeds(1,:)=[];%第一个种子点置为空
    
    %根据current_seed在neighbors中找到8个邻域点
    current_neighbors8_indice=neighbors(2:end,current_seed(1,1));%当前种子点的8个邻域点的索引数组，一个8x1的向量
   
    %检查每一个邻域点 共8个
   for seed_k=1:number_neighbor%seed_k是1~8的数
       current_neighbor=pcData(current_neighbors8_indice(seed_k),: );%current_neighbor是一个1x7的向量,存放8个近邻点中每一个近邻点的信息:x,y,z,r,g,b,flag
    
       %判断当前邻域点是否生长过
       %如果当前邻域点已经生长过就换下一个邻域点
       if current_neighbor(1,7)==1
           continue;
       end
          
       %计算选取的领域点到种子点切平面的距离
       %current_seed_plane_dis=abs([input_pnts(current_neighbors8_indice(seed_k),1]*pnts_parameter(pose_indice,:)');
       %先计算距离是否满足阈值
        %current_dis=sqrt(sum((current_neighbor(:,1:3)-current_seed(:,2:4)).^2,2)); %计算选取的领域点和当前种子点的距离
         if size(cluster,1)>2
             %当聚类点数大于2时，先利用空间直线拟合
             tempor_cluster=[cluster;current_neighbor(:,1:3)];
             [line_vector1,mean_pnt1] = space_line_TLS(tempor_cluster);
             [PL_dis1] = PL_distance_TLS(tempor_cluster, mean_pnt1, line_vector1);
             mean_PLdis=mean(PL_dis1);  %空间直线第一个判断条件
             [line_vector2,mean_pnt2] = space_line_TLS(cluster);
             [PL_dis2] = PL_distance_TLS(current_neighbor(:,1:3), mean_pnt2, line_vector2); %空间直线第二个判断条件

             %再利用空间曲线进行拟合
              [parameter1,~] = space_curve_LS(tempor_cluster,TM);  %
              [PC_dis1] = PC_distance_LS(tempor_cluster,parameter1); %[PC_dis] = PC_distance_LS(input_pnts,parameter)
              mean_PCdis=mean(PC_dis1); %空间曲线第一个判断条件
              
              [parameter2,~] = space_curve_LS(cluster,TM);
              [PC_dis2] = PC_distance_LS(current_neighbor(:,1:3),parameter2); %空间曲线第二个判断条件
 
         else
            mean_PLdis=0;
            PL_dis2=0;
            mean_PCdis=0;
            PC_dis2=0;
         end
       

       if (mean_PLdis<=range_resolution & PL_dis2<=range_resolution) | (mean_PCdis<=range_resolution & PC_dis2<=range_resolution) %邻域与当前种子点之间的夹角小于阈值，领域与初始种子点之间夹角小于阈值
           cluster=[cluster;current_neighbor(1,1:3)];%将这个点加入到cluster中
           pcData(current_neighbors8_indice(seed_k),7 )=1;%将这个近邻点的第7位置为1,代表已经进行生长过了
           seeds=[seeds;current_neighbors8_indice(seed_k) pcData(current_neighbors8_indice(seed_k),1:3)];

       end
   end   
end
segment{j}=cluster;
j=j+1;
% if j==3346
%     break
% end
end