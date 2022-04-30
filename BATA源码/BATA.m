function t=BATA(tij_index,tij_observe,param)
% Input: tij_index: 2 by n matrix specifying the edge (i,j)
%        tij_observe: 3 by n matrix specifying tij for each edge (i,j), such that
%        tj-ti/norm(tj-ti) = tij
% Output: t: 3 by n matrix specifying the camera locations
    numofcam = max(max(tij_index));     % 相机的数量
    numofobser = size(tij_observe,2);   % 相对位姿的数量    
    Ri_T = repmat(eye(3),numofobser,1); % 单位阵按竖向拼接，共有numofboser个
    Rj_T = repmat(eye(3),numofobser,1); % 单位阵按竖向拼接，共有numofboser个
    
    index_ti_I = [(1:3*numofobser)' (1:3*numofobser)' (1:3*numofobser)'];   % position of coefficient for Ri
    index_ti_J = (tij_index(1,:)-1)*3+1;    % tij_index 的第一行所有数字减去1乘以3再加上1                 
    index_ti_J = [index_ti_J index_ti_J+1 index_ti_J+2];    % index_ti_J 分别+0 +1 +2然后拼成一行
    index_ti_J = index_ti_J(ceil((1:3*size(index_ti_J,1))/3), :);   % 三个index_ti_J竖向堆叠 
    index_tj_I = [(1:3*numofobser)' (1:3*numofobser)' (1:3*numofobser)'];   % position of coefficient for Ri
    index_tj_J = (tij_index(2,:)-1)*3+1;    % tij_index 的第二行所有数字减去1乘以3再加上1             
    index_tj_J = [index_tj_J index_tj_J+1 index_tj_J+2];    % index_tj_J 分别+0 +1 +2然后拼成一行
    index_tj_J = index_tj_J(ceil((1:3*size(index_tj_J,1))/3), :);   % 三个index_tj_J竖向堆叠
    
    At0_full = sparse(index_ti_I,index_ti_J, Ri_T,3*numofobser,3*numofcam)...
       -sparse(index_tj_I,index_tj_J,Rj_T,3*numofobser,3*numofcam);  
    At0 = At0_full(:,4:end);   % 跳过前三列
        
    Aeq1 = sparse(reshape(repmat(1:numofobser,3,1),[],1),1:3*numofobser,tij_observe)*At0_full;
    Aeq1 = sum(Aeq1);           % 矩阵的每一列加起来得到一个结果，从m行n列变成1行n列
    beq1 = numofobser;
    Aeq2 = repmat(eye(3),1,numofcam);   % 生成1行n列的矩阵，矩阵里每个元素是一个单位阵
    beq2 = zeros(3,1);          % 三行一列的全零矩阵
    Aeq = [Aeq1;Aeq2];          % 两个矩阵竖向拼接，结果仍然是稀疏矩阵，因为Aeq1是稀疏的
    beq = [beq1;beq2];          % 两个矩阵竖向拼接
    
	% Initialization with LUDRevised
    Svec = rand(1,numofobser);  % 1行n列的随机矩阵，在0-1之间
    Svec = Svec/sum(Svec)*numofobser;   % 所有数字乘以一个比例，比例值为 numofobser/sum(Svec)
    S = reshape(repmat(Svec,3,1),[],1); % Svec先复制三次并竖直拼接起来，得到3行n列矩阵，然后变成一列
    W = ones(3*numofobser,1);           % n行1列的全1矩阵
    ii = 1;
    while(ii<=param.numofiterinit)
        
        A = sparse(1:3*numofobser,1:3*numofobser,sqrt(W))*At0_full; % 以sqrt(W)的值为对角线的对角阵和 At0_full 相乘
        B = sqrt(W).*S.*tij_observe(:);
        X = [2*A'*A Aeq'; Aeq zeros(size(Aeq,1))]\[2*A'*B; beq];    % 求解方程ax=b，其中a就是前面的一串，b是后面的一串
        t = X(1:3*numofcam);
        
        Aij = reshape(At0_full*t,3,numofobser);
        tij_T = reshape(tij_observe,3,numofobser);
        Svec = sum(Aij.*tij_T)./sum(tij_T.*tij_T);
        tmp3 = repmat(Svec,[3,1]);
        S = tmp3(:);
        
        tmp = reshape(At0_full*t-S.*tij_observe(:),3,[]);
        Wvec = (sum(tmp.*tmp) + param.delta).^(-0.5);
        W = reshape(repmat(Wvec,[3,1]),[],1);
          
        ii = ii + 1;
    end
    t = reshape(t,3,[]) - repmat(t(1:3),1,numofcam);
    save t_LUD.txt -ascii t
    % BATA
    % Iteratively Reweighted Least Squares
    t = reshape(t(:,2:end),[],1);    
    ii = 1;    
    while(ii<param.numofouteriter)
        A = sparse(1:length(S),1:length(S),1./S,length(S),length(S))*At0; 
        B = tij_observe(:);                  
        tmp = sqrt(sum(reshape((A*t-B).^2,3,[])));
        Wvec = zeros(size(tmp));
        Wvec(tmp <= param.robustthre) = 1;
        Wvec(tmp > param.robustthre) = param.robustthre./tmp(tmp > param.robustthre);               
        W = reshape(repmat(Wvec,3,1),[],1);
        
        jj =1;  
        while(jj<=param.numofinneriter)
            Aij = reshape(At0*t,3,numofobser);
            tij_T_weighted = reshape(tij_observe,3,numofobser);
            Svec = sum(Aij.*Aij)./sum(Aij.*tij_T_weighted);       
            Svec(Svec<0) = Inf;        
            tmp3 = repmat(Svec,[3,1]);
            S = tmp3(:);      

            A = sparse(1:length(S),1:length(S),sqrt(W)./S,length(S),length(S))*At0; 
            B = sqrt(W).*tij_observe(:);                                
            t = (A'*A)\(A'*B);                
            jj = jj +1;
        end        
        ii = ii + 1;
    end    
    t = [zeros(3,1) reshape(t,3,[])];                