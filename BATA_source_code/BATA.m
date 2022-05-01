function t=BATA(tij_index,tij_observe,param)
% Input: tij_index: 2 by n matrix specifying the edge (i,j)
%        tij_observe: 3 by n matrix specifying tij for each edge (i,j), such that
%        tj-ti/norm(tj-ti) = tij
% Output: t: 3 by n matrix specifying the camera locations
    numofcam = max(max(tij_index));     
    numofobser = size(tij_observe,2);   
    Ri_T = repmat(eye(3),numofobser,1);
    Rj_T = repmat(eye(3),numofobser,1);
    
    index_ti_I = [(1:3*numofobser)' (1:3*numofobser)' (1:3*numofobser)'];   % position of coefficient for Ri
    index_ti_J = (tij_index(1,:)-1)*3+1;                   
    index_ti_J = [index_ti_J index_ti_J+1 index_ti_J+2];    
    index_ti_J = index_ti_J(ceil((1:3*size(index_ti_J,1))/3), :);   
    index_tj_I = [(1:3*numofobser)' (1:3*numofobser)' (1:3*numofobser)'];   % position of coefficient for Ri
    index_tj_J = (tij_index(2,:)-1)*3+1;             
    index_tj_J = [index_tj_J index_tj_J+1 index_tj_J+2];    
    index_tj_J = index_tj_J(ceil((1:3*size(index_tj_J,1))/3), :);   
    
    At0_full = sparse(index_ti_I,index_ti_J, Ri_T,3*numofobser,3*numofcam)...
       -sparse(index_tj_I,index_tj_J,Rj_T,3*numofobser,3*numofcam);  
    At0 = At0_full(:,4:end);   
        
    Aeq1 = sparse(reshape(repmat(1:numofobser,3,1),[],1),1:3*numofobser,tij_observe)*At0_full;
    Aeq1 = sum(Aeq1);           
    beq1 = numofobser;
    Aeq2 = repmat(eye(3),1,numofcam);   
    beq2 = zeros(3,1);          
    Aeq = [Aeq1;Aeq2];          
    beq = [beq1;beq2];          
    
	% Initialization with LUDRevised
    Svec = rand(1,numofobser); 
    Svec = Svec/sum(Svec)*numofobser;   
    S = reshape(repmat(Svec,3,1),[],1); 
    W = ones(3*numofobser,1);           
    ii = 1;
    while(ii<=param.numofiterinit)
        
        A = sparse(1:3*numofobser,1:3*numofobser,sqrt(W))*At0_full; 
        B = sqrt(W).*S.*tij_observe(:);
        X = [2*A'*A Aeq'; Aeq zeros(size(Aeq,1))]\[2*A'*B; beq];    
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