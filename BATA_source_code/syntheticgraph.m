function [tij_index,tij_observe,t_gt]=syntheticgraph(numofcam,ratio_obser,ratio_outlier,sigma)
t_gt = (mvnrnd(zeros(3,1),eye(3),numofcam))';
tij_observe = zeros(3,numofcam*(numofcam-1));
tij_index = zeros(2,numofcam*(numofcam-1));
kk = 1;
for ii = 1:numofcam
    for jj = ii+1:numofcam
        if rand(1)>ratio_obser
            continue;
        else            
            tij_gt = (t_gt(:,ii)-t_gt(:,jj))/norm(t_gt(:,ii)-t_gt(:,jj));                        
            tij_index(:,kk) = [ii jj];
            if rand(1)<ratio_outlier
                tij_observe(:,kk) = rand(3,1)-0.5;
                tij_observe(:,kk) = tij_observe(:,kk)/norm(tij_observe(:,kk));                
            else
                tmp = rand(3,1)-0.5;
                tmp = tmp - (tmp'*tij_gt)*tij_gt;
                tmp = tmp/norm(tmp);
                rotaxis = cross(tmp,tij_gt);
                rotMat = vrrotvec2mat([rotaxis' sigma*normrnd(0,1)/180*pi]);
                tij_observe(:,kk) = rotMat*tij_gt;               
            end                
                kk = kk + 1;
        end        
    end
end
tij_observe = tij_observe(:,1:kk-1);
tij_index = tij_index(:,1:kk-1);
end