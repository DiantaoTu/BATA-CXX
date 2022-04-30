clear;

% create synthetic view graph
numofcam = 6; 
ratio_obser = 0.5;
ratio_outlier = 0.2;
sigma = 5;
[tij_index,tij_observe,t_gt]=syntheticgraph(numofcam,ratio_obser,ratio_outlier,sigma);

t_gt=t_gt';
% save tij_index.txt -ascii tij_index
% save tij_observe.txt -ascii tij_observe
% save t_gt.txt -ascii t_gt

tij_index = load('tij_index.txt');
tij_observe = load('tij_observe.txt');

% Translation Averaging
param.delta = 10^-6;
param.numofiterinit = 10;
param.numofouteriter = 10;
param.numofinneriter = 10;
param.robustthre = 10^-1;
t=BATA(tij_index,tij_observe,param);

t=t';
save t_BATA.txt -ascii t
                                                         

    
