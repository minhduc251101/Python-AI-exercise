% Arun's method
% for finding the relative transformation between two sets of 3D points using correspondences
% Shoudong, 2023 September 10
% https://jingnanshi.com/blog/arun_method_for_3d_reg.html
% input: two 3D point sets with correspondence p and p_trans (both 3 rows, N columns)
% output: the rotation R_est and tranlation t_est 
% (and the objective function value obj_fun)
%

function [R_est, t_est, obj_fun] = Arun_Shoudong_2023(p, p_trans)
load test_data 

%p = p
%p_trans = p_trans

% compute the centroid
p_ave = sum(p,2)/size(p,2);
p_trans_ave = sum(p_trans,2)/size(p_trans,2);

% 
q = p-p_ave;
q_trans = p_trans-p_trans_ave;

% compute H
H = zeros(3,3);

for i=1:size(p,2)
    H = H+q(:,i)*q_trans(:,i)';
end


% svd decomposition
[U,S,V] = svd(H);

% test the svd
test_svd = H-U*S*V';

% compute X
X=V*U';

% determinant of X
check_det = det(X);

% check whether det(X)=1
if abs(check_det-1)<1e-5
    R_est=X;
elseif abs(check_det+1)<1e-5
    disp 'algorithm failed'
else
    disp 'something wrong?'
end

% compute translation
t_est = p_trans_ave - R_est*p_ave;

% compute objective function
obj_fun = 0;

for i=1:size(p,2)
   error_i = p_trans(:,i) - (R_est*p(:,i)+t_est);
   obj_fun = obj_fun + error_i'*error_i;
end





