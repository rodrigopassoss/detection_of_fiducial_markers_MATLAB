% A = []; b = [];
% for j = 1:4
%     A = [A; X(1,j) X(2,j) 1 0 0 0 -Y(1,j)*X(1,j) -Y(1,j)*X(2,j)];
%     A = [A; 0 0 0 X(1,j) X(2,j) 1 -Y(2,j)*X(1,j) -Y(2,j)*X(2,j)];
%     b = [b;Y(1,j);Y(2,j)];
% end
% h = [inv(A)*b;1];
% H = reshape(h,3,3);
tform = fitgeotrans( Y',X', 'projective');
% H = tform.T;