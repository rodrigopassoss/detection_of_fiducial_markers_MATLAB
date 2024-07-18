clc
clear all
close all

% Y = [7, 9; 15, 20; 20, 7; 23, 22]'; % Projetado
X = [10, 10; 10, 20; 20, 10; 20, 20]'; % Originals
Y = [190   229   232   195; 177   179   155   155; 1 1 1 1];
dY = abs(Y(1:2,2:end)-Y(1:2,1:end-1));
mdist = max(dY(:));

X = [0 mdist mdist 0;0 0 mdist mdist; 1 1 1 1]

% H = randn(2,2);
% Y = H*X;

% Seja H, a matriz de transformação tal que:
%    X = H*Y
H1 = Y*X'/(X*X');
% H2 = fitgeotrans(Y', X', 'projective');


X2 = floor(inv(H1)*Y);
% X3 = H2*Y;

figure
hold on
plot(Y(1,:),Y(2,:),'.r','MarkerSize',15);
plot(X(1,:),X(2,:),'.g','MarkerSize',15);
plot(X2(1,:),X2(2,:),'*b','MarkerSize',15);
% plot(X3(1,:),X3(2,:),'*k','MarkerSize',15);



