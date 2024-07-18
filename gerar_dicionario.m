clc
clear all
close all
N = 8;
cnt = 1;
for i = 15:26
    ImCanonica = imread(['C:\Users\rodri\Documents\Visão Computacional\projeto_final\marcadores\6x6Marker_',num2str(i-1),'.png'])
    ImCanonica = logical(ImCanonica);
    imshow(ImCanonica);
    A = zeros(N,N);
    grade = [1 round(200/N):round(200/N):200];
%         debug_grade
    c = 1;
    A = zeros(N,N);
    for j = 1:N
        for k = 1:N
           ImCanonica_mj = sum(ImCanonica((grade(j)+c):(grade(j+1)-c),:));
           A(j,k) = sum(ImCanonica_mj((grade(k)+c):(grade(k+1)-c)))/...
                    (numel((grade(j)+c):(grade(j+1)-c))*numel((grade(k)+c):(grade(k+1)-c)));
        end
    end
    A = A>0.6;
    A = A(2:end-1,2:end-1);
    dicionario(cnt) = struct('id',i-1,'codigo',A);
    cnt = cnt + 1;
end

save dicionario_6x6.mat dicionario