clc
close all
clear all

% M = imread('exemplo_aruco.jpg');
% M = convert_cinza(M);
% M  = convert_bin(M,9,20);
load('image_test.mat');
M = Ibin;

% M = zeros(15,15);
% M(10:14,10:14) = 1;
% M(2,2)=1;M(3,3)=1;M(4,4)=1;M(5,5)=1;


tic
[ImSeg,Reg,ordem]=segmentador(M,4*(32-6));
toc

% [y x] = find(ImSeg==Reg(1));

ImSeg(ImSeg>0) = 255;
ImSeg(ImSeg==0) = 50;
imshow(uint8(ImSeg))

% [x y]=ginput(20)
for i = 2:length(Reg)
    points = [ordem(Reg(i)-1).x' ordem(Reg(i)-1).y'];

    tic
    simplifiedPoints = DouglasPeucker(points,2);
    if size(simplifiedPoints,1)<4, continue; end
    simplifiedPoints = DouglasPeuckerPosteriori(simplifiedPoints,0.9);
    if size(simplifiedPoints,1)<4, continue; end
    toc
%     k = convhull(simplifiedPoints(:, 1), simplifiedPoints(:, 2),'simplify',true);
%     toc
%     convexPolygon = simplifiedPoints(k, :);
%     if size(convexPolygon, 1) > 4
%         % Optionally, further process the convex polygon to obtain exactly four corners
%         % This could involve simplifying it again or applying other constraints
%         % For simplicity, we'll use the first four vertices
%         convexPolygon = convexPolygon(2:5, :);
%     end
    hold on
    plot(ordem(Reg(i)-1).x,ordem(Reg(i)-1).y,'.r');
    plot(simplifiedPoints(:,1),simplifiedPoints(:,2),'-b','linewidth',3);
    plot(simplifiedPoints(:,1),simplifiedPoints(:,2),'.g','MarkerSize',25);
    plot(points(:,1),points(:,2),'--k','linewidth',2);


end

function [ImSeg,Reg,ordem]=segmentador(im,limiar)

    f = double(im);
    f = [0.*f(1,:); f]; f = [0.*f(:,1) f];
    f = [f; 0.*f(1,:)]; f = [f 0.*f(:,1)];
    [M,N] = size(f);
    LNBD = 0; 
    Reg = 2;
    directions = [0 -1;-1 -1;-1 0;-1 1; 0 1; 1 1; 1 0; 1 -1];
    angs = [180;225;270;315;0;45;90;135];
    i1=0;j1=0;i2=0;j2=0;
    i3=0;j3=0;i4=0;j4=0;
    
    for i = 1:M
        for j = 1:N
            x = []; y = [];
            if f(i,j)==1 && f(i,j-1)==0
                i2 = i; j2 = j-1; K = round(atan2d(i2-i,j2-j))-45;
                for k = 1:size(directions,1)
                    K = convertTo360(K+45);
                    i1 = i+directions(find(angs==K),1); 
                    j1 = j+directions(find(angs==K),2);
                    if (i1>0) && (j1>0) && (i1 <= M) && (j1 <= N)
                        if f(i1,j1)~=0
                            i2 = i1; j2 = j1;
                            i3 = i; j3 = j;
                            currDir = round(atan2d(i2-i3,j2-j3));
%                             display('Entrou no While')
                            while true
                                for l = 1:size(directions,1)
                                    currDir = convertTo360(currDir-45);
                                    I = find(angs==currDir);
                                    i4 = i3+directions(I,1); 
                                    j4 = j3+directions(I,2);
                                    if (i4>0) && (j4>0) && (i4<=M) && (j4<=N)
                                        if f(i4,j4)~=0
                                            if(f(i3,j3+1)==0)
                                                f(i3,j3) = -Reg;
                                                x = [x j3-1]; y = [y i3-1]; 
                                            elseif (f(i3,j3+1)~=0)&&(f(i3,j3)==1)
                                                f(i3,j3) = Reg;
                                                x = [x j3-1]; y = [y i3-1];
                                            end
                                            break;
                                        end
                                    end
                                end
                                if (i4==i) && (j4==j) && (i3 == i1) && (j3 == j1)
                                    break;
                                else
                                    i2 = i3; j2 = j3;
                                    i3 = i4; j3 = j4;
                                    currDir = round(atan2d(i2-i3,j2-j3));
                                    LNBD = f(i3,j3);
                                end
                            end
%                             display('Saiu do While')
                            ordem(Reg-1) = struct('Reg',Reg,'x',x,'y',y);
                            LNBD = 0; Reg = Reg + 1;

                            break;
                        end
                    end
                end
            end
        end
    end
    ImSeg = abs(f);
    ImSeg = ImSeg(2:end-1,2:end-1);
    ImSeg(ImSeg==1)=0;
    
    Reg = unique(ImSeg(:));
    for r = 1:length(Reg)
      n = sum(ImSeg(:)==Reg(r));
      if(n < limiar)
        ImSeg(ImSeg==Reg(r)) = 0;
      end
    end

%     Reg = sort(unique(ImSeg(:)));
%     for r = 1:length(Reg)
%       ImSeg(ImSeg==Reg(r)) = r-1;
%     end
    Reg = sort(unique(ImSeg(:)));
    Reg = Reg(2:end);      
end

function angle360 = convertTo360(angle)
    % Usa a função mod para garantir que o ângulo esteja no intervalo de 0 a 360 graus
    angle360 = mod(angle, 360);

    % Certifica-se de que o resultado seja positivo
    if angle360 < 0
        angle360 = angle360 + 360;
    end
end

%% Simplificação
function simplifiedPoints = DouglasPeucker(points, epsilon)
    % Douglas-Peucker algorithm for simplifying a polyline
    if size(points, 1) < 3
        simplifiedPoints = points;
        return;
    end

    [index, d] = perpendicularDistance(points, points(1, :), points(end, :));
    dmax = d(index);
    if dmax > epsilon
        left = DouglasPeucker(points(1:index, :), epsilon);
        right = DouglasPeucker(points(index:end, :), epsilon);
        simplifiedPoints = [left(1:end-1, :); right];
    else
        simplifiedPoints = [points(1, :); points(end, :)];
    end
end

function [index, distance] = perpendicularDistance(points, init, fim)
    % Calculate perpendicular distance from each point to the line formed by init and end
    distance = zeros(size(points, 1), 1);

    for i = 1:size(points, 1)
        distance(i) = abs((fim(2) - init(2)) * points(i, 1) - (fim(1) - init(1)) * points(i, 2) + fim(1) * init(2) - fim(2) * init(1)) / norm(fim - init);
    end

    % Find the point with the maximum distance
    [~, index] = max(distance);
%     perpendicular = points(index, :);
end

% Conversão em tons de cinza
function imgray = convert_cinza(im);
% Insira aqui o seu código
  y_coefs = [0.299 0.587 0.114];
  im_double = double(im);
  imgray = uint8(y_coefs(1)*im_double(:,:,1) + ...
                 y_coefs(2)*im_double(:,:,2) + ...
                 y_coefs(3)*im_double(:,:,3));
end
% Conversão em preto e branco
function imbin = convert_bin(im,window,c);
    im = double(im);
    [lin,col]=size(im);

    mask = ones(window,window)/(window^2);

    i = 1:lin;
    j = 1:col;

    % Calcule o tamanho da saída
    M = zeros(lin,col);
    k = linspace(-floor(window/2),floor(window/2),window);
    for i_ = 1:window
        for j_ = 1:window
            I = i + k(i_); idc_i = I>0 & I<=lin;
            J = j + k(j_); idc_j = J>0 & J<=col;
            M(i(idc_i),j(idc_j)) = M(i(idc_i),j(idc_j)) + im(I(idc_i),J(idc_j))*mask(i_,j_);
        end
    end
      
    %     filtro = fspecial('gaussian', window, 2);
    %     M2 = imfilter(im, filtro, 'replicate');
    
    imbin = true(size(im));
    imbin(im>(M-c)) = false;

end

function simplifiedPoints = DouglasPeuckerPosteriori(points,limiar);
    dpoints = points(2:end,1:2)-points(1:end-1,1:2);
    dists = sqrt(dpoints(:,1).^2 + dpoints(:,2).^2);
    dmax = max(dists);
    simplifiedPoints = points(1,:);
    for i = 2:size(points,1)
        dist = min(sqrt((simplifiedPoints(:,1)-(points(i,1))).^2 + (simplifiedPoints(:,2)-(points(i,2))).^2));
        dist_norm = dist/dmax;
        if dist_norm > limiar
            simplifiedPoints = [simplifiedPoints; points(i,:)];  
        end
        if size(simplifiedPoints,1)==4, break; end
    end 
end

