clc
clear all
close all

%% Abre a imm e dicionário
load('dicionario_6x6.mat');
load('camera_params2.mat');
% configuracamera
% I = snapshot(cam);
I = imread('frame2.png');


%% Inicialização das Variáveis de Medição
d = 0;psi=0;theta=0;

auto_subamostragem = 0;
ti = 0; ts = 0.1;
fps = 0;
while true

tic

% I = snapshot(cam);
imshow(uint8(I));
hold on

%% Subamostragem da imagem
[Ih,Iw,Z]=size(I);
tau_c = 32; 
if auto_subamostragem==0
    ti = 0.05; % comentar se quiser deixar adaptativo
end
tau_i = tau_c + ti*max([Ih,Iw]);
passo = round(tau_i/tau_c);
I_r = I(1:passo:end,1:passo:end,:);
% sub = toc


%% Binarização da imm
% tic
Igray = convert_cinza(I_r);
Ibin = ~convert_bin2(Igray,otsu(Igray));

%% Criação da Pirâmide de Imagens
i = 2;
I_piramide(1) = struct('I',convert_cinza(I));
fator_de_escala = passo;
while true
    aux = I_piramide(i-1).I(1:2:end,1:2:end);
    [a,b]=size(aux);
    if(a*b > tau_c*tau_c)
        I_piramide(i) = struct('I',aux);
        fator_de_escala(i) = fator_de_escala(i-1)/2;
        i = i + 1;
    else
        break;
    end
end
I_piramide = I_piramide(end:-1:1); % Reordena do menor para o maior
fator_de_escala = fator_de_escala(end:-1:1);

%% Obetenção dos Contornos da imagem
% Segmentador implementado com o seguidor de bordas de Suzuki e Abe
[ImSeg,Reg,pontos]=segmentador(Ibin,4*(tau_c-tau_c*0.6));
% Filtragem e obtenção dos cantos com o algoritmo de Douglas Peucker
[cantos] = filtragem(Reg,pontos,0.3);


%% Identificação dos Marcadores
% Essa função, realiza a transformação Homográfica e identifica o padrão
[detectados,rejeitados,menor_perimetro] = obtencao_do_codigo(dicionario,cantos,I_piramide,fator_de_escala,tau_c,8,1);

%% Atualiza o parâmetro de subamostragem para o caso automatico
ti = (1-ts)*(menor_perimetro/(4*max([Ih,Iw])));

%% Corner upsampling
% usado para obter maior precisão na estimativa dos cantos
window = 11;
[detectados] = corner_upsampling(detectados,I_piramide,fator_de_escala,passo,window);

%% Estimativa da Pose Segundo o Artigo:
% Carelli, R., Soria, C. M., & Morales, B. (2005, July). Vision-based
% tracking control for mobile robots. In ICAR'05. Proceedings., 
% 12th International Conference on Advanced Robotics, 2005. (pp. 148-152). IEEE.
if length(detectados)==1
E = 53; % em mm
fx = cameraParams.FocalLength(1);
fy = cameraParams.FocalLength(2);
cx = cameraParams.PrincipalPoint(1);
hL = sqrt((detectados.cantos.x(1)-detectados.cantos.x(2))^2 + ...
          (detectados.cantos.y(1)-detectados.cantos.y(2))^2);
hR = sqrt((detectados.cantos.x(3)-detectados.cantos.x(4))^2 + ...
           (detectados.cantos.y(3)-detectados.cantos.y(4))^2);
xA = detectados.cantos.x(1); xB = detectados.cantos.x(3);
f = (fx+fy)/4;
zL = f*E/hL; zR = f*E/hR;
% xL = ((zL - f)/f)*xA; xR = ((zR - f)/f)*xB;
xL = (zL/f)*(xA-cx); xR = (zR/f)*(xB-cx);
xT = (xL+xR)/2;
zT = (zL+zR)/2;
d = sqrt(xT^2 + zT^2);
phi = atan2((xR-xL),(zR-zL))*(180/pi);
psi = atan2(xT,zT)*(180/pi);
theta = (phi+psi);


end

%% Plot do controno + id do marcador
for i=1:length(detectados)
    plot([detectados(i).cantos.x detectados(i).cantos.x(1)],[detectados(i).cantos.y detectados(i).cantos.y(1)],'g','linewidth',2);
    plot(detectados(i).cantos.x(1), detectados(i).cantos.y(1),'sr','MarkerSize',8);
    text(detectados(i).cantos.x(1)-5,detectados(i).cantos.y(1)-12,['id = ',num2str(detectados(i).id)]);
end
text(size(I,2)-150,40,['FPS = ',num2str(fps)]);
text(50,50,40,['d = ',num2str(d)]);
text(50,75,40,['psi = ',num2str(psi)]);
text(50,100,40,['theta = ',num2str(theta)]);
drawnow;
hold off
% for i=1:length(rejeitados)
%     plot([rejeitados(i).cantos.x rejeitados(i).cantos.x(1)],[rejeitados(i).cantos.y rejeitados(i).cantos.y(1)],'r','linewidth',2);
fps=toc;
fps = 1/fps;

end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                             %
%                              Funções Auxiliares                             %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
% Binarização por limiar global
function imbin = convert_bin2(im,limiar);
    imbin = false(size(im));
    imbin(im>limiar) = true;
end
% Limiar de Otsu
function limiar = otsu(imgray);
    Pr = histg(imgray);
    CumPr = cumsum(Pr);
    i = [0:255];
    mg = sum(i.*Pr);
    for k = 1:256
        mk = sum(i(1:k).*Pr(1:k));
        sigma(k) = ((mg*CumPr(k)-mk)^2)/(CumPr(k)*(1-CumPr(k)));
    end
    [~,K]=max(sigma);
    limiar = K-1;
%     #limiar_groundTruth = graythresh(imgray)*255
end
% Histograma usado no Limiar de Otsu
function [hist] = histg(im);
    [lin,col]=size(im);
    for k = 1:256
       hist(k) = numel(find(im==(k-1)));
    end
    hist = hist/(lin*col);
end

% Segmentador
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


function [cantos] = filtragem(Reg,pontos,limiar);
    cnt = 0;
    for i = 1:length(Reg)
        points = [pontos(Reg(i)-1).x' pontos(Reg(i)-1).y'];
        simplifiedPoints = DouglasPeucker(points,2);
        if size(simplifiedPoints,1)<4 continue; end
        simplifiedPoints = DouglasPeuckerPosteriori(simplifiedPoints,limiar);
        if size(simplifiedPoints,1)<4
            continue; 
        else
            cnt = cnt + 1;
            cantos(cnt) = struct('x',simplifiedPoints(:,1)','y',simplifiedPoints(:,2)');
        end
    end
    if cnt==0, cantos=[]; end

end

% Função auxliar de escalonamento
function [Xesc] = escalonar(X,top_value,botton_value);
    min_value = min(X(:))-botton_value;
    max_value = max(X(:));
    Xesc = top_value*((X-min_value)/(max_value-min_value));
end
function matriz_rotacao = matrizRotacao2D(angulo_graus)
    % Converter o ângulo de graus para radianos
    angulo_radianos = deg2rad(angulo_graus);

    % Calcular as entradas da matriz de rotação
    cos_theta = cos(angulo_radianos);
    sin_theta = sin(angulo_radianos);

    % Criar a matriz de rotação
    matriz_rotacao = [cos_theta, -sin_theta; sin_theta, cos_theta];
end

function [ok,id,angulo] = validacao_marcador(A,N,dicionario,modo)
    if modo
        tamanho_dicionario = length(dicionario);
    else
        tamanho_dicionario = 1;
    end
    distancias = zeros(4,tamanho_dicionario);
    k = 0:90:270;
    test = sum(A(1,:))+sum(A(end,:))+sum(A(2:end-1,1))+sum(A(2:end-1,end));
    
    if ((test/(4*N)) > 0.5) 
        ok = 0;
        id = [];
        angulo = [];
    else
        A = A(2:end-1,2:end-1);
        for i = 1:tamanho_dicionario
            distancias(1,i) = pdist(double([A(:)';dicionario(i).codigo(:)']),'hamming');
            A1 = imrotate(A, 90);
            distancias(2,i) = pdist(double([A1(:)';dicionario(i).codigo(:)']),'hamming');
            A2 = imrotate(A1, 90);
            distancias(3,i) = pdist(double([A2(:)';dicionario(i).codigo(:)']),'hamming');
            A3 = imrotate(A2, 90);
            distancias(4,i) = pdist(double([A3(:)';dicionario(i).codigo(:)']),'hamming');
        end
        distancias = distancias*(size(A,1)*size(A,2));
        [V,aponta_angs]=min(distancias);
        [V,idc] = min(V);
        if(V<=4)
            ok = 1;
            id = dicionario(idc).id;
            angulo = k(aponta_angs(idc));
        else
            ok = 0;
            id = [];
            angulo = [];
        end
    end
end

function [detectados,rejeitados,menor_perimetro] = obtencao_do_codigo(dicionario,cantos,I_piramide,fator_de_escala,tau_c,N,modo)

    n_detectados = 0; n_rejeitados = 0;
    if length(cantos)<1
        menor_perimetro = 0;
    else
        menor_perimetro = 1e6;
    end
    for i=1:length(cantos)
        % Transformação homográfica
        P = sum((([cantos(i).x(end) cantos(i).x(2:end)] - [cantos(i).x(1) cantos(i).x(1:end-1)]).^2 ...
           + ([cantos(i).y(end) cantos(i).y(2:end)] - [cantos(i).y(1) cantos(i).y(1:end-1)]).^2).^0.5);
        perimetro = fator_de_escala.*P;
        [~,idc] = min(abs(perimetro-4*tau_c));
        if perimetro(idc)<menor_perimetro, menor_perimetro = perimetro(idc); end
       
        Y = [fator_de_escala(idc)*cantos(i).x;
             fator_de_escala(idc)*cantos(i).y; cantos(i).y.^0];
        X = [1 1 tau_c tau_c;1 tau_c tau_c 1;1 1 1 1];
        H = Y/X;
%         i_ = 1:mdist; j_ = 1:mdist;
%         obterTransformacao
        [i_, j_] = meshgrid(1:tau_c,1:tau_c); i_ = i_(:)';j_ = j_(:)';
        ImCanonica = zeros(tau_c,tau_c);
        Y_ = round(H*[j_;i_;i_.^0]);
        for pp = 1:(tau_c*tau_c)
            if (Y_(2,pp)>0 && Y_(2,pp)<=size(I_piramide(idc).I,1) )...
                    && (Y_(1,pp)>0 && Y_(1,pp)<=size(I_piramide(idc).I,2) )
                ImCanonica(i_(pp),j_(pp)) = I_piramide(idc).I(Y_(2,pp),Y_(1,pp));
            end
        end
%         ImCanonica(i_,j_) = I_piramide(idc).I(Y_(2,:),Y_(1,:));
%         imshow(uint8(ImCanonica));
        ImCanonica = convert_bin2(ImCanonica,otsu(ImCanonica));
%         imshow(uint8(255*ImCanonica));
        
        % Obtenção do padrão
        grade = [1 round(tau_c/N):round(tau_c/N):tau_c];
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
%         imshow(A);
        % Validação
        [ok,id,angulo] = validacao_marcador(A,N,dicionario,modo);
        
        if(ok)
            n_detectados = n_detectados + 1;
            detectados(n_detectados) = struct('id',id,'angulo',angulo,'cantos',cantos(i),'pose',[],'codigo',A);
        else
            n_rejeitados = n_rejeitados + 1;
            rejeitados(n_rejeitados) = struct('id',[],'cantos',cantos(i),'codigo',A);
        end
        
        % Exiba
%         imshow(uint8(255*ImCanonica));


    end
    if(n_detectados==0) detectados = [];menor_perimetro = 0; end
    if(n_rejeitados==0) rejeitados = []; end
end

% Para a segmentação
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

function simplifiedPoints = DouglasPeuckerPosteriori(points,limiar);
    dpoints = points(2:end,1:2)-points(1:end-1,1:2);
    dists = sqrt(dpoints(:,1).^2 + dpoints(:,2).^2);
    dmax = max(dists);
    simplifiedPoints = points(1,:);
    recyclePoints = [];
    for i = 2:size(points,1)
        if size(simplifiedPoints,1)<4
            dist = min(sqrt((simplifiedPoints(:,1)-(points(i,1))).^2 + (simplifiedPoints(:,2)-(points(i,2))).^2));
            dist_norm = dist/dmax;        
            if dist_norm > limiar
                simplifiedPoints = [simplifiedPoints; points(i,:)];  
            else
                if i==2
                    recyclePoints = [recyclePoints; points(i,:)];
                else
                    dist1 = min(sqrt((simplifiedPoints(1:end-1,1)-(points(i,1))).^2 ...
                         + (simplifiedPoints(1:end-1,2)-(points(i,2))).^2));
                    dist2 = min(sqrt((simplifiedPoints(1:end-1,1)-(simplifiedPoints(end,1))).^2 ...
                          + (simplifiedPoints(1:end-1,2)-(simplifiedPoints(end,2))).^2));
                    if dist1 > dist2
                        simplifiedPoints(end,:) = points(i,:);
                    end
                end
            end
        else 
            dist1 = min(sqrt((simplifiedPoints(2:end,1)-(points(i,1))).^2 ...
                 + (simplifiedPoints(2:end,2)-(points(i,2))).^2));
            dist2 = min(sqrt((simplifiedPoints(2:end,1)-(simplifiedPoints(1,1))).^2 ...
                  + (simplifiedPoints(2:end,2)-(simplifiedPoints(1,2))).^2));
            if dist1 > dist2
                simplifiedPoints(1,:) = points(i,:);
            end
        end
    end 
    for j = 1:size(recyclePoints,1)
        dist1 = min(sqrt((simplifiedPoints(2:end,1)-(recyclePoints(j,1))).^2 ...
             + (simplifiedPoints(2:end,2)-(recyclePoints(j,2))).^2));
        dist2 = min(sqrt((simplifiedPoints(2:end,1)-(simplifiedPoints(1,1))).^2 ...
              + (simplifiedPoints(2:end,2)-(simplifiedPoints(1,2))).^2));
        if dist1 > dist2
            simplifiedPoints(1,:) = recyclePoints(j,:);
        end
    end
end

function novos_cantos = corner_upsampling(detectados,I_piramide,fator_de_escala,passo,window)

    I = find(fator_de_escala>1);
    fator_de_escala = fator_de_escala(I);
    I_piramide = I_piramide(I);
    for i = 1:length(detectados)
        cantos_x = detectados(i).cantos.x;
        cantos_y = detectados(i).cantos.y;
        for j = 1:length(I)
            for k = 1:4
                px = round(cantos_x(k)*fator_de_escala(j));
                py = round(cantos_y(k)*fator_de_escala(j));
                if (px>floor(window/2) && py>floor(window/2))&&...
                   (px<(size(I_piramide(j).I,2)-floor(window/2))&&...
                    py<(size(I_piramide(j).I,1)-floor(window/2)))
                    Im = I_piramide(j).I(py-floor(window/2):py+floor(window/2),...
                                         px-floor(window/2):px+floor(window/2));

                    corners = corner(Im, 'Harris',1);

                    dp = [(floor(window/2)+1) (floor(window/2)+1)]-corners;
                    % Atualiza
                    if length(dp)>1
                        cantos_x(k) = (px-dp(1))/fator_de_escala(j);
                        cantos_y(k) = (py-dp(2))/fator_de_escala(j);
                    end
                end
            end
        end
        detectados(i).cantos.x = cantos_x;
        detectados(i).cantos.y = cantos_y;
    end
    
    % Arrumando os cantos - Escala original e Ordem Correta
    angulos = [0:90:270];
    apontadores = [1 2 3 4;4 1 2 3;3 4 1 2;2 3 4 1];
    for i=1:length(detectados)
        pivo = find(detectados(i).angulo==angulos);
        detectados(i).cantos.x = detectados(i).cantos.x(apontadores(pivo,:))*passo;
        detectados(i).cantos.y = detectados(i).cantos.y(apontadores(pivo,:))*passo;
    end
    novos_cantos = detectados;
    
end

