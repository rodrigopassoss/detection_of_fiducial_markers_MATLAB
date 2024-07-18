
im = zeros(20,20);
im(5:8,6:9) = 1;
im(12:15,9:13) = 1;

[ImSeg,Reg,ordem]=segmentador(logical(im),1)


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
    ImSeg = (f);
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
% Para a segmentação
function angle360 = convertTo360(angle)
    % Usa a função mod para garantir que o ângulo esteja no intervalo de 0 a 360 graus
    angle360 = mod(angle, 360);

    % Certifica-se de que o resultado seja positivo
    if angle360 < 0
        angle360 = angle360 + 360;
    end
end