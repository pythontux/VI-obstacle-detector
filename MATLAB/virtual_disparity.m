%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Algoritmo para probar transformacion homogenea        %
%    y segmentacion de obstaculos detectados por stereo    %
%    DisparityMap--->VirtualDisparity                      %
%    30/10/2016                                            %
%    Alvaro Gregorio Gomez                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Setup things
% Comentar aquellos qure no son para el OS actual:
% Unix
%I1 = imread('/home/alvaro/Dropbox/Ingenieria/TFG/KITTI_dataset/image_00/data/0000000000.png');
%I2 = imread('/home/alvaro/Dropbox/Ingenieria/TFG/KITTI_dataset/image_02/data/0000000000.png');
% Windows
 I1 = imread('C:\Users\Alvaro\Dropbox\Ingenieria\TFG\KITTI_dataset\image_00\data\0000000020.png');
 I2 = imread('C:\Users\Alvaro\Dropbox\Ingenieria\TFG\KITTI_dataset\image_02\data\0000000020.png');
figure(1);
imshow(I1);
% figure
% imshow(I2).
Range=[-6 10];

% Calculo mapa de disparidad por SGBM
disparityMap = disparity( rgb2gray(I2),I1,  'BlockSize', 11,'DisparityRange', Range, 'UniquenessThreshold', 15, 'TextureThreshold', 0.0002);



%% Transformación homogenea
%S*D^(-1)*S^(-1)*U_v = U
%Para hacerlo mas rapido, A = S*D^(-1)*S^(-1)
%Unidades: m & px 
c_u = size(disparityMap,2)/2;
c_v = size(disparityMap,1)/2;
c_u_virtual = c_u;
c_v_virtual = c_v*2;
b = 0.0595662;
f = 950;
h = 1.65;   %1.65 altura de la camara
theta = 0;
% El siguiente parametro f, realmente denota f/rho, siendo rho el tamaño de
% pixel [cm/px]
S = [b 0 -b*c_u 0;
    0 b -b*c_v 0;
    0 0 b*f 0;
    0 0 0 1];
% Esta otra matriz esta hecha para hacer la imagen de disparidad virtual
% mas grande, evitando cortar la imagen resultante
S_v = [b 0 -b*c_u 0;
    0 b -b*c_v_virtual 0;
    0 0 b*f 0;
    0 0 0 1];
D = [1 0 0 0; 0 1 0 -h; 0 0 1 0; 0 0 0 1];%*[1 0 0 0; 0 cos(theta) sin(theta) 0;0 -sin(theta) cos(theta) 0;0 0 0 1];
A = S/D/S; % / mas eficiente que inv(Matriz)
D*S
B = S_v\D*S
%B=[1 0 0 -10;0 1 0 0;0 0 1 0;0 0 0 1]
% U_v=B*U
% U=B*U_v

%Inicializo a ceros la virtual_disparity y el vector representando 
%un punto en la disparityMap
virtual_disparity_img=zeros(2*c_v_virtual,2*c_u_virtual);
v_disparity = zeros(2*c_v_virtual, 100);
% Formato de vector homogéneo de un punto en disparityMap:
% U=[u/d v/d 1/d 1]' (4x1)
U = zeros(4,1);
U_v = zeros(4,1);
for i=1:size(disparityMap,1)	%Rows-->y
    for j=1:size(disparityMap,2)    %Cols-->x
        d = disparityMap(i, j); % (i,j)-->(cols,rows)-->(y,x)
        U =[j/d i/d 1/d 1]'; % [u/d v/d 1/d 1]'
        U_v = B*U;
        U_v = U_v./U_v(3);
        u_v_x=U_v(1);   %u_v_j
        u_v_y=U_v(2);   %u_v_i
        if((u_v_x>1) && (u_v_x<=size(virtual_disparity_img,2)) && (u_v_y>1) && (u_v_y<=size(virtual_disparity_img,1)))
            %virtual_disparity(round(u_v_y),round(u_v_x)) = disparityMap(i,j);
            %Prueba para colocar ese valor en todos los pixeles abyacentes:
            if((disparityMap(i,j)>2.7) && (u_v_y < c_v_virtual))  %Para eliminar "fondo" y pixeles por debajo de suelo
                 virtual_disparity_img(round(u_v_y),round(u_v_x)) = disparityMap(i,j);
                 v_disparity(round(u_v_y), round(10*disparityMap(i,j))) = v_disparity(i, round(10*disparityMap(i,j))) + 1;
%                 virtual_disparity_img(floor(u_v_y),floor(u_v_x)) = disparityMap(i,j);
%                 virtual_disparity_img(floor(u_v_y),round(u_v_x)) = disparityMap(i,j);
%                 virtual_disparity_img(round(u_v_y),floor(u_v_x)) = disparityMap(i,j);
%                 virtual_disparity_img(round(u_v_y),round(u_v_x)) = disparityMap(i,j);
            end
        end
    end
end
figure(2);
imshow(v_disparity, [0 25]);

% %% Construccion de v-disparity a partir de virtual disparity
% v_disparity=zeros(2*c_v_virtual,16*(Range(2)-0));
% %Construccion de v-disparity sobre virtual-disparity
% for i=1:size(virtual_disparity_img,1)	%Rows-->y
%     for j=1:size(virtual_disparity_img,2)    %Cols-->x
%         if virtual_disparity_img(i,j) > 0
%             v_disparity(i,round(16*(virtual_disparity_img(i,j)))) = v_disparity(i,round(16*(virtual_disparity_img(i,j)))) +1 ;
%         end
%     end
% end
%figure(4);
%imshow(v_disparity, [0 80]);   %Este rango estï¿½ ma, se puede superar, pero es para verlo correctamente

%% Acondicionamiento de la virtual disparity
% Quitar comentarios en lo que proceda

figure(3)
imshow(virtual_disparity_img, Range);
% hold on;
% line([0,2*c_u],[c_v_virtual,c_v_virtual])
% hold off;
virtual_mask_original = im2bw(virtual_disparity_img, 0);
virtual_mask = imfill(virtual_mask_original, 'holes');

% figure(4)
% imshow(virtual_mask);
% % title('Huecos cerrados');
% virtual_mask = imclose(virtual_mask,strel('line', 50, 0));
% virtual_mask = imopen(virtual_mask,strel('line', 45, 90));
% virtual_mask = imopen(virtual_mask,strel('line', 3, 0));
% 
% figure(5);
% imshow(virtual_mask);

% Convolucion con imagen de disparidad
% virtual_disparity_img = virtual_disparity_img .* virtual_mask;
% figure(6);
% imshow(virtual_disparity_img);

% Print results on original image space
%Deshago la transformacion para pasar la mascara a la imagen original
mask = zeros(size(disparityMap), 'uint8');
for i=1:size(disparityMap,1)	%Rows-->y
    for j=1:size(disparityMap,2)    %Cols-->x
        d = disparityMap(i, j); % (i,j)-->(cols,rows)-->(y,x)
        U =[j/d i/d 1/d 1]'; % [u/d v/d 1/d 1]'
        U_v = B*U;
        U_v = U_v./U_v(3);
        u_v_x=U_v(1);   %u_v_j
        u_v_y=U_v(2);   %u_v_i
        if((disparityMap(i,j) > 0) && (u_v_x>1) && (u_v_x<=size(virtual_mask,2)) && (u_v_y>1) && (u_v_y<=size(virtual_mask,1)))
            mask(i,j) = virtual_mask_original(round(u_v_y),round(u_v_x));
        end
    end
end

%Convolucion con imagen original para ver obstaculos y con disparidad
I1 = I1 .* mask;
figure(6)
imshow(I1);

obstacles_disparity = disparityMap .* single(mask);
figure(7)
imshow(obstacles_disparity, Range);

%% Segmentacion
obstacles_disparity = obstacles_disparity.*(255/10);
obstacles_disparity = uint8(obstacles_disparity);
c = 3;  % 3 Clusters
[L,C,U,LUT,H] = FastFCMeans(obstacles_disparity,3);
L = L(:,:,2:c); % El primer canal no son obstáculos, sino el espacio libre
%L = imfill(L(:,:,2:3),'holes'); 
L = imopen(L,strel('square', 8));


%% Etiquetado en obstáculos cercanos
% objects = bwlabel(L(:,:,c-1));
% figure(10)
% imshow(objects, [0, max(max(objects))]);
blobAnalyser = vision.BlobAnalysis('BoundingBoxOutputPort', true, ...
            'AreaOutputPort', true, 'CentroidOutputPort', true, ...
            'MinimumBlobArea', 400);
[~,centroids_near,bboxes_near] = step(blobAnalyser, im2bw(L(:,:,c-1),0));
[~,centroids_far,bboxes_far] = step(blobAnalyser, im2bw(L(:,:,c-2),0));
RGB = zeros(size(L,1),size(L,2),3);
RGB(:,:,3) = im2double(255.*L(:,:,c-1)+32.*L(:,:,c-2));
figure(9);
imshow(RGB);
for i=1:size(bboxes_near,1)
    % A tener en cuenta: La distancia se estima en el centroide, que puede 
    % ser un agujero y fallar
    distancia = b*f/disparityMap(round(centroids_near(i,2)), round(centroids_near(i,1))); 
    RGB = insertObjectAnnotation(RGB,'Rectangle', bboxes_near(i,:) ,distancia,'LineWidth',3);
end
for i=1:size(bboxes_far,1)
    distancia = b*f/disparityMap(round(centroids_far(i,2)), round(centroids_far(i,1)));
    RGB = insertObjectAnnotation(RGB,'Rectangle', bboxes_far(i,:) ,distancia,'LineWidth',3);
end
figure(10)
imshow(RGB);




