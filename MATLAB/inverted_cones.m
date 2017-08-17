%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Codigo para probar el algoritmo de los conos          %
%    invertidos como deteccion de obstaculos /eliminacion  %
%    de suelo                                              %
%    26/12/2016                                            %
%    Alvaro Gregorio Gomez                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Lectura de im·genes y par·metros de la camara
% Comentar los que no son para el OS actual:
% Unix
I1 = imread('/home/alvaro/Dropbox/Ingenieria/TFG/KITTI_dataset/image_00/data/0000000000.png');
I2 = imread('/home/alvaro/Dropbox/Ingenieria/TFG/KITTI_dataset/image_02/data/0000000000.png');
% Windows
%  I1 = imread('C:\Users\Alvaro\Dropbox\Ingenieria\TFG\KITTI_dataset\image_00\data\0000000020.png');
%  I2 = imread('C:\Users\Alvaro\Dropbox\Ingenieria\TFG\KITTI_dataset\image_02\data\0000000020.png');
figure(1);
imshow(I1);
figure(2);
imshow(I2)
Range=[-6 10];

% Calibration matrixes (Ojo que la referencia de MATLAB es la traspuesta de la que da el dataset)
P0=[7.215377e+02 0.000000e+00 6.095593e+02;
    0.000000e+00 7.215377e+02 1.728540e+02; 
    0.000000e+00 0.000000e+00 1.000000e+00]';

P2=[7.215377e+02 0.000000e+00 6.095593e+02;
    0.000000e+00 7.215377e+02 1.728540e+02;
    0.000000e+00 0.000000e+00 1.000000e+00]';

f = P0(1,1);

T2=[5.956621e-02 2.900141e-04 2.577209e-03]';
b = T2(1);
T2=[5.956621e-02 0 0]';   % Usar este!!

R2=[9.998817e-01 1.511453e-02 -2.841595e-03;
    -1.511724e-02 9.998853e-01 -9.338510e-04;
    2.827154e-03 9.766976e-04 9.999955e-01];
R2=eye(3);

% Parametros del algoritmo:
H_m = 0.4; 
H_t = 0.25;
phi = 89 * pi/180;  %Angulo con la vertical
theta = pi/2-phi;   %√?ngulo con la horizontal

% Calculo los parametros del montaje estereo
camera0Params = cameraParameters('IntrinsicMatrix', P0, 'WorldUnits', 'm');
camera2Params = cameraParameters('IntrinsicMatrix', P2, 'WorldUnits', 'm');
stereoParams = stereoParameters(camera0Params,camera2Params,R2,T2);
[J1, J2] = rectifyStereoImages(I1, rgb2gray(I2), stereoParams);
figure(3);
imshow((J1))
figure(4);
imshow((J2))

% Calculo el mapa de disparidad por SGBM
disparityMap = disparity( J2, J1,  'BlockSize', 11,'DisparityRange', Range, 'UniquenessThreshold', 15, 'TextureThreshold', 0.0002);
figure(5);
imshow(disparityMap, Range);

% Reconstruir posiciones xyz de los puntos
% MATLAB obliga a rectificar antes de reconstruir
xyzPoints = reconstructScene(disparityMap,stereoParams);
xyzPoints = imcrop(xyzPoints,[6 0 (1230-12) size(xyzPoints,1)]); %Le quito 6 px por cada lado (experimentalmente son el numero que sale mal)
% Los ejes Z e X estan cambiados de signo, ya que al ser la camara colocada
% a la derecha la que hace de referencia el eje Y va hacia la izquierda, y
% por tanto el Z hacia atras

% Ahora cambio los ejes Z y X para que concuerden con la referencia que
% tiene sentido (derecha, arriba y delante)
xyzPoints(:,:,3) = -1.*xyzPoints(:,:,3);
xyzPoints(:,:,1) = -1.*xyzPoints(:,:,1);

figure(6);
imshow(xyzPoints(:,:,3),[-30,20]);
title('Eje Z')
figure(7);
imshow(xyzPoints(:,:,1),[-30,20]);
title('Eje X')
figure(8);
imshow(xyzPoints(:,:,2),[-30,20]);
title('Eje Y')
xyzPoints(218, 816,:)

%Para acelerar el algoritmo reduzco el tamano de las imagenes de
%profundidad y disparidad
scale = 0.7;
xyzPoints = imresize(xyzPoints,scale);
disparityMap = imresize(xyzPoints,scale);
% Creo la mascara de obstaculos
obstaculos = zeros(size(xyzPoints, 1), size(xyzPoints, 2), 'single');
%obstaculos = cell(size(xyzPoints, 1),1);


%% Algoritmo del cono invertido para deteccion de obstaculos v2 (reduccion del tiempo)
for i=size(xyzPoints,1):-1:1	%Rows-->y
    for j=size(xyzPoints,2):-1:1    %Cols-->x
        z_p = xyzPoints(i,j,3); % profundidad
        if (z_p > 0)  % Para no calcularlo en puntos con errores de disparidad
            % Parametros para definir poligono de busqueda
            x_p = xyzPoints(i,j,1);
            y_p = xyzPoints(i,j,2);
            P = [j i]';  % Vertice en la imagen [u v]
            Pr = [x_p y_p z_p]'; % Vertice en mundo real
            h_t = H_t * f * scale / z_p;  % Limite inferior trapecio
            h_m = H_m * f * scale / z_p;  % Limite supeior trapecio
            m = tan(theta);                    
            % Bucle para recorrer poligono de busqueda
            for v=max(round(P(2)-h_m), 1):min(round(P(2)-h_t), size(xyzPoints,1))%Rows-->y                
            u_min = m*(v-P(2))+P(1);
            u_max = -m*(v-P(2))+P(1);
            for u=max(round(u_min), 1):min(round(u_max), size(xyzPoints,2))     %Cols-->x                   
                    x = xyzPoints(v, u, 1);
                    y = xyzPoints(v, u, 2);
                    z = xyzPoints(v, u, 3);
                    Pr_2 = [x y z]';    % Punto actual del scan en mundo real                   
                    % Ahora compruebo las dos condiciones para que sea un
                    % obstaculo (ver paper de las referencias)
                    cond1 = (H_t < abs(Pr_2(2)-Pr(2))) && (abs(Pr_2(2)-Pr(2)) < H_m);
                    cond2 = (Pr_2(2)-Pr(2))/(norm(Pr_2-Pr)) > sin(phi);
                    % Si ambas conduiciones son ciertas, entonces Pr_2 es
                    % un obstaculo
                    if cond1 && cond2
                        obstaculos(v,u) = 1;
                    end
                end
            end            
        end
    end
    figure(10);
    imshow(obstaculos);
end


% %% Algoritmo del cono invertido para deteccion de obst√°culos v1 (Realmente no hace falta calcular Pr_3)
% for i=size(xyzPoints,1):-1:1	%Rows-->y
%     i
%     for j=size(xyzPoints,2):-2:1    %Cols-->x
%         if (i < size(xyzPoints,1)/2)
%         end
%         z_p = xyzPoints(i,j,3); % profundidad
%         if (z_p > 0)  % Para no calcularlo en puntos con errores de disparidad
%             % Parametros para definir poligono de b√∫squeda
%             x_p = xyzPoints(i,j,1);
%             y_p = xyzPoints(i,j,2);
%             P = [j i]';  % Vertice en la imagen [u v]
%             Pr = [x_p y_p z_p]'; % V√©rtice en mundo real
%             h_t = H_t * f / z_p;  % Limite inferior trapecio
%             h_m = H_m * f / z_p;  % Limite supeior trapecio
%             m = tan(theta);                    
%             % Bucle para recorrer poligono de b√∫squeda
%             for v=max(round(P(2)-h_m), 1):min(round(P(2)-h_t), size(xyzPoints,1))	%Rows-->y
%             u_min = m*(v-P(2))+P(1);
%             u_max = -m*(v-P(2))+P(1);
%                 for u=max(round(u_min), 1):min(round(u_max), size(xyzPoints,2))     %Cols-->x
%                     if(obstaculos(v,u) ~= 1)
%                         x = xyzPoints(v, u, 1);
%                         y = xyzPoints(v, u, 2);
%                         z = xyzPoints(v, u, 3);
%                         Pr_2 = [x y z]';    % Punto actual del scan en mundo real
%                         Pr_3 = Pr+[0 H_m 0]'; % Punto superior central del trapecio                            
%                         % Ahora compruebo las dos condiciones para que sea un
%                         % obst√°culo (ver paper)
%                         cond1 = (H_t < abs(Pr_2(3)-Pr(3))) && (abs(Pr_2(3)-Pr(3)) < H_m);
%                         cond2 = ((Pr_2-Pr)'*(Pr_3-Pr))/(norm(Pr_2-Pr)*norm(Pr_3-Pr)) > cos(theta);
%                         % Si ambas conduiciones son ciertas, entonces Pr_2 es
%                         % un obstaculo
%                         if cond1 && cond2
%                             obstaculos(v,u) = 1;
%                         end
%                     end
%                 end
%             end
%         end
%     end
%     figure(10);
%     imshow(obstaculos);
% end





