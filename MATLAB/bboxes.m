% Numero de frames a procesar 
FRAME_INICIAL = 0;
N_FRAMES = 100;

path ='C:\Users\Alvaro\TFG\video4\resultados1\'; % Esta carpeta contiene 3 subcarpetas: disparity, labels, y labels_bin
scale = 0.7; % Escalado previo a calcular la disparidad. Necesario para calcular correctamente la coordenada Z de los obstáculos
min_area_blob = 1; %Minimo area en m^2 para ser considerado un obstaculo
min_area_blob_pixels = 10; %Minimo area en la imagen en px para ser considerado un obstaculo

% Matrices de calibración (Ojo que la referencia de MATLAB es la traspuesta de la que da el dataset)
% 2->Izquierda
% 0->Derecha y origen de coordenadas
P0=[7.215377e+02 0.000000e+00 6.095593e+02;
    0.000000e+00 7.215377e+02 1.728540e+02; 
    0.000000e+00 0.000000e+00 1.000000e+00]';

P2=[7.215377e+02 0.000000e+00 6.095593e+02;
    0.000000e+00 7.215377e+02 1.728540e+02;
    0.000000e+00 0.000000e+00 1.000000e+00]';

f = P0(1,1);

T2=[5.956621e-02 2.900141e-04 2.577209e-03]';
b = T2(1);


R2=[9.998817e-01 1.511453e-02 -2.841595e-03;
    -1.511724e-02 9.998853e-01 -9.338510e-04;
    2.827154e-03 9.766976e-04 9.999955e-01];


%%


% Creo objetos con parámetros del sistema stereo
camera0Params = cameraParameters('IntrinsicMatrix', P0, 'WorldUnits', 'm');
camera2Params = cameraParameters('IntrinsicMatrix', P2, 'WorldUnits', 'm');
stereoParams = stereoParameters(camera0Params,camera2Params,R2,T2);

% Bucle para procesar secuencia
for frame = FRAME_INICIAL:N_FRAMES
    
    % Calculo de disparidad solo necesario en caso de medir coordenadas
    
    % Compongo el nombre de la foto real
    frame_name={path,'..\image_02\data\',num2str(frame,'%010i'),'.png'};     
    % Compongo nombre del archivo de disparidad
    disparity_map_name={path,'disparity\',num2str(frame,'%010i'),'.png'};   
    % Compongo nombre del archivo de etiquetas
    labels_name={path,'labels\',num2str(frame,'%010i'),'.png'}; 
    % Compongo nombre del archivo de salida BBoxes
    output_bboxes={path,'bboxes\',num2str(frame,'%010i'),'.png'}; 
    % Compongo nombre del archivo de salida BBoxes (referido a otra imagen)
    output_bboxes_labels={path,'bboxes_labels\',num2str(frame,'%010i'),'.png'}; 
    
    % Abro frame 
    frame_img = imread(strjoin(frame_name,''));    
    
    % Disparidad en formato uint -> double:
    disp = imread(strjoin(disparity_map_name,''));
    disp = 255/16*im2double(disp); %im2double lo escala del rango 0-255 a 0-1
    disp_dimensions=size(disp);
    u0=disp_dimensions(2)/2; % Centro de la imagen de disparidad (u)
    v0=disp_dimensions(1)/2; % Centro de la imagen de disparidad (v)
    
    % Etiquetas en formato uint8:
    labels = imread(strjoin(labels_name,''));
    
    % Cálculo de las coordenadas (LENTO!!!)
    
    % Array de Coordenadas en formato double
    XYZ=zeros(disp_dimensions(1),disp_dimensions(2),3);
    
    % Bucle para calcular las coordenadas de los puntos
    for v=1:disp_dimensions(1)
        for u=1:disp_dimensions(2)            
                Z = (b*f*scale/disp(v,u));
            if (Z < 20)
                X = (b*(u-u0)/disp(v,u));
                Y = (b*(v0-v)/disp(v,u));  
                XYZ(v,u,1)=X;
                XYZ(v,u,2)=Y;
                XYZ(v,u,3)=Z;
            else
                Z = -1; % Z negativo indica objeto demasiado lejano
            end

        end
    end              

    
    blobMeasurements=regionprops(labels,'Area','BoundingBox','Centroid');
    for label_id=1:size(blobMeasurements)
        % Area en el mundo real = (B/d)^2*Area en la imagen (px)
        if(blobMeasurements(label_id).Area>0)
            Area_mundo=(b/disp(round(blobMeasurements(label_id).Centroid(2)),round(blobMeasurements(label_id).Centroid(1))))^2*blobMeasurements(label_id).Area;
            if ((Area_mundo > min_area_blob)&&(blobMeasurements(label_id).Area>min_area_blob_pixels))
                % Etiquetar bboxes con labels
                % labels = insertObjectAnnotation(labels,'Rectangle', blobMeasurements(label_id).BoundingBox , label_id,'LineWidth',3); 
                % Etiquetar bboxes con coordenada X
                coords_str = num2str(XYZ(round(blobMeasurements(label_id).Centroid(2)),round(blobMeasurements(label_id).Centroid(1)),:));
                %labels = insertObjectAnnotation(labels,'Rectangle', blobMeasurements(label_id).BoundingBox , coords_str,'LineWidth',3);
                labels = insertObjectAnnotation(labels,'Rectangle', blobMeasurements(label_id).BoundingBox , '','LineWidth',3);
                % Escalo bbox para que se ajuste a la imagen real (mayor
                % tamaño)
                frame_img = insertObjectAnnotation(frame_img,'Rectangle', blobMeasurements(label_id).BoundingBox/scale , coords_str,'LineWidth',3);
            end
        end
    end
    figure(1);
    imshow(labels);
    imwrite(frame_img, strjoin(output_bboxes,''));
    imwrite(labels, strjoin(output_bboxes_labels,''));
end