# VI-obstacle-detector
Computer Vision based obstacle detector for Visually Impaired people

1. MATLAB: Contiene los scripts de MATLAB para pruebas y visualización de resultados. 

a. virtual_disparity.m: Script para probar el método del mapa de disparidad virtual. El script primero calcula la disparidad, a partir de la cual, teniendo los datos intrínsecos de la cámara, la transforma para estar situada en el suelo, generando la disparidad virtual, así como la v-disparity. Acto seguido, se deshace la transformación para mostrar en la perspectiva de la cámara real únicamente los obstáculos. 

b. inverted_cones.m: Primera versión del algoritmo de detección de obstáculos descrito en este documento, implementado en MATLAB. Genera una imagen binaria, en la que un píxel a 1 representa un obstáculo y a 0 representa fondo. Se pueden ajustar los parámetros 𝐻𝑇,𝐻𝑀,theta 𝑦 𝑍𝑀𝑎𝑥 según la descripción en el artículo Fast and Reliable Obstacle Detection and Segmentation for Cross-country Navigation. No incluye la segmentación de obstáculos ni las cajas delimitadoras de obstáculo. 

c. bboxes.m: Script para generar las imágenes incluyendo las cajas delimitadoras, partiendo de las imágenes de etiquetas. Es necesario cambiar en el script la localización de las carpetas en las que se encuentran almacenadas las imágenes de disparidad, las imágenes de etiquetas, y las imágenes originales así como también las carpetas donde se guardarán las imágenes de etiquetas y las originales con las cajas delimitadoras marcadas.  También se deben de fijar las variables FRAME_INICIAL y N_FRAMES para delimitar los frames numerados que se procesarán. 


2. OpenCV: Contiene el código fuente para el método descrito en este documento, así como el método de la v-disparity. Para hacerlos funcionar es necesario enlazar las librerías de OpenCV 2.4.9 imgproc y calib3d, así como indicar donde se encuentran instalados los respectivos archivos de cabecera (.hpp). En nuestro caso hemos usado el compilador g++ en un sistema Ubuntu 14.04.  

a. process_sequence.cpp: Código escrito en C++ para automatizar las pruebas del método descrito en este documento (método de los “conos invertidos”). Está especialmente pensado para procesar secuencias rectificadas del dataset KITTI. Una vez compilado, se llama al programa de la siguiente manera: process_sequence video_path_input output_path H_t H_m theta start_frame end_frame Donde: 
- video_path_input es la ruta de entrada de imágenes. Estas se pueden descargar directamente desde la página web del dataset KITTI, pero hay que asegurarse de que sean secuencias rectificadas. Siguiendo la nomenclatura del dataset KITTI debe contener las siguientes carpetas: o /image_00/data con las imágenes del canal derecho rectificadas. 
- /image_02/data con las imágenes del canal izquierdo rectificadas. 
- output_path es la ruta de salida de los resultados, la cual debe contener las siguientes subcarpetas: o bboxes donde se guardarán las imágenes con las cajas delimitadoras rodeando los obstáculos. o disparity donde se guardarán las imágenes de disparidad. 
- labels donde se guardarán las imágenes de etiquetas, rodeando los obstáculos representados con cajas delimitadoras. 
- labels_bin donde se guardarán las imágenes de etiquetas binarizadas 
- H_t, H_m y theta son los parámetros del algoritmo de detección de obstáculos. 
- start_frame y end_frame son los frames inicial y final que queremos procesar  El programa procesa el intervalo de frames indicado por comando devolviendo para cada par de imágenes de entrada, una imagen con los obstáculos rodeados por cajas delimitadoras, la imagen de etiquetas con los obstáculos también rodeados, la imagen de disparidad, y una imagen binaria con los obstáculos. Además, los parámetros de llamada se guardarán en output_path/parámetros_algoritmo.txt. 
 
b. v_disparity.cpp: Código de prueba para el método de la u-disparidad y vdisparidad. No acepta parámetros y hay que cambiar en el código la ruta a los archivos mediante las variables path, image_name_left, e image_name_right. Los parámetros se han implementado mediante directivas de preprocesador #define. Graba en la carpeta path/Stereo_ROI las imágenes de salida con las cajas delimitadoras de los obstáculos y en rojo la forma de los obstáculos. En path/Stereo_ROI/Debug guarda en diferentes carpetas los resultados de disparidad, u-disparidad, v-disparidad, y una máscara con la forma de los obstáculos
