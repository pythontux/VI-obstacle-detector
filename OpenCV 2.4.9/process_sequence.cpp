/* 
 *
 *  Created on: 1 de Junio de 2017
 *      Author: Álvaro Gregorio Gómez
 *  Este programa procesa una secuencia desde FRAME_INICIAL hasta N_FRAMES devolviendo el mapa de disparidad y la imágen de etiquetas correspondientes
 *  En caso de llamar al programa sin parámetros, se fijan parámetros predeterminados para pruebas
 *  Al llamarlo se proporciona una ayuda con los parámetros que se deben proporcionar:
 *  Usage: process_sequence video_path_input output_path_input H_t H_m phi start_frame end_frame
 *  H_t, H_m y phi son los parámetros del algoritmo de segmentación de obstáculos (phi es el ángulo con el plano horizontal)
 *  Se debe definir la máxima distancia de scan mediante Z_MAX
 */

# define PI           3.14159265358979323846  /* pi */
# define FRAME_INICIAL 0
# define N_FRAMES 100
# define Z_MAX 20
//OpenCV libraries
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/contrib/contrib.hpp"

//C++ libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <iostream>
#include <fstream> // Para escribir parametros en archivo
#include <math.h>

//Namespaces
using namespace cv;
using namespace std;

//Variable global roots & n_roots
// Guardo en roots los componentes representativos de cada conjunto (los numeros de cada obstaculo)
int n_roots = 0;
int roots[250]; //Espacio para 250 obstaculos

class UF    {
    int *id, cnt, *sz;
public:
    // Constructor: Crea una estructura de datos Union-Find, con N nodos aislados
    UF(int N)   {
        cnt = N;
	id = new int[N];
	sz = new int[N];
        for(int i=0; i<N; i++)	{
            id[i] = i;	//NOTA: Reservo el valor cero para no obstaculo
	    sz[i] = 1;
	}
    }
    ~UF()	{
	delete [] id;
	delete [] sz;
    }
	// Devuelve el valor el padre absoluto para el componente/etiqueta al que pertenece p
    int find(int p)	{
        int root = p;
        while (root != id[root])
            root = id[root];
        //Aprovecha para comprimir el componente/etiqueta ( reduce iterativamente los niveles al nivel 1)
        while (p != root) {
            int newp = id[p];
            id[p] = root;
            p = newp;
        }
        return root;
    }
    // Une los componentes/etiquetas a los que pertenecen x e y
    void merge(int x, int y)	{
        int i = find(x);
        int j = find(y);
        if (i == j) return;

		// Cuelgo el arbol pequeño del grande para minimizar el numero de niveles
        if   (sz[i] < sz[j])	{
		id[i] = j;
		sz[j] += sz[i];
	} else	{
		id[j] = i;
		sz[i] += sz[j];
	}
        cnt--;
    }
	// Pertenecen x e y al mismo componente/etiqueta?
    bool connected(int x, int y)    {
        return find(x) == find(y);
    }
	// Devuelve el numero de componentes/etiquetas
    int count() {
        return cnt;
    }
};

int Coords(Mat &CoordImage,  Mat_<Point3f> &_CoordImage, Mat &disp_32F, float b, float f, float cd_u, float cd_v, double scale){
	//Función que rellena la imágen de tres canales CoordImage y el mapa de puntos _CoordImage
	float z = -1;
	float z_max = Z_MAX;
    for( int i = 0; i < CoordImage.rows; ++i)
       for( int j = 0; j < CoordImage.cols; ++j )	//Si hay defecto en el calculo de la disparidad, las coordenadas quedan a cero
          {
    	   if (disp_32F.at<float>(i,j)>0){
    		   z = (b*f*scale/(disp_32F.at<float>(i,j)));
    		   if (z<z_max){
			   _CoordImage(i,j).z = (b*f*scale/(disp_32F.at<float>(i,j)));
			   _CoordImage(i,j).x = (b*(j-cd_u)/(disp_32F.at<float>(i,j)));
			   _CoordImage(i,j).y = (b*(cd_v-i)/(disp_32F.at<float>(i,j)));
    		   }
    	   }
       }
    CoordImage = _CoordImage;
    return 0;
}

UF InvertedCones(Mat_<Point3f> &_CoordImage, Mat &labels_img, float phi, float H_t, float H_m, float f, float scale){
	/************************************************************************\
	|Inverted cones algorithm			                                     |
	\************************************************************************/
	//LUT para remapear imagen segmentada. Es muy importante que no se cambie el elemento 0 (background)
	Mat lut;
	lut=Mat::zeros(1,256,CV_8U);
	
	//Instancia de estructura Union-Find
	UF labels_uf = UF(2000);	//Inicialmente, hay muchas etiquetas, que después se reducirán
	//Parámetros
    float theta = PI/2 - phi;	//Angulo con la vertical
    float m = tan(theta);
    cout<<"phi: "<<phi*180/PI<<"; theta: "<<theta*180/PI<<endl;
    float z_p, x_p, y_p, h_t, h_m, Norm;	
    // Usando pointer iterator 
    int u_min, u_max;
    int debug = 0;	
	int n_et = 0; // # etiquetas encontradas en total en la imágen
    for(int i = _CoordImage.rows-1; i >=0; --i){
		for(int j = _CoordImage.cols-1; j >= 0;  --j){
			z_p = _CoordImage(i,j).z;
			if(z_p > 0){	//Para no procesar puntos con error en el cáclulo de la disparidad
				x_p = _CoordImage(i,j).x;
				y_p = _CoordImage(i,j).y;
				float P_i[2] = {(float)j, (float)i}; //(x, y) en la imagen
				float P[3] = {x_p, y_p, z_p};
				h_t = H_t * f * scale / z_p;
				h_m = H_m * f * scale / z_p;
				bool is_p_obstacle = 0;
				//u_Max-u_Min = 2*m*h_m ; area triangulo = h*h/2
				Point S_p[int(h_m)*int(m*h_m)]; // Conjunto compatible con p (sin etiquetar todavia) (reservo espacio de sobra)
				int n_S_p = 0;
				int et_trp[50];	// Etiquetas del conjunto compatible con p (ya etiquetados) (reservo espacio de sobra)
				int n_et_trp = 0; // Número de elementos
				for (int v = std::max(int(round(P_i[1]-h_m)), 1); v < std::min(int(round(P_i[1]-h_t)), _CoordImage.rows); ++v ){
					u_min = int(round(m*(v-P_i[1])+P_i[0]));
					u_max = int(round(-m*(v-P_i[1])+P_i[0]));
					for (int u=std::max(u_min, 1); u < std::min(u_max, _CoordImage.cols); ++u){
						float P_2[3] = {_CoordImage(v,u).x, _CoordImage(v,u).y, _CoordImage(v,u).z};

						//Calculo si se cumplen las dos condiciones que el articulo indica
						bool cond1 = (H_t < (P_2[1]-P[1])) && ((P_2[1]-P[1]) < H_m);
						Norm=sqrt((P_2[0]-P[0])*(P_2[0]-P[0])+(P_2[1]-P[1])*(P_2[1]-P[1])+(P_2[2]-P[2])*(P_2[2]-P[2]));
						bool cond2 = (abs(P_2[1]-P[1])/Norm) > sin(phi);
						if (cond1 && cond2){
							is_p_obstacle |= 1;	//Si al menos un punto es compatible => P es obstáculo
							//Si no tiene etiqueta, simplemente añado el punto a S_p
							if(labels_img.at<uchar>(v,u) == 0){
								S_p[n_S_p] = Point(v,u);
								n_S_p++;
							}
							//Si tiene etiqueta de un nuevo componente, añado a et_trp dicha etiqueta
							else{
								bool new_et = 1;
								for(int i = 0; i < n_et_trp; i++){
									if(labels_uf.connected((int)labels_img.at<uchar>(v,u), et_trp[i])){
										new_et = 0;
										break;
									}
								}
								if(new_et){
								et_trp[n_et_trp] = labels_img.at<uchar>(v,u);
								n_et_trp++;
								}
							}

						}

					}
				}
				// Si el punto base tiene ya una etiqueta y pertenece a nuevo componente, lo añado a et_trp
				if(labels_img.at<uchar>(P_i[1],P_i[0]) != 0){
					bool new_et = 1;
					for(int i = 0; i < n_et_trp; i++){
						if(labels_uf.connected((int)labels_img.at<uchar>(P_i[1],P_i[0]), et_trp[i])){
							new_et = 0;
							break;
						}
					}
					if(new_et){
					et_trp[n_et_trp] = labels_img.at<uchar>(P_i[1],P_i[0]);
					n_et_trp++;
					}
				}
				//Si no la tiene, pero es punto de obstáculo, lo añado a S_p
				else if(is_p_obstacle){
					S_p[n_S_p] = Point(P_i[1],P_i[0]);
					n_S_p++;
				}
				// Caso de que p NO sea obstáculo: No hago nada (dejo todos los píxeles del trapecio a 0)

				//Caso de que p sea obstáculo:
				// 1. Si no hay ningun punto etiquetado (incluyendo el punto base)
				// Etiqueto todos con ETIQUETA NUEVA
				if((n_et_trp == 0) && (is_p_obstacle)){
					n_et++;	//Darse cuenta de que la etiqueta cero, es no-obstaculo
					for(int p = 0; p < n_S_p; p++){
						labels_img.at<uchar>(S_p[p].x, S_p[p].y) = n_et;
					}
				}

				// 2. Si hay solo una etiqueta en el trapecio entero(incluyendo el punto base)
				// Etiqueto todos con esa etiqueta
				if(n_et_trp == 1 && (is_p_obstacle)){
					for(int p = 0; p < n_S_p; p++){
						labels_img.at<uchar>(S_p[p].x, S_p[p].y) = et_trp[0];
					}
				}

				//3. Si hay un conflicto de etiquetas
				if(n_et_trp >1 && (is_p_obstacle)){
					//Junto todas en la estructura UF
					for(int n_label_trp = 1; n_label_trp < n_et_trp; n_label_trp++){
						//cout<<"merge "<< et_trp[0]<<" with "<<et_trp[n_label_trp]<<endl;
						labels_uf.merge(et_trp[0], et_trp[n_label_trp]);
						debug++;
					}
					//Etiqueto con et_trp[0] los puntos del trapecio que no están etiquetados
					for(int p = 0; p < n_S_p; p++){
						labels_img.at<uchar>(S_p[p].x, S_p[p].y) = et_trp[0];
					}
				}
			}
		}
	}

    n_roots = 0; //Vuelvo a rellenar roots

    //Substituyo todos los colores por el color representando la raíz del componente Union-Find
    for(int v = 0; v < labels_img.rows; v++){
		for(int u = 0; u < labels_img.cols; u++){
			if (labels_img.at<uchar>(v,u) != labels_uf.find(int(labels_img.at<uchar>(v,u)))){
				labels_img.at<uchar>(v,u) = labels_uf.find(int(labels_img.at<uchar>(v,u)));
			}
			else{
				// Si es raiz, miro a ver si estaba en roots, y si no la añado siempre que no sea cero
				bool new_root = 1;
				for(int i = 0;i<n_roots;i++){
					if((roots[i]==(int)labels_img.at<uchar>(v,u))&&((int)labels_img.at<uchar>(v,u)!=0)){
						new_root=0;
						break;
					}
				}
				if (new_root&&(labels_img.at<uchar>(v,u)!=0)){
					roots[n_roots] = (int)labels_img.at<uchar>(v,u);
					cout<<"Se añadio la raiz "<<roots[n_roots]<<endl;
					n_roots++;
					cout<<"Numero de roots="<<n_roots<<endl;
				}
			}
		}
	}
    cout<<"Numero de componentes encontrados (no representativos)="<<n_et<<endl;
    // Mediante una LUT remapeo valores de labels para que queden consecutivos
    // Construyo la LUT
    for(int i = 0;i<n_roots;i++){
    	lut.at<uchar>(roots[i])=i;
    }
    lut.at<uchar>(0)=0;
    // Aplico la LUT
    LUT(labels_img,lut,labels_img);

    return labels_uf;
}

vector<Rect> DrawObstacles(Mat_<Point3f> &_CoordImage, Mat &obstacles, Mat &obstacles_contours, Mat &disp_32F, float b, float f, float min_area){
	//Find contours and mark their bounding boxes
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		int n_big_obstacles = 0;

		cout<<"Se van a buscar los contornos"<<endl;
		findContours(obstacles, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		cout<<"Se encontraron "<<contours.size()<<" contornos"<<endl;
		vector<vector<Point> > contours_poly( contours.size() );	//Poligonalizar contornos
		vector<Rect> boundRect;	//ROI's
		vector<Point3f> coordsContours;
		//El siguiente bucle deshecha obstáculos muy pequeños
		for( uint i = 0; i < contours.size(); i++ ){
					double area_img = contourArea(contours[i]);	//Area en la imagen (px²)
					float area_real = 0;
					float u_c, v_c;
					Moments M = moments(contours[i], false);
					if (M.m00 != 0){
						u_c = M.m10/M.m00;	//u_c en el centro del contorno
						v_c = M.m01/M.m00;	//v_c en el centro del contorno
						//cout<<"M.m00: "<<M.m00<<"; u_c: "<<u_c<<"; v_c: "<<v_c<<endl;
						//Ahora calculo el área en metros cuadrados
						area_real = area_img*(b/disp_32F.at<float>(v_c,u_c))*(b/disp_32F.at<float>(v_c,u_c));
					}
			if (area_real > min_area){
				approxPolyDP( Mat(contours[i]), contours_poly[n_big_obstacles], 3, true );
				//boundRect[n_big_obstacles] = boundingRect( Mat(contours_poly[n_big_obstacles]) );
				boundRect.push_back(boundingRect( Mat(contours_poly[n_big_obstacles])));
				coordsContours.push_back(_CoordImage(v_c,u_c));
				n_big_obstacles++;
			}
		}
		cout<<"Se crearon "<<n_big_obstacles<<" bounding boxes"<<endl;
		for( int i = 0; i< n_big_obstacles; i++ ){
			drawContours( obstacles_contours, contours_poly, i, Scalar(255,0,0), 1, 8, vector<Vec4i>(), 0, Point() );
			rectangle( obstacles_contours, boundRect[i].tl(), boundRect[i].br(), Scalar(0,255,0), 2, 8, 0 );
			//Componer y mostrar posicion del centro del obstáculo
			ostringstream ss;
			ss << coordsContours[i].x<<","<<coordsContours[i].y<<","<<coordsContours[i].z;
			std::string s(ss.str());
//			putText(obstacles_contours, s, Point(boundRect[i].tl().x,boundRect[i].tl().y+10),
//					FONT_HERSHEY_SCRIPT_COMPLEX, 0.5, CV_RGB(255,0,0), 1);
			//cout<<"Dibujada Bounding Box ("<<boundRect[i].tl()<<","<<boundRect[i].br()<<")"<<endl;
		}
		return boundRect;
}

int main(int argc, char *argv[]){
	cout << "OpenCV version : " << CV_VERSION << endl;
	cout << "Author : Alvaro Gregorio Gómez" << CV_SUBMINOR_VERSION << endl;
	cout << "Usage: process_sequence video_path_input output_path_input H_t H_m theta start_frame end_frame"<<endl;
	cout << "Se escribe en el path de salida un archivo txt con los parametros usados"<<endl;
	cout << "Se escribe en el la carpeta labels un archivo txt por imágen con la escala"<<endl<<"en primera linea y en lineas sucesivas las etiquetas de obstaculos"<<endl;
	cout << "TODAS LAS CARPETAS DE ENTRADA Y SALIDA DEBEN DE ESTAR YA CREADAS EN EL SISTEMA DE ARCHIVOS"<<endl;
			/************************************************************************\
			|Parametros & variables para el algoritmo					  |
			\************************************************************************/
	float H_t;
	float H_m;
	float phi;
	int initial_frame = 0;
	int final_frame = 100;

	// Parámetros por defecto
	if(argc!=8){
		H_t = 0.4; //0.4
		H_m = 0.9; //0.9
		phi = 45*PI/180;  //Angulo con la horizontal
	}
	else{
		H_t = atof(argv[3]);
		H_m = atof(argv[4]);
		phi = atof(argv[5]); //Angulo con la horizontal
		initial_frame = (int)atof(argv[6]);
		final_frame = (int)atof(argv[7]);
	}

	// Matrices de calibracion
		float P0[3][3]={
				{7.215377e+02, 0.000000e+00, 6.095593e+02},
				{0.000000e+00, 7.215377e+02, 1.728540e+02},
				{0.000000e+00, 0.000000e+00, 1.000000e+00}
		};
		float P2[3][3]={
				{7.215377e+02, 0.000000e+00, 6.095593e+02},
				{0.000000e+00, 7.215377e+02, 1.728540e+02},
				{0.000000e+00, 0.000000e+00, 1.000000e+00}
		};
		float f = P0[0][0];	//distancia focal
		cout<<"f = "<<f<<endl;
		float T[3]={5.956621e-02, 0, 0};
		float b = T[0];	//distancia base
		cout<<"b = "<<b<<endl;
		float R[3][3]={
				{1, 0, 0},
				{0, 1, 0},
				{0, 0, 1}
		};

	string image_name_left, image_name_right, path_input, path_output, params_file_name;
	ofstream algorithm_params_file;

	// Directorios por defecto
	if(argc!=8){
		cout<<"Introduce la carpeta a tratar (por defecto pulsar d, /home/alvaro/video/)";
		cin>>path_input;
		if(path_input=="d")
			path_input="/home/alvaro/video/";
		// Escribo parametros en archivo
		path_output="/home/alvaro/sequence/";
		params_file_name=path_output+"parametros_algoritmo.txt";
		algorithm_params_file.open(params_file_name.c_str());
		algorithm_params_file<<"H_t: "<<H_t<<"H_m: "<<endl<<H_m<<"phi: "<<endl<<phi<<"Z_MAX: "<<Z_MAX<<endl;
		algorithm_params_file.close();
	}
	else{
		path_input=argv[1];	// Input folder
		// Escribo parametros en archivo
		path_output=argv[2];
		params_file_name=path_output+"parametros_algoritmo.txt";
		algorithm_params_file.open(params_file_name.c_str());
		algorithm_params_file<<"H_t: "<<H_t<<endl<<"H_m: "<<H_m<<endl<<"phi: "<<phi<<endl<<"Z_MAX: "<<Z_MAX<<endl;
		algorithm_params_file.close();
	}
	char image_number[12];
	// Empiezo el bucle para procesar todos los frames para ambas cámaras
	for(int i=initial_frame;i<=final_frame;i++){
		sprintf(image_number,"%010d",i);
		image_name_left=path_input+"image_02/data/"+(string)image_number+".png";
		image_name_right=path_input+"image_00/data/"+(string)image_number+".png";
		Mat left_frame = imread(image_name_left.c_str(), IMREAD_GRAYSCALE);
		Mat right_frame = imread(image_name_right.c_str(), IMREAD_GRAYSCALE);

		if( left_frame.empty() ) // Check de imágenes invalidas
		{
			cout <<  "No se ha podido abrir la imágen izquierda" << endl ;
			cout << image_name_left << endl;
			return -1;
		}
		if( right_frame.empty() ) // Check de imágenes invalidas
		{
			cout <<  "No se ha podido abrir la imágen derecha" << endl ;
			cout << image_name_left << endl;
			return -1;
		}
		//Escalo las imágenes para acelerar
		double scale = 0.7;	// Parámetro de escalado, para procesar mas rapido las imagenes
		resize(left_frame, left_frame, Size(0,0), scale, scale);
		resize(right_frame, right_frame, Size(0,0), scale, scale);

		// Quitar comentario para ver la imagen izquierda
		//namedWindow( "Left frame", WINDOW_AUTOSIZE );
		//imshow("Left frame", left_frame);

		//Stereo disparity usando Semi-global block matching algorithm
		StereoSGBM sgbm;

		//Parámetros del SGBM(Semi-Global Block matching)
		int numberOfDisparities = 64;//((img_size.width/8) + 15) & -16;	//Hace la primera operación y redondea haciendo el LSNibble cero (divisible entre 16)
		sgbm.preFilterCap = 25;
		sgbm.SADWindowSize = 14;
		//int cn = left_frame.channels();
		sgbm.P1 = 8*sgbm.SADWindowSize*sgbm.SADWindowSize;
		sgbm.P2 = 32*sgbm.SADWindowSize*sgbm.SADWindowSize;
		sgbm.minDisparity = 0;
		sgbm.numberOfDisparities = numberOfDisparities;	//Con 16 se aprovechan los 8 bits de imágen por completo
		sgbm.uniquenessRatio = 10;
		sgbm.speckleWindowSize = 100;
		sgbm.speckleRange = 2;
		sgbm.disp12MaxDiff = 1;
		sgbm.fullDP = false;

		Mat disp_matlab, disp, disp8U, disp_32F;
		//Generamos la imagen de disparidades
		sgbm(left_frame, right_frame, disp);	//disp tiene los valores de disparidades escalados por 16
		disp_matlab=disp;
		disp.convertTo(disp,CV_16U);	//Le quito el signo ( los erores serán 0 ahora, no negativo)

		// (Quitar comentario para incorporar filtro para que sean mas suaves los cambios en la disparidad)
		// blur(disp, disp, Size(1,16));
		// GaussianBlur(disp, disp, Size (1, 15), 1, 5);

		// Convierto a valores reales
		disp.convertTo(disp_32F,CV_32F);
		disp_32F = disp_32F / 16.0f;	//Disparidad en valores reales

		// (Quitar comentarios para mostrar la imágen de disparidad)
		disp.convertTo(disp8U,CV_8U);	// Mucho OJO, porque esta imagen está truncada, no usar para calculos, solo para mostrarla
		// namedWindow( "Disparity", WINDOW_AUTOSIZE );
		// imshow("Disparity", disp8U);

		// Centro de la imágen de disparidad
		float cd_u=float(disp.cols)/2;
		float cd_v=float(disp.rows)/2;
		cout<<"Center u: " << cd_u <<endl;
		cout<<"Center v: " << cd_v <<endl;

		//Mapa de coordenadas en formato Mat y en formato Point3f
		Mat CoordImage;
		CoordImage=Mat::zeros(disp.rows, disp.cols, CV_32FC3);
		Mat_<Point3f> _CoordImage = CoordImage;
		Coords(CoordImage, _CoordImage, disp_32F, b,f,cd_u,cd_v, scale);

		// Imágen de etiquetas
		Mat labels_img;
		labels_img=Mat::zeros(disp.rows, disp.cols, CV_8U);

		// Llamo al algoritmo de los conos invertidos
		UF labels_uf = InvertedCones(_CoordImage, labels_img, phi, H_t, H_m, f, scale);

		// Como los valores de las etiquetas son bajos, debo escalarla para visualizarla
		//labels_img *= 2;

		// Imágen binarizada de obstáculos
		Mat labels_img_bin;

		threshold( labels_img, labels_img_bin, 0, 255, THRESH_BINARY );

		// Guardo resultados, para procesar después
		string disparity_name, labels_name, labels_bin_name;
		disparity_name= path_output+"disparity/"+(string)image_number+".png";
		labels_name=path_output+"labels/"+(string)image_number+".png";
		labels_bin_name=path_output+"labels_bin/"+(string)image_number+".png";
		imwrite( disparity_name, disp_matlab );
		imwrite( labels_name, labels_img );
		imwrite( labels_bin_name, labels_img_bin );


		//Guardo tambien archivos de texto con los identificadores de obstaculo (roots) y la escala
		ofstream labels;
		labels.open(string(path_output+"labels/"+(string)image_number+".txt").c_str());
		labels<<scale<<endl<<n_roots<<endl<<endl;

		for(int i=0; i<n_roots; i++){
			labels<<roots[i]<<endl;
		}
		labels.close();
		cout<<"labels encontradas: "<<n_roots<<endl;
		cout<<"Se escribieron los datos para la imagen "<<(string)image_number<<endl;


		// Descomentar el siguiente código para buscar las bounding boxes en este programa en lugar de en MATLAB
		// Y para activar el detector de personas HOG


		/* NO MARCO LAS BOUNDING BOXES, LO HARÉ EN MATLAB
		 * //Find contours and mark their bounding boxes
		float min_area = 0.5*0.5;	//m² minimos para ser considerado un obstáculo
		Mat obstacles_contours = Mat::zeros( labels_img.size(), CV_8UC3 );
		vector<Rect> boundRect;
		boundRect = DrawObstacles(_CoordImage, labels_img, obstacles_contours, disp_32F, b, f, min_area);

		//Inicializo detector de gradientes orientados con el detector de personas ya entrenado
		vector<Rect> pedestrians, currentROIPedestrians;
		HOGDescriptor hog;
		hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
		//Mat ROI;
		for( size_t i = 0; i < boundRect.size(); i++ ){
			//Amplio la ROI del obstáculo
			Rect currentBBox(max(boundRect[i].x-30, 0), max(boundRect[i].y-40, 0), min(boundRect[i].width+20, left_frame.cols), min(boundRect[i].height+20, left_frame.rows));
			if((currentBBox.width>64)&&(currentBBox.height>128)){//Tamaño mínimo de la ventana de detección
				hog.detectMultiScale(left_frame(currentBBox), currentROIPedestrians, 0, Size(8,8), Size(0,0),1.05, 2);
				for(uint j = 0; j<currentROIPedestrians.size();j++){
					//Cálculo de las coordenadas desde la referencia global
					currentROIPedestrians[j]=Rect(currentROIPedestrians[j].x+currentBBox.x, currentROIPedestrians[j].y+currentBBox.y,
							currentROIPedestrians[j].width, currentROIPedestrians[j].height);
					pedestrians.push_back(currentROIPedestrians[j]);
					cout << "Peatón detectado HOG= " << Mat(currentROIPedestrians) << endl << endl;
				}


			}
		}
		//Dibujo las bounding boxes para las personas
		for( uint i = 0; i< pedestrians.size(); i++ ){
			rectangle( obstacles_contours, pedestrians[i].tl(), pedestrians[i].br(), Scalar(0,0,255), 2, 8, 0 );
		}



		//Show it
		Mat results_img;
		resize(obstacles_contours, obstacles_contours, left_frame.size());
		cvtColor(left_frame, left_frame, CV_GRAY2RGB);
		addWeighted( obstacles_contours, 1, left_frame, 1, 0.0, results_img);

		/// Show the bounding boxes in a window
		namedWindow( "Bounding boxes", CV_WINDOW_AUTOSIZE );
		imshow( "Bounding boxes", results_img );
		imwrite("/home/alvaro/resultadov4.png", results_img);*/

		int c = waitKey(5);
		if( (char)c == 27 )
			return 0;
		else if((char)c == 's'){	//s stops playing the video
			int c = waitKey(0);
		}

	}

	return 0;
}

