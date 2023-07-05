#include <math.h>
#include <stdio.h>

#define M 4
#define SIZE 3  //Grados de libertad

//Distancia conocida entre juntas. Son a1, a2 y a3 respectivamente
float dimensiones[SIZE] = { 7.7, 8.9, 3.6 };

//Definicion de prototipos de las funciones
void cinemdirecta(float ang1, float ang2, float ang3);
void cinemInversa(float x, float y, float z);
void imprimir_matriz(double matriz[M][M]);

void imprimir_matriz(double matriz[M][M]) {
int i, j;
for(i = 0; i < M; i++) {
  for(j = 0; j < M; j++) {
    Serial.print(matriz[i][j]);
    Serial.print("\t");
    }
  Serial.println("\n");
  }
  Serial.print("-------------------------------------\n");
}

void cinemdirecta(float ang1, float ang2, float ang3)
{
    double alpha1 = PI / 2;
    double alpha2 = 0.0;
    double alpha3 = 0.0;
    char buffer[10];
    float x,y,z;
    //Convertir angulos a radianes
    float theta1= ang1 * (PI / 180.0);
    float theta2= ang2 * (PI / 180.0);
    float theta3= ang3 * (PI / 180.0);
    //-----------------------------------
    //Calculo de las matrices homogeneas de cada marco de referencia
    double H01[4][4] = {{cos(theta1), -cos(theta1)*sin(alpha1), sin(theta1)*sin(alpha1), 0.0},
                        {sin(theta1), cos(theta1)*cos(alpha1), -cos(theta1)*sin(alpha1), 0.0},
                        {0.0, sin(alpha1), cos(alpha1), dimensiones[0]},
                        {0.0, 0.0, 0.0, 1.0}};

    double H12[4][4] = {{cos(theta2), -cos(theta2)*sin(alpha2), sin(theta2)*sin(alpha2), dimensiones[1]*cos(theta2)},
                        {sin(theta2), cos(theta2)*cos(alpha2), -cos(theta2)*sin(alpha2), dimensiones[1]*sin(theta2)},
                        {0.0, sin(alpha2), cos(alpha2), 0.0},
                        {0.0, 0.0, 0.0, 1.0}};

    double H23[4][4] = {{cos(theta3), -cos(theta3)*sin(alpha3), sin(theta3)*sin(alpha3), dimensiones[2]*cos(theta3)},
                        {sin(theta3), cos(theta3)*cos(alpha3), -cos(theta3)*sin(alpha3), dimensiones[2]*sin(theta3)},
                        {0.0, sin(alpha3), cos(alpha3), 0.0},
                        {0.0, 0.0, 0.0, 1.0}};

    double H03[4][4] = {0};

    //Imprimir matrices
    imprimir_matriz(H01);
    imprimir_matriz(H12);
    imprimir_matriz(H23);

    for (int i = 0; i < 4; i++)
     {
        for (int j = 0; j < 4; j++)
         {
            for (int k = 0; k < 4; k++) 
            {
                H03[i][j] += H01[i][k] * H12[k][j];
            }
        }
    }

    for (int i = 0; i < 4; i++)
     {
        for (int j = 0; j < 4; j++) 
        {
            H03[i][j] *= H23[i][j];
        }
    }
    imprimir_matriz(H03);
    
    //Se guardan los valores de las posiciones finales para luego ser ingresadas en
    //el calculo de la cinematica inversa
    x = H03[0][3];
    y = H03[1][3];
    z = H03[2][3];

  // Imprimir los resultados por la comunicación serial
  Serial.println("Posición final:");
  Serial.print("X=");
  dtostrf(H03[0][3], 8, 5, buffer);
  Serial.println(buffer);
  Serial.print("Y= ");
    //Serial.println(y);
 dtostrf(H03[1][3], 8, 5, buffer);
  Serial.println(buffer);
  Serial.print("Z= ");
  //  Serial.println(z);
   dtostrf(H03[2][3], 8, 5, buffer);
  Serial.println(buffer);
  delay(5000);

//Calculo de la cinematica inversa en funcion de la posicion obtenida
  cinemInversa(x,y,z);
  delay(10000); // Delay de 10 segundos finales antes de volver a comenzar el programa
  
}

void cinemInversa(float x, float y, float z){
  
  float angulos[SIZE] = { 0.0 };  //Angulos resultantes
  char buffer[10];

  //CALCULO DE DISTANCIAS 'R'
  float r1 = sqrt(pow(x,2) + pow(y,2));  // raiz de x0^2 + y0^2
  float r2 = z - dimensiones[0];        // Z0 - a1
  float r3 = sqrt(pow(r1,2) + pow(r2,2));

  //CALCULO DE ANGULOS 'PHI'
  float phi1 = acosf((pow(dimensiones[2],2) - pow(dimensiones[1],2) - pow(r3,2)) / (-2.0 * dimensiones[1] * r3) * (180.0/PI)) ;
  float phi2 = atan(r2 / r1) * (180.0/PI);
  float phi3 = acosf((pow(r3,2) - pow(dimensiones[1],2) - pow(dimensiones[2],2)) / (-2.0 * dimensiones[1] * dimensiones[2])) * (180.0/PI);
  
  //CALCULO DE ANGULOS 'THETA'
  angulos[0] = atan(y / x) * (180/PI);
  angulos[1] = phi2 - phi1;
  angulos[2] = 180.0 - phi3;
  
  Serial.println("");
  Serial.println("Los angulos resultantes son:");
  Serial.println("----------------------------");
  for (int i = 0; i < 3; i++) {
    Serial.print("theta ");
    Serial.print(i + 1);
    Serial.print("       : ");
    dtostrf(angulos[i], 8, 5, buffer);
    Serial.print(buffer);
    Serial.println(" grados");
  }
  Serial.println("----------------------------");
}

void setup()
 {
  float ang1, ang2, ang3= 0.0;
  Serial.begin(9600);
  Serial.setTimeout(20000); //establece cuando tiempo va a estar esperando que se ingrese algun dato por el puerto serie
}

void loop()
 {
  // Calcular la posición y orientación del sistema de coordenadas final
  //Ingresar por puerto serie los valores de los Theta correspondientes uno a la vez
  
    Serial.println("Ingresar el angulo de Theta1: ");
    double ang1 = Serial.parseFloat();
    Serial.println("Ingresar el angulo de Theta2: ");
    double ang2= Serial.parseFloat();
    Serial.println("Ingresar el angulo de Theta3: ");
    double ang3= Serial.parseFloat();
    
  //Calculo de la cinematica directa en funcion de los angulos ingresados
    cinemdirecta(ang1, ang2, ang3);
    
  delay(1000);
}
