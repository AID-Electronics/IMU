#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <utility/imumaths.h>
                                                        // DATOS FIJOS DEL SISTEMAS DE POLEAS Y ACTUADORES
#define RESOLUCION 1.8  //GRADOS POR PASO
#define RADIO_POLEA 25 //mm
#define ALTURA_POLEAS 360 //mm
#define D_REF 333//mm

#define DIST 50

#define H 360

/*En este código se probará la recepción de los datos del sensor de orientación de la IMU 
 * Adafruit BNO055.
 * Se realizará la obtención de los datos a través de la biblioteca de Adafruit y la obtención 
 * del dato numérico en crudo.
 */

 const double pi=3.141592;



Adafruit_BNO055 bno = Adafruit_BNO055(55);
void displayCalStatus ()
{
  /*Cogemos los cuatro velores de calibración (0...3)
   * Cualquier sensor cuyo valor sea 0 es ignorado,
   * 3 significa calibrado.
   */
   uint8_t system, gyro, accel, mag;
   system = gyro = accel = mag = 0;
   bno.getCalibration(&system, &gyro, &accel, &mag);

   Serial.print("\t");
   if(!system)
    Serial.print("! ");

   Serial.print("Sys: ");
   Serial.print(system, DEC);
   Serial.print (" G: ");
   Serial.print (gyro, DEC);
   Serial.print (" A: ");
   Serial.print (accel, DEC);
   Serial.print (" M: ");
   Serial.println(mag, DEC);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);
  Serial.println("Prueba Orientación Sensor"); Serial.println(" ");

  if(!bno.begin())
  {
    Serial.print("BNO055 no detectado");
    while(1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);

  displayCalStatus();
}

void loop() {

  double cabeceoAnterior=0;
  double cabeceoPosterior=0;
  double alabeoAnterior=0;
  double alabeoPosterior=0;

  int pasosMotor1;
  int pasosMotor2;
  int pasosMotor3;
  int pasosMotor4;
  
  int TOL=2;
  double deg2rad= pi/180;
  sensors_event_t event;
  bno.getEvent (&event);

  Serial.print ("X: ");
  Serial.print (event.orientation.x,4);
  Serial.print ("\tY: ");
  Serial.print (event.orientation.y,4);
  Serial.print ("\tZ: ");
  Serial.println (event.orientation.z,4);

  


  cabeceoPosterior=event.orientation.y*deg2rad; //No estoy demasiado seguro de que sea el eje correcto
  alabeoPosterior=event.orientation.z*deg2rad;
  
  if(abs(cabeceoPosterior-cabeceoAnterior)>TOL || abs(alabeoPosterior-alabeoAnterior)>TOL ) //Esta sentencia se puede omitir
  {
  
pasosMotor1=calcularPasos2D(cabeceoPosterior-cabeceoAnterior,alabeoPosterior-alabeoAnterior,RESOLUCION,RADIO_POLEA,H,333,0,D_REF);
pasosMotor2=calcularPasos2D(cabeceoPosterior-cabeceoAnterior,alabeoPosterior-alabeoAnterior,RESOLUCION,RADIO_POLEA,H,0,333,D_REF);
pasosMotor3=calcularPasos2D(cabeceoPosterior-cabeceoAnterior,alabeoPosterior-alabeoAnterior,RESOLUCION,RADIO_POLEA,H,-333,0,D_REF);
pasosMotor4=calcularPasos2D(cabeceoPosterior-cabeceoAnterior,alabeoPosterior-alabeoAnterior,RESOLUCION,RADIO_POLEA,H,0,-333,D_REF);
  
  
  //AQUI iría la accion de movimiento
  
  
  cabeceoAnterior=cabeceoPosterior;
  alabeoAnterior=alabeoPosterior;
  }

    Serial.print ("Pasos motor 1: ");
  Serial.print (pasosMotor1);
      Serial.print ("        Pasos motor 2: ");
  Serial.print (pasosMotor2);
  Serial.print ("      Pasos motor 3: ");
  Serial.print (pasosMotor3);
  Serial.print ("    Pasos motor 4: ");
  Serial.print (pasosMotor4);
  Serial.println(" ");
  delay (200);


/*
     Serial.print ("Pasos motor 1: ");
  Serial.print (calcularPasos2D(18*deg2rad,15*deg2rad,RESOLUCION,RADIO_POLEA,H,0,333,D_REF));           ESTO ES PARA PROBAR Y COMPARAR CON LA FORMULA DEL EXCEL
      Serial.print ("        Pasos motor 2: ");
  Serial.print (calcularPasos2D(18*deg2rad,15*deg2rad,RESOLUCION,RADIO_POLEA,H,333,0,D_REF));
  Serial.print ("      Pasos motor 3: ");
  Serial.print (calcularPasos2D(18*deg2rad,15*deg2rad,RESOLUCION,RADIO_POLEA,H,0,-333,D_REF));
  Serial.print ("    Pasos motor 4: ");
  Serial.print (calcularPasos2D(18*deg2rad,15*deg2rad,RESOLUCION,RADIO_POLEA,H,-333,0,D_REF));
  Serial.println(" "); */
}




int calcularPasos1D(double cabeceo,double resolucion,double radioPolea,double distCentro)
{ 
  double PASOS;
  double tangente= tan(cabeceo);
  PASOS=(((tangente*distCentro)/(2*pi*radioPolea))*(360/resolucion)) ;

  
  int aux=int(PASOS);
  
  
  double aux2=abs(PASOS)-abs(aux);
  

  
  if(abs(aux2)>0.5)
    {
      if (aux>0)
      aux++;
      else
      aux--;
    }
  

  
  return aux;
  
}


int calcularPasos2D(double cabeceo,double alabeo ,double resolucion,double radioPolea,double h,double posX, double posY,double Dref)
{
    double PASOS;
  double tangenteCAB= tan(cabeceo);  //ES EL ANGULO RESPECTO EL EJE X
  double tangenteAL= tan(alabeo); //ES EL ANGULO RESPECTO EL EJE Y
  double numerador= (sqrt((tangenteCAB*h-posY)*(tangenteCAB*h-posY)+(tangenteAL*h-posX)*(tangenteAL*h-posX))-Dref);
  
  PASOS=((numerador/(2*pi*radioPolea))*(360/resolucion)) ;

  
  int aux=(int)PASOS;
  double aux2=PASOS-aux;

    if(abs(aux2)>0.5)
    {
      if (aux>0)
      aux++;
      else
      aux--;
    }
  
  
  
  return aux;
  
  
}

