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

/*En este código se probará la recepción de los datos del sensor de orientación de la IMU 
 * Adafruit BNO055.
 * Se realizará la obtención de los datos a través de la biblioteca de Adafruit y la obtención 
 * del dato numérico en crudo.
 */



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

  int pasosMotor1;
  int pasosMotor2;
  
  int TOL=2;
 
  sensors_event_t event;
  bno.getEvent (&event);

  Serial.print ("X: ");
  Serial.print (event.orientation.x,4);
  Serial.print ("\tY: ");
  Serial.print (event.orientation.y,4);
  Serial.print ("\tZ: ");
  Serial.println (event.orientation.z,4);

  


  cabeceoPosterior=event.orientation.x //No estoy demasiado seguro de que sea el eje correcto

  if(abs(cabeceoPosterior-cabeceoAnterior)>TOL) //Esta sentencia se puede omitir
  {
  pasosMotor1=calcularPasos1D(cabeceoPosterior-cabeceoAnterior,RESOLUCION,RADIO_POLEA,DIST);
  pasosMotor2=calcularPasos1D(cabeceoPosterior-cabeceoAnterior,RESOLUCION,RADIO_POLEA,-DIST);
  
  //AQUI iría la accion de movimiento
  
  
  cabeceoAnterior=cabeceoPosterior;
  }
  delay (200);
}




int calcularPasos1D(double cabeceo,double resolución,double radioPolea,double distCentro)
{ 
  double PASOS;
  double tangente= tan(cabeceo);
  PASOS=(((tangente*distCentro)/(2*pi*radioPolea))*(360/resolución)) ;

  
  int aux=(int)PASOS;
  double aux2=PASOS-aux;

  if(aux2>0.5)
  aux++;
  
  
  return aux;
  
}


int calcularPasos2D(double cabeceo,double alabeo ,double resolución,double radioPolea,double h,double posX, double posY,double Dref)
{
    double PASOS;
  double tangenteCAB= tan(cabeceo);  //ES EL ANGULO RESPECTO EL EJE X
  double tangeteAL= tan(alabeo); //ES EL ANGULO RESPECTO EL EJE Y
  double numerador= (sqrt((tangenteCAB*h-posX)*(tangenteCAB*h-posX)+(tangenteAL*h-posY)*(tangenteAL*h-posY))-Dref);
  
  PASOS=((numerador/(2*pi*radioPolea))*(360/resolución)) ;

  
  int aux=(int)PASOS;
  double aux2=PASOS-aux;

  if(aux2>0.5)
  aux++;
  
  
  return aux;
  
  
}

