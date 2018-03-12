#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <utility/imumaths.h>

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
  
  sensors_event_t event;
  bno.getEvent (&event);

  Serial.print ("X: ");
  Serial.print (event.orientation.x,4);
  Serial.print ("\tY: ");
  Serial.print (event.orientation.y,4);
  Serial.print ("\tZ: ");
  Serial.println (180 + event.orientation.z,4);
  delay (100);
}
