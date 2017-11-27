#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "BMP085.h"
#include <Wire.h>
#include <SoftwareSerial.h> //시리얼통신 라이브러리 호출
/* 
 pin 
 3,4,5,6 -> 확장핀

 0 0 0 0 -> 0
 1 0 0 0 -> 1
 0 1 0 0 -> 2
 1 1 0 0 -> 3
 0 0 1 0 -> 4
 7,8 -> 압력센서 
 */
int blueTx=4;   //Tx (보내는핀 설정)
int blueRx=5;   //Rx (받는핀 설정)]
int angle = 90;
SoftwareSerial BTSerial(blueTx, blueRx);  //시리얼 통신을 위한 객체선언

static const char LED = 2;
static const float ACCEL_SENS = 16384.0; // Accel Sensitivity with default +/- 2g scale
static const float GYRO_SENS  = 131.0;   // Gyro Sensitivity with default +/- 250 deg/s scale

// Magnetometer class default I2C address is 0x1E
// specific I2C addresses may be passed as a parameter here
// this device only supports one I2C address (0x1E)
HMC5883L mag;
int16_t mx, my, mz;

// Accel/Gyro class default I2C address is 0x68 (can be 0x69 if AD0 is high)
// specific I2C addresses may be passed as a parameter here
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

//---------------------------------------------------------------------------//
//kalman class
class kalman {
  public :
    double getkalman(double acc, double gyro, double dt) {
      //project the state ahead
      angle += dt * (gyro - bias) ;

      //Project the error covariance ahead
      P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle) ;
      P[0][1] -= dt * P[1][1] ;
      P[1][0] -= dt * P[1][1] ;
      P[1][1] += Q_gyro * dt ;

      //Compute the Kalman gain
      double S = P[0][0] + R_measure ;
      K[0] = P[0][0] / S ;
      K[1] = P[1][0] / S ;

      //Update estimate with measurement z
      double y = acc - angle ;
      angle += K[0] * y ;
      bias += K[1] * y ;

      //Update the error covariance
      double P_temp[2] = {P[0][0], P[0][1]} ;
      P[0][0] -= K[0] * P_temp[0] ;
      P[0][1] -= K[0] * P_temp[1] ;
      P[1][0] -= K[1] * P_temp[0] ;
      P[1][1] -= K[1] * P_temp[1] ;

      return angle ;
    } ;
    void init(double angle, double gyro, double measure) {
      Q_angle = angle ;
      Q_gyro = gyro ;
      R_measure = measure ;

      angle = 0 ;
      bias = 0 ;

      P[0][0] = 0 ;
      P[0][1] = 0 ;
      P[1][0] = 0 ;
      P[1][1] = 0 ;
    } ;
    double getvar(int num) {
      switch (num) {
        case 0 :
          return Q_angle ;
          break ;
        case 1 :
          return Q_gyro ;
          break ;
        case 2 :
          return R_measure ;
          break ;
      }
    } ;
  private :
    double Q_angle, Q_gyro, R_measure ;
    double angle, bias ;
    double P[2][2], K[2] ;
} ;

kalman kal ;

double deg, dgy_y ;
double dt ;
uint32_t pasttime ;
double kalmanVal;
String sensorData;

//---------------------------------------------------------------------------//
//setup
void setup() {
  boolean state = HIGH;
  unsigned int count = 0;
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  BTSerial.begin(115200); //블루투스 시리얼

  while (!Serial && (count < 30) )
  {
    delay(200); // Wait for serial port to connect with timeout. Needed for native USB
    //digitalWrite(LED, state);
    state = !state;
    count++;
  }
  
  pinMode(13, OUTPUT);
  
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  pinMode(7, INPUT);    
  pinMode(8, INPUT);    
    
  digitalWrite(3, HIGH);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // ==================== MPU6050 ========  ====================
  accelgyro.initialize();
  Serial.print("Testing Accel/Gyro... ");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // Starts up with accel +/- 2 g and gyro +/- 250 deg/s scale
  accelgyro.setI2CBypassEnabled(true); // set bypass mode
  // Now we can talk to the HMC5883l

  // ==================== HMC5883L ============================
  mag.initialize();
  Serial.print("Testing Mag...  ");
  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
  Serial.println("Setup Complete");

//kalman
   kal.init(0.001, 0.003, 0.03) ;  //init kalman filter

}

//---------------------------------------------------------------------------//
//loop

void loop() {
  // put your main code here, to run repeatedly:

   static unsigned long ms = 0;
  static boolean state = HIGH;

  digitalWrite(3, HIGH);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);


  // Serial Output Format
  // === Accel === | === Gyro === | ======= Mag ======= | === Flex === |
  //   X   Y   Z   |  X   Y   Z   |  X   Y   Z  Heading |     Flex     |
  
  if (millis() - ms > 100)
  {
    // read raw accel/gyro measurements
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // display tab-separated accel/gyro x/y/z values
//    Serial.print("accel x : ");Serial.print(ax/ACCEL_SENS); Serial.print("\t");
//    Serial.print("y : "); Serial.print(ay/ACCEL_SENS); Serial.print("\t");
//    Serial.print("z : ");Serial.print(az/ACCEL_SENS); Serial.print("\t");
//    Serial.print("gyro x : ");Serial.print(gx/GYRO_SENS); Serial.print("\t");
//    Serial.print("y : "); Serial.print(gy/GYRO_SENS); Serial.print("\t");
//    Serial.print("z : ");Serial.print(gz/GYRO_SENS); Serial.print("\t");

    // read raw heading measurements
    mag.getHeading(&mx, &my, &mz);

//    // display tab-separated mag x/y/z values
//    Serial.print("mag x : "); Serial.print(mx); Serial.print("\t");
//    Serial.print("y : "); Serial.print(my); Serial.print("\t");
//    Serial.print("z : "); Serial.print(mz); Serial.print("\t");
    
    // To calculate heading in degrees. 0 degree indicates North
    float heading = atan2(my, mx);
    if(heading < 0) heading += 2 * M_PI;
    //Serial.print(heading * 180/M_PI); Serial.print("\t");

    //Serial.println("\t");
    ms = millis();
    digitalWrite(LED, state);
    state = !state;

    //Flex_sensor_val
    int indexfingerFlex = analogRead(A0);
    int middlefingerFlex = analogRead(A1);

    //finallllllllllllllly
    sensorData = "flex " + String(indexfingerFlex) + " " + String(middlefingerFlex) + ":" + 
                 "kalman " + String(kalmanVal) + ":" + 
                 "mag " + String(mx) +  " " + String(my) + " " + String(mz);
    
    
    Serial.print(sensorData);
  }
  
    //kalman filter
    deg = atan2(ax/ACCEL_SENS, az/ACCEL_SENS) * 180 / PI ;  //acc data to degree data
    dgy_y = gy/GYRO_SENS / 131. ;  //gyro output to

    dt = (double)(micros() - pasttime) / 1000000;
    pasttime = micros();  //convert output to understandable data

    kalmanVal = kal.getkalman(deg, dgy_y, dt) ;  //get kalman data
  //digitalWrite(LED, HIGH);//led = 6->2
  
  
//Alive test  
  //digitalWrite(13, LOW);
}
