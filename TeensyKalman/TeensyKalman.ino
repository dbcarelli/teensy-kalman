#include "MPU6050.h"

double GYROSCOPE_SENSITIVITY = 32.8;
double ACCELEROMETER_SENSITIVITY = 4096;
unsigned long newTime;

MPU6050 imu;
int16_t ax, ay, az, gx, gy, gz;

double accelX, accelY, accelZ, gyroX, gyroY, gyroZ, aRoll, aPitch;
double ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
double axStd, ayStd, azStd, gxStd, gyStd, gzStd;
double rollStd, pitchStd;

double predictedRoll;
double predictedPitch;
double predictedRollStd;
double predictedPitchStd;

double kalmanRoll;
double kalmanPitch;
double kalmanRollStd;
double kalmanPitchStd;

double dt = 0.010; // seconds

void setup()
{
    Serial.begin(9600);

    Wire.begin();
    imu.writeByte(0x6B, 0x00);
    imu.writeByte(0x1A, 0x03);
    imu.writeByte(0x38, 0x00);
    imu.writeByte(0x1B, 0x70);
    imu.writeByte(0x1C, 0x70);
    
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
    newTime = millis();
    calibrateImu();
    
}

void readImu(void){

  imu.getMotion6Counts(&ax, &ay, &az, &gx, &gy, &gz);
  accelX = (double)ax/ACCELEROMETER_SENSITIVITY+ax_offset;
  accelY = (double)ay/ACCELEROMETER_SENSITIVITY+ay_offset;
  accelZ = (double)az/ACCELEROMETER_SENSITIVITY+az_offset;
  gyroX = (double)gx/GYROSCOPE_SENSITIVITY+gx_offset;
  gyroY = (double)gy/GYROSCOPE_SENSITIVITY+gy_offset;
  gyroZ = (double)gz/GYROSCOPE_SENSITIVITY+gz_offset;

  aRoll = atan2(-accelX, accelZ)*(180/M_PI);
  aPitch = atan2(accelY, (sqrt(sq(accelX)+sq(accelZ))))*(180/M_PI);

}

void calibrateImu(){

  digitalWrite(13, HIGH);

  // get accelerometer offsets
  int sampleSize = 1000;

  double axNew;
  double ayNew;
  double azNew;
  double gxNew;
  double gyNew;
  double gzNew;
  double aRollNew;
  double aPitchNew;

  double ax_mean = 0;
  double ay_mean = 0;
  double az_mean = 0;
  double gx_mean = 0;
  double gy_mean = 0;
  double gz_mean = 0;
  double a_roll_mean;
  double a_pitch_mean;

  double prev_ax_mean = 0;
  double prev_ay_mean = 0;
  double prev_az_mean = 0;
  double prev_gx_mean = 0;
  double prev_gy_mean = 0;
  double prev_gz_mean = 0;
  double prev_a_roll_mean = 0;
  double prev_a_pitch_mean = 0;

  double stdRunningAX = 0;
  double stdRunningAY = 0;
  double stdRunningAZ = 0;
  double stdRunningGX = 0;
  double stdRunningGY = 0;
  double stdRunningGZ = 0;
  double stdRunningRoll = 0;
  double stdRunningPitch = 0;
  
  float count;

  for (int sample = 0; sample < sampleSize; sample++){
    readImu();
    axNew = (double)ax/ACCELEROMETER_SENSITIVITY;
    ayNew = (double)ay/ACCELEROMETER_SENSITIVITY;
    azNew = (double)az/ACCELEROMETER_SENSITIVITY;
    gxNew = (double)gx/GYROSCOPE_SENSITIVITY;
    gyNew = (double)gy/GYROSCOPE_SENSITIVITY;
    gzNew = (double)gz/GYROSCOPE_SENSITIVITY;

    aRollNew = atan2(-axNew, azNew);
    aPitchNew = atan2(ayNew, (sqrt(sq(axNew)+sq(azNew))));

    count = (double)(sample+1);
    delayMicroseconds(100);

    prev_ax_mean = ax_mean;
    prev_ay_mean = ay_mean;
    prev_az_mean = az_mean;
    prev_gx_mean = gx_mean;
    prev_gy_mean = gy_mean;
    prev_gz_mean = gz_mean;
    prev_a_roll_mean = a_roll_mean;
    prev_a_pitch_mean = a_pitch_mean;

    ax_mean = (1.0/count)*(axNew)+((count-1)/count)*ax_mean;
    ay_mean = (1.0/count)*(ayNew)+((count-1)/count)*ay_mean;
    az_mean = (1.0/count)*(azNew)+((count-1)/count)*az_mean;
    gx_mean = (1.0/count)*(gxNew)+((count-1)/count)*gx_mean;
    gy_mean = (1.0/count)*(gyNew)+((count-1)/count)*gy_mean;
    gz_mean = (1.0/count)*(gzNew)+((count-1)/count)*gz_mean;
    a_roll_mean = (1.0/count)*(aRollNew)+((count-1)/count)*a_roll_mean;
    a_pitch_mean = (1.0/count)*(aPitchNew)+((count-1)/count)*a_pitch_mean;

    stdRunningAX = stdRunningAX + (axNew - ax_mean) * (axNew - prev_ax_mean);
    stdRunningAY = stdRunningAY + (ayNew - ay_mean) * (ayNew - prev_ay_mean);
    stdRunningAZ = stdRunningAZ + (azNew - az_mean) * (azNew - prev_az_mean);
    stdRunningGX = stdRunningGX + (gxNew - gx_mean) * (gxNew - prev_gx_mean);
    stdRunningGY = stdRunningGY + (gyNew - gy_mean) * (gyNew - prev_gy_mean);
    stdRunningGZ = stdRunningGZ + (gzNew - gz_mean) * (gzNew - prev_gz_mean);
    stdRunningRoll = stdRunningRoll + (aRollNew - a_roll_mean) * (aRollNew - prev_a_roll_mean);
    stdRunningPitch = stdRunningPitch + (aPitchNew - a_pitch_mean) * (aPitchNew - prev_a_pitch_mean);
  }

  // calculate standard deviations
  axStd = stdRunningAX/(double)sampleSize;
  ayStd = stdRunningAY/(double)sampleSize;
  azStd = stdRunningAZ/(double)sampleSize;
  gxStd = stdRunningGX/(double)sampleSize;
  gyStd = stdRunningGY/(double)sampleSize;
  gzStd = stdRunningGZ/(double)sampleSize;
  rollStd = stdRunningRoll/(double)sampleSize;
  pitchStd = stdRunningPitch/(double)sampleSize;

  // calculate offsets
  ax_offset = 0 - ax_mean;
  ay_offset = 0 - ay_mean;
  az_offset = 1 - az_mean;
  gx_offset = 0 - gx_mean;
  gy_offset = 0 - gy_mean;
  gz_offset = 0 - gz_mean;
  
  digitalWrite(13, LOW);
  delay(5000);

}

void loop()
{  

  if  ((millis()-newTime) > 10){
    newTime = millis();
    readImu();
    kalmanPredict();
    kalmanUpdate();

    Serial.print("Roll: ");
    Serial.print(kalmanRoll);
    Serial.print("    Pitch: ");
    Serial.print(kalmanPitch);
    Serial.println();
    
  }
}






void kalmanPredict(){

  // our previous state is equal to our last kalman update
  double previousRoll = kalmanRoll;
  double previousPitch = kalmanPitch;
  double previousRollStd = kalmanRollStd;
  double previousPitchStd = kalmanRollStd;

  // calculate the predicted state values
  predictedRoll = previousRoll + (dt*gyroY);
  predictedPitch = previousPitch + (dt*gyroX);

  // calculate the co-variances in the predictions
  predictedRollStd = previousRollStd + gyStd;
  predictedPitchStd = previousPitchStd + gxStd;

}

void kalmanUpdate(){

  // compute kalman gains
  double kalmanGainRoll = predictedRollStd*(1/(predictedRollStd+rollStd));
  double kalmanGainPitch = predictedPitchStd*(1/(predictedPitchStd+pitchStd));

  // compute the optimal estimate
  kalmanRoll = predictedRoll + kalmanGainRoll*(aRoll-predictedRoll);
  kalmanPitch = predictedPitch + kalmanGainPitch*(aPitch-predictedPitch);

  // compute variances of optimal estimates
  kalmanRollStd = predictedRollStd-kalmanGainRoll*predictedRollStd;
  kalmanPitchStd = predictedPitchStd-kalmanGainPitch*predictedPitchStd;
  
}

// helper function
void printDouble( double val, unsigned int precision){
// prints val with number of decimal places determine by precision
// NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
// example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

   Serial.print (int(val));  //prints the int part
   Serial.print("."); // print the decimal point
   unsigned int frac;
   if(val >= 0)
       frac = (val - int(val)) * precision;
   else
       frac = (int(val)- val ) * precision;
   Serial.println(frac,DEC) ;
} 

