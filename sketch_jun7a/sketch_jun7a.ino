// // #include <A2DPSink.h>
// // #include <A2DPSource.h>
// // #include <BluetoothAudio.h>
// // #include <BluetoothAudioConsumer.h>
// // #include <BluetoothAudioConsumerI2S.h>
// // #include <BluetoothAudioConsumerPWM.h>
// // #include <BluetoothMediaConfigurationSBC.h>

// // #include <Mouse.h>

// // #include <QMC5883LCompass.h>


// // const byte qmc5883l_mode_stby = 0x00;
// // const byte qmc5883l_mode_cont = 0x01;

// // const byte qmc5883l_odr_10hz  = 0x00;
// // const byte qmc5883l_odr_50hz  = 0x04;
// // const byte qmc5883l_odr_100hz = 0x08;
// // const byte qmc5883l_odr_200hz = 0x0C;

// // const byte qmc5883l_rng_2g    = 0x00;
// // const byte qmc5883l_rng_8g    = 0x10;

// // const byte qmc5883l_osr_512   = 0x00;
// // const byte qmc5883l_osr_256   = 0x40;
// // const byte qmc5883l_osr_128   = 0x80;
// // const byte qmc5883l_osr_64    = 0xC0;

// // QMC5883LCompass compass;

// // void setup(){
// //    Serial.begin(9600);
// //    compass.init();
// // }

// // void loop(){
// //    float x_value;
// //    float y_value;
// //    float z_value;
// //    compass.read(); // Read compass values via I2C

// //    x_value   = compass.getX();
// //    y_value   = compass.getY();
// //    z_value   = compass.getZ();
   
// //    float bi1,bi2,bi3;
// //    bi1 =631.706326;
// //    bi2 =1044.187608;
// //    bi3 =-990.048842;
// //    float x_ab = x_value-bi1;
// //    float y_ab = y_value-bi2;
// //    float z_ab = z_value-bi3;

// //    float a1=0.297962, 
// //    a2=-0.006435, 
// //    a3=0.005633, 
// //    b1=-0.006435, 
// //    b2=0.309702, 
// //    b3=-0.004306, 
// //    c1=0.005633, 
// //    c2=-0.004306,
// //    c3=0.302040;

// //    long x_fin,y_fin,z_fin;
// //    x_fin = a1*x_ab+a2*y_ab+a3*z_ab;
// //    y_fin = b1*x_ab+b2*y_ab+b3*z_ab;
// //    z_fin = c1*x_ab+c2*y_ab+c3*z_ab;
// //    //Serial.println(x_fin*x_fin);
// //    //Serial.println(y_fin*y_fin);
// //    //Serial.println(z_fin*z_fin);
// //    double norm = sqrt(x_fin*x_fin + y_fin*y_fin + z_fin*z_fin);

// //    Serial.print(x_fin); Serial.print(" ");
// //    Serial.print(y_fin); Serial.print(" ");
// //    Serial.print(z_fin); Serial.print(" ");
// //    //Serial.print(norm); Serial.print(" ");
// //    //float angle = atan2(y_fin,x_fin)*180/PI;
// //    //Serial.println(angle);

// //    delay(10);
// // }

// // #include <matrix3.h>
// // #include <quaternion.h>
// // #include <rotations.h>
// // #include <vector2.h>
// // #include <vector3.h>

// #include <Wire.h>
// #include <QMC5883LCompass.h>

// const byte qmc5883l_mode_stby = 0x00;
// const byte qmc5883l_mode_cont = 0x01;

// const byte qmc5883l_odr_10hz  = 0x00;
// const byte qmc5883l_odr_50hz  = 0x04;
// const byte qmc5883l_odr_100hz = 0x08;
// const byte qmc5883l_odr_200hz = 0x0C;

// const byte qmc5883l_rng_2g    = 0x00;
// const byte qmc5883l_rng_8g    = 0x10;

// const byte qmc5883l_osr_512   = 0x00;
// const byte qmc5883l_osr_256   = 0x40;
// const byte qmc5883l_osr_128   = 0x80;
// const byte qmc5883l_osr_64    = 0xC0;

// float RateRoll, RatePitch, RateYaw;
// float AccX, AccY, AccZ;
// float x_value, y_value, z_value;
// float AngleRoll, AnglePitch;
// QMC5883LCompass compass;
// float x0, yo, z0;


// String message;
// String sendMessage;

// void gyro_signals(void) {
//   Wire.beginTransmission(0x68);
//   Wire.write(0x1A);
//   Wire.write(0x05);
//   Wire.endTransmission();
//   Wire.beginTransmission(0x68);
//   Wire.write(0x1C);
//   Wire.write(0x10);
//   Wire.endTransmission();
//   Wire.beginTransmission(0x68);
//   Wire.write(0x3B);
//   Wire.endTransmission(); 
//   Wire.requestFrom(0x68,6);
//   int16_t AccXLSB = Wire.read() << 8 | Wire.read();
//   int16_t AccYLSB = Wire.read() << 8 | Wire.read();
//   int16_t AccZLSB = Wire.read() << 8 | Wire.read();
//   Wire.beginTransmission(0x68);
//   Wire.write(0x1B); 
//   Wire.write(0x8);
//   Wire.endTransmission();                                                   
//   Wire.beginTransmission(0x68);
//   Wire.write(0x43);
//   Wire.endTransmission();
//   Wire.requestFrom(0x68,6);
//   int16_t GyroX=Wire.read()<<8 | Wire.read();
//   int16_t GyroY=Wire.read()<<8 | Wire.read();
//   int16_t GyroZ=Wire.read()<<8 | Wire.read();
//   RateRoll=(float)GyroX/65.5;
//   RatePitch=(float)GyroY/65.5;
//   RateYaw=(float)GyroZ/65.5;
//   AccY=(float)AccXLSB/4096;
//   AccX=(float)AccYLSB/4096;
//   AccZ=-(float)AccZLSB/4096;
//   AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
//   AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
// }

// void magnet_cal(void){
//   int sx = 0;
//   int sy = 0;
//   int sz = 0;

//   for(int i= 1; i<=10; i++)
//   {
//    compass.read();
//     x_value   = compass.getX();
//    y_value   = compass.getY();
//    z_value   = compass.getZ();
   
//    float bi1,bi2,bi3;
//    bi1 =631.706326;
//    bi2 =1044.187608;
//    bi3 =-990.048842;
//    float x_ab = x_value-bi1;
//    float y_ab = y_value-bi2;
//    float z_ab = z_value-bi3;

//    float a1=0.297962, 
//    a2=-0.006435, 
//    a3=0.005633, 
//    bb1=-0.006435, 
//    bb2=0.309702, 
//    bb3=-0.004306, 
//    c1=0.005633, 
//    c2=-0.004306,
//    c3=0.302040;

//    long x_fin,y_fin,z_fin;
//    x_fin = a1*x_ab+a2*y_ab+a3*z_ab;
//    y_fin = bb1*x_ab+bb2*y_ab+bb3*z_ab;
//    z_fin = c1*x_ab+c2*y_ab+c3*z_ab;
//    sx += x_fin;
//    sy += y_fin;
//    sz += z_fin;
//   }
//   x0 = (float)sx/10.0;
//   yo = (-1.0)*(float)sy/10.0;
//   z0 = -1.0*(float)sz/10.0;
// }
// // void rotationMatrixToQuaternion(const Matrix3f& M, double& q1, double& q2, double& q3, double& q4) {
// //     double trace = M.a.x + M.b.y + M.c.z;

// //     if (trace > 0) {
// //         double S = sqrt(trace + 1.0) * 2; // S=4*q1
// //         q1 = 0.25 * S;
// //         q2 = (M.c.y - M.b.z) / S;
// //         q3 = (M.a.z - M.c.x) / S;
// //         q4 = (M.b.x - M.a.y) / S;
// //     } else if ((M.a.x > M.b.y) && (M.a.x > M.c.z)) {
// //         double S = sqrt(1.0 + M.a.x - M.b.y - M.c.z) * 2; // S=4*q2
// //         q1 = (M.c.y - M.b.z) / S;
// //         q2 = 0.25 * S;
// //         q3 = (M.a.y + M.b.x) / S;
// //         q4 = (M.a.z + M.c.x) / S;
// //     } else if (M.b.y > M.c.z) {
// //         double S = sqrt(1.0 + M.b.y - M.a.x - M.c.z) * 2; // S=4*q3
// //         q1 = (M.a.z - M.c.x) / S;
// //         q2 = (M.a.y + M.b.x) / S;
// //         q3 = 0.25 * S;
// //         q4 = (M.b.z + M.c.y) / S;
// //     } else {
// //         double S = sqrt(1.0 + M.c.z - M.a.x - M.b.y) * 2; // S=4*q4
// //         q1 = (M.b.x - M.a.y) / S;
// //         q2 = (M.a.z + M.c.x) / S;
// //         q3 = (M.b.z + M.c.y) / S;
// //         q4 = 0.25 * S;
// //     }
// // }

// int i;
// bool increasing;

// void setup() {
//   Serial.begin(57600);
//   compass.init();
//   compass.setSmoothing(5,true);
//   pinMode(13, OUTPUT);
//   digitalWrite(13, HIGH);
//   Wire.setClock(400000);
//   Wire.begin();
//   delay(250);
//   Wire.beginTransmission(0x68); 
//   Wire.write(0x6B);
//   Wire.write(0x00);
//   Wire.endTransmission();
//   i = 0;
//   increasing = true;
// }

// void loop() {
//   long x, y, z;
//   //get magnetometer initial values
//   //magnet_cal();
//   // Read compass values
//   // compass.read();

//   // // Return XYZ readings
  
//   //  x_value   = compass.getX();
//   //  y_value   = compass.getY();
//   //  z_value   = compass.getZ();
//   //  Serial.println("x0");
//   //  Serial.print("x_value");
//   //  Serial.println("y_value");
//   //  Serial.println("z_value");

//   if(i == 10){
//     increasing = false;
//   }
//   else if(i == -10){
//     increasing = true;
//   }
//   if(increasing){
//     i++;
//   }
//   else{
//     i--;
//   }
//   Serial.print(i);
//   Serial.print(", ");
//   Serial.print(i + 1);
//   Serial.print(", ");
//   Serial.println(i);
// //    float bi1,bi2,bi3;
// //    bi1 =631.706326;
// //    bi2 =1044.187608;
// //    bi3 =-990.048842;
// //    float x_ab = x_value-bi1;
// //    float y_ab = y_value-bi2;
// //    float z_ab = z_value-bi3;

// //    float a1=0.297962, 
// //    a2=-0.006435, 
// //    a3=0.005633, 
// //    bb1=-0.006435, 
// //    bb2=0.309702, 
// //    bb3=-0.004306, 
// //    c1=0.005633, 
// //    c2=-0.004306,
// //    c3=0.302040;

// //    long x_fin,y_fin,z_fin;
// //    x_fin = a1*x_ab+a2*y_ab+a3*z_ab;
// //    y_fin = bb1*x_ab+bb2*y_ab+bb3*z_ab;
// //    z_fin = c1*x_ab+c2*y_ab+c3*z_ab;
// //    x = x_fin;
// //    y = -y_fin;
// //    z = -z_fin;

// //   //read IMU
// //   gyro_signals();
// //  // Serial.println("mag");
// //   //Serial.print(x);
// //   //Serial.print(" ");
// //   //Serial.print(y);
// //   //Serial.print(" ");
// //   //Serial.println(z);
// //   //Serial.println();
// //   Vector3f r1(0.0f, 0.0f, 1.0f);
// //   Serial.print("x0: ");
// //   Serial.print(x0);
// //   Serial.print(" y0: ");
// //   Serial.print(yo);
// //   Serial.print(" z0: ");
// //   Serial.print(z0);
// //   x0 = x0/1000.0;
// //   yo = yo/1000.0;
// //   z0 = z0/1000.0;
// //   Vector3f r2(x0 , yo, z0);
  
// //   r1.normalize();
// //   r2.normalize();
 
// //   Vector3f b1(-AccX, -AccY, -AccZ);
// //   x = x/1000;
// //   y = y/1000;
// //   z = z/1000;
// //   Vector3f b2(x, y, z);
// //   b1.normalize();
// //   b2.normalize();
// //    //Serial.print(b1.x);
// //   //Serial.print(',');
// //   //  Serial.print(b1.y);
// //   // Serial.print(',');
// //   //  Serial.println(b1.z);

// //   //triad
// //   Vector3f v1(r1.x, r1.y, r1.z);
// //   Vector3f v2 = r1 % r2;
// //   v2.normalize();
// //   Vector3f v3 = v1 % v2;
// //   Vector3f w1(b1.x, b1.y, b1.z);
// //   Vector3f w2 = b1 % b2;
// //   w2.normalize();
// //   Vector3f w3 = w1 % w2;
// //   Matrix3f attitude;
// //   Matrix3f v(v1, v2, v3);
// //   v.transpose();
// //   Matrix3f w(w1, w2, w3);
// //   w.transpose();
// //   attitude = w*v.transposed();
// //   float theta = -asin(attitude.a.z)*180.0/3.14;
// //   //Matrix3f det = attitude*attitude.transposed();

// //   //float tr = attitude.a.x + attitude.b.y + attitude.c.z ;
// //   // float theta = acos((tr - 1)/2);
// //   //message = String(x)+','+String(y)+','+String(z)+','+String(AngleRoll);
// //   //Serial.println(theta);

// //   double q1, q2, q3, q4;
// //   rotationMatrixToQuaternion(attitude, q1, q2, q3, q4);

// //   Serial.print(q1);
// //   Serial.print(',');Serial.println(x_value);
// //   Serial.print(q2);
// //   Serial.print(",");
// //   Serial.print(q3);
// //   Serial.print(',');
// //   Serial.println(q4);
// //   Serial.println();
// //   delay(50);
// }

void setup() {
  // Start serial communication at 9600 baud rate
  Serial.begin(9600);
}

void loop() {
  // Check if data is available to read
  if (Serial.available() > 0) {
    // Read the incoming byte
    char received = Serial.read();
    
    // Send the received byte back
    Serial.println(received);
  }
  else {
    Serial.println("1,2,3");
  }
  // int16_t data = 123;  // Define the data as int16
  
  // // Send the data as raw binary (2 bytes)
  // Serial.println("1,2,3");

  // if (Serial.available()) {
  //   String input = Serial.readStringUntil('\n'); // Read the incoming string until newline character
  //   float values[3];
  //   int i = 0;

  //   // Parse the input string and split at commas
  //   int commaIndex;
  //   while ((commaIndex = input.indexOf(',')) != -1 && i < 3) {
  //     values[i] = input.substring(0, commaIndex).toFloat();
  //     values[i] *= 2; // Double the value
  //     input = input.substring(commaIndex + 1);
  //     i++;
  //   }
  //   values[i] = input.toFloat() * 2; // Last value after the last comma

  //   // Output doubled values as comma-separated string
  //   Serial.print(values[0]);
  //   Serial.print(",");
  //   Serial.print(values[1]);
  //   Serial.print(",");
  //   Serial.print(values[2]);
  // }

  // Delay to control the rate of data transmission
  delay(5);
}