#include <matrix.h>

// #include <matrix3.h>
// #include <quaternion.h>
// #include <rotations.h>
// #include <vector2.h>
// #include <vector3.h>
// #include <matrix.cpp>
#include <Wire.h>
#include <QMC5883LCompass.h>

double RateP, RateQ, RateR;
double AccX, AccY, AccZ;
double AngleRoll, AnglePitch;
QMC5883LCompass compass;
double xo, yo, zo;
double x_fin, y_fin, z_fin;

// mekf variable delcaration
Matrix att_triad(3, 3);
Quaternion qo, q_prop;
Vector b_w(0.0, 0.0, 0.0);
Vector b_f(0.0, 0.0, 0.0);
Vector b_m(0.0, 0.0, 0.0);

double temp[15][15];
// in gain
Matrix H(6, 12);
Matrix H1(3, 3);
Matrix H2(3, 3);
Matrix K_gain(12, 6);
Matrix P_pre = 0.10 * identity(12);
Matrix R = 0.02 * identity(6);
Matrix attitude(3, 3);
Vector accel_g(0, 0, -1.0);
Vector mag_fixed(1.0, 0, 0);
// in update
Matrix P_update(12, 12);
Matrix delta_x(12, 1);
Quaternion q_est;

// in propagation
Vector w_est;
Vector w_measured;
Matrix phi(12, 12);
Matrix Fk(12, 12);
Matrix Q = zeros(12, 12);
unsigned long curr_time;
unsigned long pre_time = 0;
double dt;
// double sigma_bw , sigma_bf;
Vector sigma_w(0.1, 0.1, 0.3);
double sigma_bw = 0.1;
double sigma_bf = 0.001;
double sigma_bm = 0.2;
String message;

void mag_calibration()
{
  float x_value;
  float y_value;
  float z_value;
  compass.read(); // Read compass values via I2C

  x_value = compass.getX();
  y_value = compass.getY();
  z_value = compass.getZ();

  double bi1, bi2, bi3;
  bi1 = -158.227540;
  bi2 = -485.725478;
  bi3 = 217.206439;

  double x_ab = x_value - bi1;
  double y_ab = y_value - bi2;
  double z_ab = z_value - bi3;

  double a1 = 0.296698,
         a2 = 0.007968,
         a3 = -0.004701,
         bb1 = 0.007968,
         bb2 = 0.369302,
         bb3 = -0.010403,
         c1 = -0.004701,
         c2 = -0.010403,
         c3 = 0.365830;

  x_fin = a1 * x_ab + a2 * y_ab + a3 * z_ab;
  y_fin = bb1 * x_ab + bb2 * y_ab + bb3 * z_ab;
  z_fin = c1 * x_ab + c2 * y_ab + c3 * z_ab;
}
// Obtaining Angular Velocity in Body Frame from IMU for propagation
void imuOmega()
{
  // Updates the global variables RateP, RateQ, RateR with latest angular velocity measurements
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateP = (double)GyroY / 65.5 * (3.14 / 180);
  RateQ = (double)GyroX / 65.5 * (3.14 / 180);
  RateR = -(double)GyroZ / 65.5 * (3.14 / 180);
}

// Obtaining acceleration in body frame from IMU
void imuAcceleration()
{
  // Updates the global variables AccX, AccY, AccZ with latest acceleration measurements
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  AccX = (double)AccYLSB / 4096;
  AccY = (double)AccXLSB / 4096;
  AccZ = -(double)AccZLSB / 4096;
}

void triad(void)
{

  int x, y, z;

  // Read compass values
  compass.read();

  // Return XYZ readings
  y = (double)compass.getX();
  x = -1.0 * (double)(compass.getY());
  z = (double)compass.getZ();

  // read IMU
  imuAcceleration();
  imuOmega();

  Vector r1(0.0f, 0.0f, -1.0f);
  Vector r2(xo, yo, zo);
  r1.normalize();
  r2.normalize();
  Vector b1(AccX, AccY, AccZ);

  Vector b2(x, y, z);
  b1.normalize();
  b2.normalize();
  // triad
  Vector v1(r1.getX(), r1.getY(), r1.getZ());
  Vector v2 = r1 % r2;
  v2.normalize();
  Vector v3 = r1 % v2;
  Vector w1(b1.getX(), b1.getY(), b1.getZ());
  Vector w2 = b1 % b2;
  w2.normalize();
  Vector w3 = b1 % w2;
  Vector v1_v2_v3[] = {v1, v2, v3};
  Matrix v = matrix_from_vectors(v1_v2_v3, 3);
  Vector w1_w2_w3[] = {w1, w2, w3};
  Matrix w = matrix_from_vectors(w1_w2_w3, 3);
  att_triad = v * w.transpose();
}

void setup()
{
  Serial.begin(57600);
  compass.init();
  compass.setSmoothing(5, true);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  // get magnetometer initial values
  for (int i = 0; i < 10; i++)
  {
    mag_calibration();
    xo = xo + x_fin;
    yo = yo + y_fin;
    zo = zo + z_fin;
  }
  xo = xo / 10.0;
  yo = yo / 10.0;
  zo = zo / 10.0;
  triad();
  attitude = att_triad;
  // attitude = q_est.attitude_matrix();
  //  initialize
  mag_fixed.setX(1);
  mag_fixed.setY(0);
  mag_fixed.setZ(0);
  pre_time = 0;
  // Once characterized, we can keep sigma_x, sigma_y, sigma_z
  for (int i = 0; i < 3; i++)
  {
    for (int k = 0; k < 3; k++)
    {
      if (i == k)
        R.matrix[i][k] = 0.01;
      else
        R.matrix[i][k] = 0.001;
    }
    R.matrix[i + 3][i + 3] = 1.0;
  }
  {
    int num = 1000;
    Vector Total_acc(0, 0, 0);
    Vector Total_w(0, 0, 0);
    while (num > 0)
    {

      imuAcceleration();
      imuOmega();
      Total_acc = Total_acc + Vector(-9.8 * AccX, -9.8 * AccY, -9.8 * AccZ);
      Total_w = Total_w + Vector(RateP, RateQ, RateR);
      num--;
    }
    Total_acc = Total_acc / 1000;
    Total_w = Total_w / 1000;
    for (int i = 0; i < 3; i++)
    {
      b_w.matrix[i][0] = Total_w.matrix[i][0];
      // accel_g.matrix[i][0] = Total_acc.matrix[i][0];
    }
    b_f = Total_acc - Vector(0.0, 0.0, -9.8);
    // b_w.display();
    // delay(2000);
    //  b_f.display();
  }
}

void meas_update()
{
  curr_time = millis();
  // dt = (curr_time - pre_time) * 0.001;
  dt = 0.1;
  // gain calculation
  H1 = Vector(attitude * accel_g).skew_from_vec();
  H2 = Vector(attitude * mag_fixed).skew_from_vec();
  // Serial.println("H1");
  // H1.display();
  for (int i = 0; i < 6; i++)
  {
    for (int k = 0; k < 12; k++)
    {
      H.matrix[i][k] = 0.0;
    }
  }
  H.matrix[0][6] = 1.0;
  H.matrix[1][7] = 1.0;
  H.matrix[2][8] = 1.0;
  H.matrix[3][9] = 1.0;
  H.matrix[4][10] = 1.0;
  H.matrix[5][11] = 1.0;
  for (int i = 0; i < 3; i++)
  {
    for (int k = 0; k < 3; k++)
    {
      H.matrix[i][k] = H1.matrix[i][k];
      H.matrix[i + 3][k] = H2.matrix[i][k];
    }
  }

  // Serial.println("H");
  // H.display();
  K_gain = P_pre * H.transpose() * (H * P_pre * H.transpose() + R).inverse();
  // Serial.println("Kalman Gain: ");
  // K_gain.display();
  //  update
  Matrix I = identity(12);
  // P_update = (I - K_gain * H) * P_pre * (I - K_gain*H).transpose() + K_gain*R*K_gain.transpose();
  P_update = (I - K_gain * H) * P_pre;
  // Serial.println("K_gain*H");
  //(K_gain*H).display();
  // Serial.println("I-K*H");
  //(I - K_gain * H).display();
  // Serial.println("P_update");
  // P_update.display();
  Matrix temp1(6, 1);
  mag_fixed.normalize();
  Vector rotated_g = attitude * accel_g;
  Vector rotated_m = attitude * mag_fixed;
  // Serial.println("Attitude Matrix");
  // attitude.display();
  // Serial.println("accel_g");
  // accel_g.display();
  // Serial.println("Rotated g");
  // rotated_g.display();
  Vector accel_measured;
  Vector mag_measured;

  Vector acc_raw(AccX, AccY, AccZ);
  // double acc_norm = acc_raw.modulus();
  Vector(AccX, AccY, AccZ).display();
  double mag_norm = sqrt(x_fin * x_fin + y_fin * y_fin + z_fin * z_fin);
  b_f.display();
  accel_measured = acc_raw - b_f;
  accel_measured.normalize();
  Serial.println("Accel Measured");
  accel_measured.display();

  mag_measured = Vector(x_fin / mag_norm, y_fin / mag_norm, z_fin / mag_norm) - b_m;
  // Serial.println("accel_meas");
  // accel_measured.display();
  temp1.matrix[0][0] = accel_measured.getX() - rotated_g.getX();
  temp1.matrix[1][0] = accel_measured.getY() - rotated_g.getY();
  temp1.matrix[2][0] = accel_measured.getZ() - rotated_g.getZ();
  temp1.matrix[3][0] = mag_measured.getX() - rotated_m.getX();
  temp1.matrix[4][0] = mag_measured.getY() - rotated_m.getY();
  temp1.matrix[5][0] = mag_measured.getZ() - rotated_m.getZ();
  delta_x = delta_x + K_gain * temp1;
  // Serial.println("H1");
  // H1.display();
  // Serial.print("State Vector: ");
  //(1000*delta_x).display();

  b_w.setX(b_w.getX() + delta_x.matrix[3][0]);
  b_w.setY(b_w.getY() + delta_x.matrix[4][0]);
  b_w.setZ(b_w.getZ() + delta_x.matrix[5][0]);
  b_f.setX(b_f.getX() + delta_x.matrix[6][0]);
  b_f.setY(b_f.getY() + delta_x.matrix[7][0]);
  b_f.setZ(b_f.getZ() + delta_x.matrix[8][0]);
  b_m.setX(b_m.getX() + delta_x.matrix[9][0]);
  b_m.setY(b_m.getY() + delta_x.matrix[10][0]);
  b_m.setZ(b_m.getZ() + delta_x.matrix[11][0]);
}

void predict_state()
{
  w_measured.matrix[0][0] = RateP;
  w_measured.matrix[1][0] = RateQ;
  w_measured.matrix[2][0] = -RateR;
  // Serial.println("w_measured");
  // w_measured.display();
  w_est.setX(w_measured.getX() - b_w.getX());
  w_est.setY(w_measured.getY() - b_w.getY());
  w_est.setZ(w_measured.getZ() - b_w.getZ());
  // Serial.println("w_est");
  // w_est.display();
  Matrix w_est_cross = w_est.skew_from_vec();
  Matrix epsilon1 = q_est.epsillon();
  Matrix Fk = zeros(12, 12);
  Matrix I3 = identity(3);
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      Fk.matrix[i][j] = -w_est_cross.matrix[i][j];
    }
  }

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      Fk.matrix[i][3 + j] = -I3.matrix[i][j];
    }
  }
  // Serial.println("Fk");
  // Fk.display();
  q_prop = q_est + 0.5 * dt * epsilon1 * w_est;
  q_prop.normalize();
  // Serial.println("Propogated q");
  // q_prop.display();
  Matrix I12 = identity(12);
  phi = I12 + dt * Fk;

  // Serial.println("Phi, transition");
  // phi.display();
  // initialise Q
  for (int i = 0; i < 3; i++)
  {
    // Q.matrix[3 + i][j] = (-sigma_bw * sigma_bw * dt * dt / 2) * I3.matrix[i][j];
    // Q.matrix[i][3 + j] = (sigma_w * sigma_w * dt + sigma_bw * sigma_bw * dt * dt / 3) * I3.matrix[i][j];
    // Q.matrix[3 + i][j] = (-sigma_bw * sigma_bw * dt * dt / 2) * I3.matrix[i][j];
    // Q.matrix[3 + i][3 + j] = (sigma_bw * sigma_bw * dt) * I3.matrix[i][j];
    // Q.matrix[6 + i][6 + j] = (sigma_bf * sigma_bf * dt) * I3.matrix[i][j];
    // Q.matrix[i][i] = ((sigma_w * sigma_w * dt) + (sigma_bw * sigma_bw * dt * dt / 3));
    Q.matrix[3 + i][3 + i] = (sigma_bw * sigma_bw * dt);
    Q.matrix[6 + i][6 + i] = (sigma_bf * sigma_bf * dt);
    Q.matrix[i][i + 3] = (-sigma_bw * sigma_bw * dt * dt / 2);
    Q.matrix[3 + i][i] = (-sigma_bw * sigma_bw * dt * dt / 2);
    Q.matrix[9 + i][9 + i] = (sigma_bm * sigma_bm * dt);
  }
  Q.matrix[0][0] = ((sigma_w.getX() * sigma_w.getX() * dt) + (sigma_bw * sigma_bw * dt * dt / 3));
  Q.matrix[1][1] = ((sigma_w.getY() * sigma_w.getY() * dt) + (sigma_bw * sigma_bw * dt * dt / 3));
  Q.matrix[2][2] = ((sigma_w.getZ() * sigma_w.getZ() * dt) + (sigma_bw * sigma_bw * dt * dt / 3));

  // Serial.println("No error here");

  P_pre = phi * P_update * phi.transpose() + Q;
  // Serial.println("P_pre");
  // P_pre.display();
  pre_time = curr_time;
  attitude = q_prop.attitude_matrix();
}

void reset()
{
  // resets the delta_x vector to zero and the reference quaternion
  Vector a;
  a.setX(delta_x.matrix[0][0]);
  a.setY(delta_x.matrix[1][0]);
  a.setZ(delta_x.matrix[2][0]);
  b_w.setX(b_w.getX() + delta_x.matrix[3][0]);
  b_w.setY(b_w.getY() + delta_x.matrix[4][0]);
  b_w.setZ(b_w.getZ() + delta_x.matrix[5][0]);
  Matrix epsilon = q_prop.epsillon();
  q_est = q_prop + 0.5 * epsilon * a;
  q_est.normalize();
  double norm1 = sqrt(q_est.getq1() * q_est.getq1() + q_est.getq2() * q_est.getq2() + q_est.getq3() * q_est.getq3() + q_est.getq4() * q_est.getq4());
  q_est = q_est / norm1;
  for (int i = 0; i < 6; i++)
  {
    delta_x.matrix[i][0] = 0;
  }
}

void loop()
{
  imuAcceleration();
  imuOmega();
  mag_calibration();

  meas_update();

  predict_state();

  reset();
  // prediction

  // Serial.println("b_w");
  // b_w.display();

  // Serial.print("q_est, estimated");
  // q_est.display();
  //  Propagation

  // Serial.println("Estimated q");
  // q_est.display();
  delay(100);
  // Serial.println("dt");
  // Serial.println(dt);
  // message = String(q_est.getq4())+','+String(q_est.getq1())+','+String(q_est.getq2()) + ',' + String(q_est.getq3());
  Serial.print(q_est.getq4(), 5);
  Serial.print(',');
  Serial.print(q_est.getq1(), 5);
  Serial.print(',');
  Serial.print(q_est.getq2(), 5);
  Serial.print(',');
  Serial.println(q_est.getq3(), 5);
  // Serial.println(message);
}