#include <matrix.h>
#include <Wire.h>
#include <QMC5883LCompass.h>

// double RateRoll, RatePitch, RateYaw;
double RateP, RateQ, RateR;
double AccX, AccY, AccZ;
double AngleRoll, AnglePitch;
QMC5883LCompass compass;
double xo, yo, zo;
Vector bias_acc;
Vector bias_w;
Vector acc_meas;
Vector w_meas;
// mekf variable delcaration
Matrix attitude(3, 3);
Matrix att_triad(3, 3);
// state prediction
Vector w_est(0.01, 0.01, 0.01);
Quaternion del_w;
double w_norm;
Quaternion q_st_pred;
Matrix F_n(6, 6);
Matrix P_st_pred(6, 6);
Matrix Q_n(6, 6);
Matrix Q_nw(3, 3); // define it
Matrix Q_nv(3, 3); // define it
Matrix prod(4, 4);
Quaternion q_temp(0, 0, 0, 0);
// measurement prediction
Vector v;
Vector qv_mean(0.0, 0.0, 0.001);
Vector w_meas_est;
Matrix z_est(6, 1);
Matrix z(6, 1);
Matrix H_n(6, 6);
Matrix R_nv(3, 3);
Matrix R_nw(3, 3);
Matrix R(6, 6);
Matrix S(6, 6);
// Kalman gain
Matrix K_n(6, 6);
Matrix x_est(6, 1);
Matrix P_meas_pred(6, 6);
// update q
Vector e;
Quaternion q_est;
Quaternion del_q;
Matrix T(3, 3);
Matrix T_n(6, 6);
Matrix P_update(6, 6);
unsigned long curr_time;
unsigned long pre_time = 0;
double dt;

String message;

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
void magnet_cal(void)
{
  int sx = 0;
  int sy = 0;
  int sz = 0;

  for (int i = 1; i <= 10; i++)
  {
    compass.read();
    sx += compass.getX();
    sy += compass.getY();
    sz += compass.getZ();
  }
  yo = (double)sx / 10.0;
  xo = (-1.0) * (double)sy / 10.0;
  zo = (double)sz / 10.0;
}

void triad(void)
{

  int x, y, z;
  // get magnetometer initial values
  magnet_cal();
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
  triad();
  // attitude = att_triad;
  attitude = q_est.attitude_matrix();
  // initialize
  pre_time = 0;
  // Once characterized, we can keep sigma_x, sigma_y, sigma_z

  {
    int num = 1000;
    Vector Total_acc(0, 0, 0);
    Vector Total_w(0, 0, 0);
    while (num > 0)
    {

      imuAcceleration();
      imuOmega();
      Total_acc = Total_acc + Vector(AccX, AccY, AccZ - 1.0);
      Total_w = Total_w + Vector(RateP, RateQ, RateR);
      num--;
    }
    bias_acc = Total_acc / 1000;
    bias_w = Total_w / 1000;
  }
  // initilize Q_nw, Q_nv, qv_mean, P_update, w_est , R_nv, R_nw
  Q_nw = 0.1 * identity(3);
  Q_nv = 0.001 * identity(3);
  R_nw = 0.001 * identity(3);
  R_nv = 0.001 * identity(3);
  P_update = identity(6);
  for (int i = 0; i < 6; i++)
    x_est.matrix[i][0] = 0.0;
  q_est.setq1(0.0);
  q_est.setq2(0.0);
  q_est.setq3(0.0);
  q_est.setq4(1.0);
}

void predict_state()
{
  w_norm = w_est.modulus();
  // Serial.println(w_norm);
  del_w.setq4(cos(w_norm * dt / 2));
  del_w.setq1(w_est.getX() / w_norm * sin(w_norm * dt / 2));
  del_w.setq2(w_est.getY() / w_norm * sin(w_norm * dt / 2));
  del_w.setq3(w_est.getZ() / w_norm * sin(w_norm * dt / 2));
  // del_w.display();
  // Quaternion q_temp(0, 0, 0, 0);
  // Matrix prod(4, 4);
  for (int i = 0; i < 4; i++)
  {
    for (int k = 0; k < 3; k++)
      prod.matrix[i][k] = q_est.epsillon().matrix[i][k];
  }
  prod.matrix[0][3] = q_est.getq1();
  prod.matrix[1][3] = q_est.getq2();
  prod.matrix[2][3] = q_est.getq3();
  prod.matrix[3][3] = q_est.getq4();
  // prod.display();
  // (prod * del_w).display();
  // .
  
  // q_temp = q_est;
  q_st_pred = q_temp + prod * del_w;
  q_st_pred.normalize();
  // q_st_pred.display();
  F_n = identity(6);
  for (int i = 0; i < 3; i++)
  {
    for (int k = 0; k < 3; k++)
      F_n.matrix[i][k] = del_w.attitude_matrix().matrix[i][k];
    F_n.matrix[i][i + 3] = dt;
  }
  for (int i = 0; i < 3; i++)
  {
    for (int k = 0; k < 3; k++)
    {
      Q_n.matrix[i][k] = Q_nw.matrix[i][k] * dt * dt * dt / 3;
      Q_n.matrix[i][k + 3] = -Q_nw.matrix[i][k] * dt * dt / 2;
      Q_n.matrix[i + 3][k] = -Q_nw.matrix[i][k] * dt * dt / 2;
      Q_n.matrix[i + 3][k + 3] = Q_nw.matrix[i][k] * dt;
    }
  }
  P_st_pred = F_n * (P_update + Q_n) * F_n.transpose();
}

void measurement_update()
{
  v = q_st_pred.attitude_matrix() * (Vector(0, 0, -1) + qv_mean);
  w_meas_est = w_est;
  for (int i = 0; i < 3; i++)
  {
    z_est.matrix[i][0] = v.matrix[i][0];
    z_est.matrix[i + 3][0] = w_meas_est.matrix[i][0];
  }
  H_n = identity(6);
  for (int i = 0; i < 3; i++)
  {
    for (int k = 0; k < 3; k++)
      H_n.matrix[i][k] = v.skew_from_vec().matrix[i][k];
  }
  for (int i = 0; i < 3; i++)
  {
    for (int k = 0; k < 3; k++)
    {
      R.matrix[i][k] = (q_st_pred.attitude_matrix() * Q_nv * q_st_pred.attitude_matrix().transpose() + R_nv).matrix[i][k];
      R.matrix[i + 3][k + 3] = R_nw.matrix[i][k];
    }
  }
  S = H_n * P_st_pred * H_n.transpose() + R;
  // Kalman gain
  K_n = P_st_pred * H_n.transpose() * S.inverse();
  for (int i = 0; i < 3; i++)
  {
    z.matrix[i][0] = acc_meas.matrix[i][0];
    z.matrix[i + 3][0] = w_meas.matrix[i][0];
  }
  x_est = x_est + K_n * (z - z_est);
  Matrix I = identity(6);
  P_meas_pred = (I - K_n * H_n) * P_st_pred;

  e.setX(x_est.matrix[0][0]);
  e.setY(x_est.matrix[1][0]);
  e.setZ(x_est.matrix[2][0]);
  // RV chart
  del_q.setq1(e.getX() * sin(e.modulus() / 2));
  del_q.setq2(e.getY() * sin(e.modulus() / 2));
  del_q.setq3(e.getZ() * sin(e.modulus() / 2));
  del_q.setq4(cos(e.modulus() / 2));
  //
  for (int i = 0; i < 4; i++)
  {
    for (int k = 0; k < 3; k++)
      prod.matrix[i][k] = q_st_pred.epsillon().matrix[i][k];
  }
  prod.matrix[0][3] = q_st_pred.getq1();
  prod.matrix[1][3] = q_st_pred.getq2();
  prod.matrix[2][3] = q_st_pred.getq3();
  prod.matrix[3][3] = q_st_pred.getq4();
  // q_temp = q_st_pred;
  q_est = q_temp + prod * del_q;
  q_est.normalize();
  // update covariance matrix;
  Matrix d_vec(3, 1);
  d_vec.matrix[0][0] = del_q.getq1();
  d_vec.matrix[1][0] = del_q.getq2();
  d_vec.matrix[2][0] = del_q.getq3();
  double d_norm = sqrt(d_vec.matrix[0][0] * d_vec.matrix[0][0] + d_vec.matrix[1][0] * d_vec.matrix[1][0] + d_vec.matrix[2][0] * d_vec.matrix[2][0]);
  Matrix ddt(3, 3);
  ddt = d_vec * d_vec.transpose();
  Matrix I3 = identity(3);
  Vector d_vec1;
  d_vec1.setX(d_vec.matrix[0][0]);
  d_vec1.setY(d_vec.matrix[1][0]);
  d_vec1.setZ(d_vec.matrix[2][0]);
  T = (d_norm / asin(d_norm)) * (del_q.getq4() * (I3 - ddt) - d_vec1.skew_from_vec()) + ddt;
  T_n = identity(6);
  for (int i = 0; i < 3; i++)
  {
    for (int k = 0; k < 3; k++)
      T_n.matrix[i][k] = T.matrix[i][k];
  }
  P_update = T_n * P_meas_pred * T_n.transpose();
  // reset e
  x_est.matrix[0][0] = 0;
  x_est.matrix[1][0] = 0;
  x_est.matrix[2][0] = 0;
  w_est.setX(x_est.matrix[3][0]);
  w_est.setY(x_est.matrix[4][0]);
  w_est.setZ(x_est.matrix[5][0]);
}

void loop()
{
  imuAcceleration();
  imuOmega();
  dt = 0.1;
  acc_meas.setX(AccX - bias_acc.getX());
  acc_meas.setY(AccY - bias_acc.getY());
  acc_meas.setZ(AccZ - bias_acc.getZ());
  w_meas.setX(RateP - bias_w.getX());
  w_meas.setY(RateQ - bias_w.getY());
  w_meas.setX(RateR - bias_w.getZ());

  predict_state();

  measurement_update();
  // update quaternion

  // q_est.display();
  message = String(q_est.getq4()) + ',' + String(q_est.getq1()) + ',' + String(q_est.getq2()) + ',' + String(q_est.getq3());
  Serial.println(message);
  delay(100);
}