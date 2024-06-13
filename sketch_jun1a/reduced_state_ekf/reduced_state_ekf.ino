#include <matrix.h>
#include <Wire.h>
#include <QMC5883LCompass.h>

double RateP, RateQ, RateR;
double AccX, AccY, AccZ;

QMC5883LCompass compass;
double xo, yo, zo;
Matrix theta_transition = identity(4);
Matrix psi_transition = zeros(4, 3);
Matrix att_prev = identity(3);
Matrix att_curr = identity(3);
Matrix x_prop(7, 1), x_est(7, 1), x_meas_updated(7, 1), dx_meas_updated(7, 1), dx_prop(7, 1), dx_pred(7,1);
Matrix P_prop(7, 7), P_est(7, 7); // To be initialised, state covariance
Matrix R = 0.001 * identity(3);   // To be initialised, Process Noise Covariance
Quaternion q_prop, q_est, q_meas_updated;
Matrix Q = 0.01 * identity(6);
Matrix sigma = 0.01 * identity(6); // This Spectral Noise Covariance
double prop_time;
// mekf variable delcaration
// Matrix att_triad(3, 3);
// Quaternion qo, q_prop;
Vector b_w(0.0, 0.0, 0.0);
// Vector b_f(0.0, 0.0, 0.0);
// Vector a;
// double temp[15][15];
// // in gain
// Matrix H(3, 9);
// Matrix H1(3, 3);
// Matrix K_gain(9, 3);
// Matrix P_pre = 0.10 * identity(9);
// Matrix R = 0.02 * identity(3);
// Matrix attitude(3, 3);
// Vector accel_g(0, 0, -1.0);
// // in update
// Matrix P_update(9, 9);
// Matrix delta_x(9, 1);
// Quaternion q_est;

// // in propagation
Vector w_est;
Vector w_measured;
// Matrix phi(9, 9);
// Matrix Fk(9, 9);
// Matrix Q = zeros(9, 9);
// unsigned long curr_time;
// unsigned long pre_time = 0;
// double dt;
// // double sigma_bw , sigma_bf;
// Vector sigma_w(0.1, 0.1, 0.1);
// double sigma_bw = 0.1;
// double sigma_bf = 0.001;
String message;

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

// void magnet_cal(void)
// {
//     int sx = 0;
//     int sy = 0;
//     int sz = 0;

//     for (int i = 1; i <= 10; i++)
//     {
//         compass.read();
//         sx += compass.getX();
//         sy += compass.getY();
//         sz += compass.getZ();
//     }
//     yo = (double)sx / 10.0;
//     xo = (-1.0) * (double)sy / 10.0;
//     zo = (double)sz / 10.0;
// }

// void triad(void)
// {

//     int x, y, z;
//     // get magnetometer initial values
//     magnet_cal();
//     // Read compass values
//     compass.read();

//     // Return XYZ readings
//     y = (double)compass.getX();
//     x = -1.0 * (double)(compass.getY());
//     z = (double)compass.getZ();

//     // read IMU
//     imuAcceleration();

//     Vector r1(0.0f, 0.0f, -1.0f);
//     Vector r2(xo, yo, zo);
//     r1.normalize();
//     r2.normalize();
//     Vector b1(AccX, AccY, AccZ);

//     Vector b2(x, y, z);
//     b1.normalize();
//     b2.normalize();
//     // triad
//     Vector v1(r1.getX(), r1.getY(), r1.getZ());
//     Vector v2 = r1 % r2;
//     v2.normalize();
//     Vector v3 = r1 % v2;
//     Vector w1(b1.getX(), b1.getY(), b1.getZ());
//     Vector w2 = b1 % b2;
//     w2.normalize();
//     Vector w3 = b1 % w2;
//     Vector v1_v2_v3[] = {v1, v2, v3};
//     Matrix v = matrix_from_vectors(v1_v2_v3, 3);
//     Vector w1_w2_w3[] = {w1, w2, w3};
//     Matrix w = matrix_from_vectors(w1_w2_w3, 3);
//     att_triad = w * v.transpose();
// }

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
    // triad();
    //  attitude = att_triad;
    //  attitude = q_est.attitude_matrix();
    //  // initialize
    //  pre_time = 0;
    //  Once characterized, we can keep sigma_x, sigma_y, sigma_z
    for (int i = 0; i < 3; i++)
    {
        for (int k = 0; k < 3; k++)
        {
            if (i == k)
                R.matrix[i][k] = 0.01;
            else
                R.matrix[i][k] = 0.001;
        }
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

        // Initialising the gyro bias values
        for (int i = 0; i < 3; i++)
        {
            b_w.matrix[i][0] = Total_w.matrix[i][0];
            x_est.matrix[i + 4][0] = b_w.matrix[i][0];
        }
        x_est.matrix[3][0] = 1;
        x_prop = x_est;
        // b_f = Total_acc - Vector(0.0, 0.0, -9.8);
    }
}

// void propogate()
// {
//     w_measured.matrix[0][0] = RateP;
//     w_measured.matrix[1][0] = RateQ;
//     w_measured.matrix[2][0] = -RateR;
//     // Serial.println("w_measured");
//     // w_measured.display();
//     w_est.setX(w_measured.getX() - b_w.getX());
//     w_est.setY(w_measured.getY() - b_w.getY());
//     w_est.setZ(w_measured.getZ() - b_w.getZ());
//     // Serial.println("w_est");
//     // w_est.display();
//     Matrix w_est_cross = w_est.skew_from_vec();
//     Matrix epsilon1 = q_est.epsillon();
//     Matrix Fk = zeros(9, 9);
//     Matrix I3 = identity(3);
//     for (int i = 0; i < 3; i++)
//     {
//         for (int j = 0; j < 3; j++)
//         {
//             Fk.matrix[i][j] = -w_est_cross.matrix[i][j];
//         }
//     }

//     for (int i = 0; i < 3; i++)
//     {
//         for (int j = 0; j < 3; j++)
//         {
//             Fk.matrix[i][3 + j] = -I3.matrix[i][j];
//         }
//     }
//     // Serial.println("Fk");
//     // Fk.display();
//     q_prop = q_est + 0.5 * dt * epsilon1 * w_est;
//     q_prop.normalize();

//     attitude = q_prop.attitude_matrix();
//     // Serial.println("Propogated q");
//     // q_prop.display();
//     Matrix I9 = identity(9);
//     phi = I9 + dt * Fk;

//     // initialise Q
//     for (int i = 0; i < 3; i++)
//     {
//         Q.matrix[3 + i][3 + i] = (sigma_bw * sigma_bw * dt);
//         Q.matrix[6 + i][6 + i] = (sigma_bf * sigma_bf * dt);
//         Q.matrix[i][i + 3] = (-sigma_bw * sigma_bw * dt * dt / 2);
//         Q.matrix[3 + i][i] = (-sigma_bw * sigma_bw * dt * dt / 2);
//     }
//     Q.matrix[0][0] = ((sigma_w.getX() * sigma_w.getX() * dt) + (sigma_bw * sigma_bw * dt * dt / 3));
//     Q.matrix[1][1] = ((sigma_w.getY() * sigma_w.getY() * dt) + (sigma_bw * sigma_bw * dt * dt / 3));
//     Q.matrix[2][2] = ((sigma_w.getZ() * sigma_w.getZ() * dt) + (sigma_bw * sigma_bw * dt * dt / 3));

//     P_pre = phi * P_update * phi.transpose() + Q;
// }

// void measure_update()
// {
//     // gain calculation
//     H1 = Vector(attitude * accel_g).skew_from_vec();
//     // Serial.println("H1");
//     // H1.display();
//     for (int i = 0; i < 3; i++)
//     {
//         for (int k = 3; k < 9; k++)
//         {
//             H.matrix[i][k] = 0.0;
//         }
//     }
//     H.matrix[0][6] = 1.0;
//     H.matrix[1][7] = 1.0;
//     H.matrix[2][8] = 1.0;
//     H.matrix[0][0] = H1.matrix[0][0];
//     H.matrix[0][1] = H1.matrix[0][1];
//     H.matrix[0][2] = H1.matrix[0][2];
//     H.matrix[1][0] = H1.matrix[1][0];
//     H.matrix[1][1] = H1.matrix[1][1];
//     H.matrix[1][2] = H1.matrix[1][2];
//     H.matrix[2][0] = H1.matrix[2][0];
//     H.matrix[2][1] = H1.matrix[2][1];
//     H.matrix[2][2] = H1.matrix[2][2];
//     // Serial.println("H");
//     // H.display();
//     K_gain = P_pre * H.transpose() * (H * P_pre * H.transpose() + R).inverse();
//     // Serial.println("Kalman Gain: ");
//     // K_gain.display();
//     //  update
//     Matrix I = identity(9);
//     P_update = (I - K_gain * H) * P_pre;
//     // Serial.println("K_gain*H");
//     //(K_gain*H).display();
//     // Serial.println("I-K*H");
//     //(I - K_gain * H).display();
//     // Serial.println("P_update");
//     // P_update.display();
//     Matrix temp1(3, 1);
//     Vector rotated_g = attitude * accel_g;
//     // Serial.println("Attitude Matrix");
//     // attitude.display();
//     // Serial.println("accel_g");
//     // accel_g.display();
//     // Serial.println("Rotated g");
//     // rotated_g.display();
//     Vector accel_measured;
//     double acc_norm = sqrt(AccX * AccX + AccY * AccY + AccZ * AccZ);
//     accel_measured = Vector(AccX / acc_norm, AccY / acc_norm, AccZ / acc_norm) - b_f;
//     // Serial.println("accel_meas");
//     // accel_measured.display();
//     temp1.matrix[0][0] = accel_measured.getX() - rotated_g.getX();
//     temp1.matrix[1][0] = accel_measured.getY() - rotated_g.getY();
//     temp1.matrix[2][0] = accel_measured.getZ() - rotated_g.getZ();
//     delta_x = K_gain * temp1;
//     // Serial.println("H1");
//     // H1.display();
//     // Serial.print("State Vector: ");
//     //(1000*delta_x).display();
//     a.setX(delta_x.matrix[0][0]);
//     a.setY(delta_x.matrix[1][0]);
//     a.setZ(delta_x.matrix[2][0]);
//     b_w.setX(b_w.getX() + delta_x.matrix[3][0]);
//     b_w.setY(b_w.getY() + delta_x.matrix[4][0]);
//     b_w.setZ(b_w.getZ() + delta_x.matrix[5][0]);
//     b_f.setX(b_f.getX() + delta_x.matrix[6][0]);
//     b_f.setY(b_f.getY() + delta_x.matrix[7][0]);
//     b_f.setZ(b_f.getZ() + delta_x.matrix[8][0]);
//     // Serial.println("b_w");
//     // b_w.display();
//     Matrix epsilon = q_prop.epsillon();
//     q_est = q_prop + 0.5 * epsilon * a;
//     q_est.normalize();
//     double norm1 = sqrt(q_est.getq1() * q_est.getq1() + q_est.getq2() * q_est.getq2() + q_est.getq3() * q_est.getq3() + q_est.getq4() * q_est.getq4());
//     q_est = q_est / norm1;
//     // Serial.print("q_est, estimated");
//     // q_est.display();
// }

// void propogate2()
// {
//     w_measured.matrix[1][0] = RateQ;
//     w_measured.matrix[2][0] = -RateR;
//     // Serial.println("w_measured");
//     // w_measured.display();
//     w_est.setX(w_measured.getX() - b_w.getX());
//     w_est.setY(w_measured.getY() - b_w.getY());
//     w_est.setZ(w_measured.getZ() - b_w.getZ());
//     // Serial.println("w_est");
//     // w_est.display();
//     Matrix w_est_cross = w_est.skew_from_vec();
//     Matrix epsilon1 = q_est.epsillon();
//     Matrix Fk = zeros(9, 9);
//     Matrix I3 = identity(3);
//     for (int i = 0; i < 3; i++)
//     {
//         for (int j = 0; j < 3; j++)
//         {
//             Fk.matrix[i][j] = -w_est_cross.matrix[i][j];
//         }
//     }

//     for (int i = 0; i < 3; i++)
//     {
//         for (int j = 0; j < 3; j++)
//         {
//             Fk.matrix[i][3 + j] = -I3.matrix[i][j];
//         }
//     }
//     // Serial.println("Fk");
//     // Fk.display();
//     q_prop = q_est + 0.5 * dt * epsilon1 * w_est;
//     q_prop.normalize();

//     attitude = q_prop.attitude_matrix();

//     Matrix G = identity(9);
//     for (int i = 0; i < 3; i++)
//     {
//         G.matrix[i][i] = -1;
//     }

//     // initialise Q
//     // for (int i = 0; i < 3; i++)
//     // {
//     //     Q.matrix[3 + i][3 + i] = (sigma_bw * sigma_bw * dt);
//     //     Q.matrix[6 + i][6 + i] = (sigma_bf * sigma_bf * dt);
//     //     Q.matrix[i][i + 3] = (-sigma_bw * sigma_bw * dt * dt / 2);
//     //     Q.matrix[3 + i][i] = (-sigma_bw * sigma_bw * dt * dt / 2);
//     // }
//     // Q.matrix[0][0] = ((sigma_w.getX() * sigma_w.getX() * dt) + (sigma_bw * sigma_bw * dt * dt / 3));
//     // Q.matrix[1][1] = ((sigma_w.getY() * sigma_w.getY() * dt) + (sigma_bw * sigma_bw * dt * dt / 3));
//     // Q.matrix[2][2] = ((sigma_w.getZ() * sigma_w.getZ() * dt) + (sigma_bw * sigma_bw * dt * dt / 3));
//     Q = G * Q * G.transpose() * dt;

//     P_pre = phi * P_update * phi.transpose() + Q;
// }

// void propogate3()
// {
//     w_measured.matrix[1][0] = RateQ;
//     w_measured.matrix[2][0] = -RateR;
//     // Serial.println("w_measured");
//     // w_measured.display();
//     w_est.setX(w_measured.getX() - b_w.getX());
//     w_est.setY(w_measured.getY() - b_w.getY());
//     w_est.setZ(w_measured.getZ() - b_w.getZ());
//     // Serial.println("w_est");
//     // w_est.display();
//     Matrix w_est_cross = w_est.skew_from_vec();
//     Matrix epsilon1 = q_est.epsillon();
//     Matrix Fk = zeros(9, 9);
//     Matrix I3 = identity(3);
//     for (int i = 0; i < 3; i++)
//     {
//         for (int j = 0; j < 3; j++)
//         {
//             Fk.matrix[i][j] = -w_est_cross.matrix[i][j];
//         }
//     }

//     for (int i = 0; i < 3; i++)
//     {
//         for (int j = 0; j < 3; j++)
//         {
//             Fk.matrix[i][3 + j] = -I3.matrix[i][j];
//         }
//     }
//     // Serial.println("Fk");
//     // Fk.display();
//     q_prop = q_est + 0.5 * dt * epsilon1 * w_est;
//     q_prop.normalize();

//     attitude = q_prop.attitude_matrix();

//     Matrix G = identity(9);
//     for (int i = 0; i < 3; i++)
//     {
//         G.matrix[i][i] = -1;
//     }

//     // initialise Q
//     for (int i = 0; i < 3; i++)
//     {
//         Q.matrix[i][i] = (sigma_w.getX() * sigma_w.getY());
//         Q.matrix[3 + i][3 + i] = (sigma_bw * sigma_bw);
//         Q.matrix[6 + i][i + 6] = (sigma_bf * sigma_bf);
//     }
//     // Q.matrix[0][0] = ((sigma_w.getX() * sigma_w.getX() * dt) + (sigma_bw * sigma_bw * dt * dt / 3));
//     // Q.matrix[1][1] = ((sigma_w.getY() * sigma_w.getY() * dt) + (sigma_bw * sigma_bw * dt * dt / 3));
//     // Q.matrix[2][2] = ((sigma_w.getZ() * sigma_w.getZ() * dt) + (sigma_bw * sigma_bw * dt * dt / 3));
//     // Q = G * Q * G.transpose() * dt;
//     Matrix P_dot = Fk * P_update + P_update * Fk.transpose() + G * Q * G.transpose();
//     P_pre = P_update + P_dot * dt;
// }

// Matrix theta_transition = identity(4);
// Matrix psi_transition = zeros(4, 3);
// Matrix att_prev = identity(3);
// Matrix att_curr = identity(3);
// Matrix x_prop(7, 1), x_est(7, 1), x_meas_updated(7, 1), dx_meas_updated(7, 1), dx_prop(7, 1);
// Matrix P_prop(7, 7), P_est(7, 7); // To be initialised, state covariance
// Matrix R = 0.001 * identity(6);   // To be initialised, Process Noise Covariance
// Quaternion q_prop, q_est, q_meas_updated;
// Matrix Q = 0.01 * identity(6);
// Matrix sigma = 0.01 * identity(6); // This Spectral Noise Covariance
// double prop_time;

void propagateEKF()
{
    // Serial.print("Prop_start");
    Matrix omega = w_est.omega_matrix();
    Matrix eps = q_est.epsillon();
    theta_transition = theta_transition + (0.5 * omega * theta_transition) * prop_time;
    psi_transition = psi_transition + ((0.5 * omega * psi_transition) - (0.5 * eps)) * prop_time;
    Matrix F_matrix(7, 7);
    double f_elements[][7] = {{theta_transition.matrix[0][0], theta_transition.matrix[0][1], theta_transition.matrix[0][2], theta_transition.matrix[0][3], psi_transition.matrix[0][0], psi_transition.matrix[0][1], psi_transition.matrix[0][2]},
                              {theta_transition.matrix[1][0], theta_transition.matrix[1][1], theta_transition.matrix[1][2], theta_transition.matrix[1][3], psi_transition.matrix[1][0], psi_transition.matrix[1][1], psi_transition.matrix[1][2]},
                              {theta_transition.matrix[2][0], theta_transition.matrix[2][1], theta_transition.matrix[2][2], theta_transition.matrix[2][3], psi_transition.matrix[2][0], psi_transition.matrix[2][1], psi_transition.matrix[2][2]},
                              {theta_transition.matrix[3][0], theta_transition.matrix[3][1], theta_transition.matrix[3][2], theta_transition.matrix[3][3], psi_transition.matrix[3][0], psi_transition.matrix[3][1], psi_transition.matrix[3][2]},
                              {0, 0, 0, 0, 1, 0, 0},
                              {0, 0, 0, 0, 0, 1, 0},
                              {0, 0, 0, 0, 0, 0, 1}};
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            F_matrix.matrix[i][j] = f_elements[i][j];
        }
    }
    Matrix I = identity(7);
    Matrix phi = (I + F_matrix * prop_time);
    dx_pred = phi * dx_prop;
    Matrix wt(7, 6);
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            wt.matrix[i][j] = eps.matrix[i][j];
        }
    }
    wt.matrix[6][5] = wt.matrix[5][4] = wt.matrix[4][3] = 1;
    wt = wt * 0.5 * prop_time;
    Q = wt * sigma * wt.transpose();
    Matrix P_pred = phi * P_prop * phi.transpose() + Q;
    P_prop = P_pred;
    dx_prop = dx_pred;
    x_prop = x_prop + dx_pred; // Newly Added
    for (int i = 0; i < 4; i++)
    {
        q_prop.matrix[i][0] = x_prop.matrix[i][0];
    }
    q_prop.normalize();
    for (int i = 0; i < 4; i++)
    {
        x_prop.matrix[i][0] = q_prop.matrix[i][0];
    }
    q_est = q_prop;
    for (int i = 0; i < 3; i++)
    {
        b_w.matrix[i][0] = x_prop.matrix[4 + i][0];
    }
}

void meas_updateEKF()
{
    // Serial.print("Meas_update starts");
    Vector zk(AccX, AccY, AccZ);
    double qx, qy, qz, qw;
    for (int i = 0; i < 4; i++)
    {
        q_prop.matrix[i][0] = x_prop.matrix[i][0];
    }
    qx = q_prop.matrix[0][0];
    qy = q_prop.matrix[1][0];
    qz = q_prop.matrix[2][0];
    qw = q_prop.matrix[3][0];
    Matrix H(3, 7);
    H.matrix[0][0] = AccX * qx + AccY * qy + AccZ * qz;
    H.matrix[0][1] = -AccX * qy + AccX * qy - AccZ * qw;
    H.matrix[0][2] = -AccX * qz + AccX * qw + AccZ * qx;
    H.matrix[0][3] = AccX * qw + AccY * qz - AccZ * qy;
    H.matrix[0][4] = 0;
    H.matrix[0][5] = 0;
    H.matrix[0][6] = 0;
    H.matrix[1][0] = AccX * qy - AccY * qx + AccZ * qw;
    H.matrix[1][1] = AccX * qx + AccY * qy + AccZ * qz;
    H.matrix[1][2] = -AccX * qw - AccY * qz + AccZ * qy;
    H.matrix[1][3] = -AccX * qz + AccY * qw + AccZ * qx;
    H.matrix[1][4] = 0;
    H.matrix[1][5] = 0;
    H.matrix[1][6] = 0;
    H.matrix[2][0] = AccX * qz - AccY * qw - AccZ * qx;
    H.matrix[2][1] = AccX * qw + AccY * qz - AccZ * qy;
    H.matrix[2][2] = AccX * qx + AccY * qy + AccZ * qz;
    H.matrix[2][3] = AccX * qy - AccY * qx + AccZ * qw;
    H.matrix[2][4] = 0;
    H.matrix[2][5] = 0;
    H.matrix[2][6] = 0;
    Vector h_xk = q_prop.attitude_matrix() * Vector(0, 0, -1);
    Matrix K = P_prop * H.transpose() * (H * P_prop * H.transpose() + R).inverse();
    dx_meas_updated = K * (zk - h_xk);
    dx_prop = dx_meas_updated;
    x_est = x_prop + dx_meas_updated;
    x_prop = x_est;
    Matrix I = identity(7);
    P_est = (I - K * H) * P_prop;
    P_prop = P_est;
    for (int i = 0; i < 4; i++)
    {
        q_prop.matrix[i][0] = x_prop.matrix[i][0];
    }
    for (int i = 0; i < 3; i++)
    {
        b_w.matrix[i][0] = x_prop.matrix[4 + i][0];
    }
    q_prop.normalize();
    for (int i = 0; i < 4; i++)
    {
        x_prop.matrix[i][0] = q_prop.matrix[i][0];
    }

    q_est = q_prop;
    // zk.display();
}
// void reset()
// {
//     a.setX(a.getX() + delta_x.matrix[0][0]);
//     a.setY(a.getY() + delta_x.matrix[1][0]);
//     a.setZ(a.getZ() + delta_x.matrix[2][0]);
//     b_w.setX(b_w.getX() + delta_x.matrix[3][0]);
//     b_w.setY(b_w.getY() + delta_x.matrix[4][0]);
//     b_w.setZ(b_w.getZ() + delta_x.matrix[5][0]);
//     b_f.setX(b_f.getX() + delta_x.matrix[6][0]);
//     b_f.setY(b_f.getY() + delta_x.matrix[7][0]);
//     b_f.setZ(b_f.getZ() + delta_x.matrix[8][0]);
//     for (int i = 0; i < 9; i++)
//     {
//         delta_x.matrix[i][0] = 0;
//     }
//     Matrix epsilon1 = q_est.epsillon();
//     q_est = q_est + 0.5 * epsilon1 * a;
//     q_est.normalize();
// }

// void propagate4()
// {
//     w_measured.matrix[1][0] = RateQ;
//     w_measured.matrix[2][0] = -RateR;
//     // Serial.println("w_measured");
//     // w_measured.display();
//     w_est.setX(w_measured.getX() - b_w.getX());
//     w_est.setY(w_measured.getY() - b_w.getY());
//     w_est.setZ(w_measured.getZ() - b_w.getZ());
//     // Serial.println("w_est");
//     // w_est.display();
//     Matrix w_est_cross = w_est.skew_from_vec();
//     Matrix epsilon1 = q_est.epsillon();
//     Matrix Fk = zeros(9, 9);
//     Matrix I3 = identity(3);
//     for (int i = 0; i < 3; i++)
//     {
//         for (int j = 0; j < 3; j++)
//         {
//             Fk.matrix[i][j] = -w_est_cross.matrix[i][j];
//         }
//     }

//     for (int i = 0; i < 3; i++)
//     {
//         for (int j = 0; j < 3; j++)
//         {
//             Fk.matrix[i][3 + j] = -I3.matrix[i][j];
//         }
//     }
//     // Serial.println("Fk");
//     // Fk.display();

//     Vector del_x_dot = Vector(F * del_x);
//     delta_x = delta_x + del_x_dot * dt;

//     Vector tempDev(delta_x.);

//     attitude = q_prop.attitude_matrix();

//     Matrix G = identity(9);
//     for (int i = 0; i < 3; i++)
//     {
//         G.matrix[i][i] = -1;
//     }

//     // initialise Q
//     for (int i = 0; i < 3; i++)
//     {
//         Q.matrix[i][i] = (sigma_w.getX() * sigma_w.getY());
//         Q.matrix[3 + i][3 + i] = (sigma_bw * sigma_bw);
//         Q.matrix[6 + i][i + 6] = (sigma_bf * sigma_bf);
//     }
//     // Q.matrix[0][0] = ((sigma_w.getX() * sigma_w.getX() * dt) + (sigma_bw * sigma_bw * dt * dt / 3));
//     // Q.matrix[1][1] = ((sigma_w.getY() * sigma_w.getY() * dt) + (sigma_bw * sigma_bw * dt * dt / 3));
//     // Q.matrix[2][2] = ((sigma_w.getZ() * sigma_w.getZ() * dt) + (sigma_bw * sigma_bw * dt * dt / 3));
//     // Q = G * Q * G.transpose() * dt;
//     Matrix P_dot = Fk * P_update + P_update * Fk.transpose() + G * Q * G.transpose();
//     P_pre = P_update + P_dot * dt;
// }

void loop()
{

    // prediction
    // curr_time = millis();
    // dt = (curr_time - pre_time) * 0.001;
    prop_time = 0.1;

    imuAcceleration();
    meas_updateEKF();

    // pre_time = curr_time;

    message = String(q_est.getq4()) + ',' + String(q_est.getq1()) + ',' + String(q_est.getq2()) + ',' + String(q_est.getq3());
    Serial.println(message);
    // x_est.display();

    imuOmega();
    w_est = Vector(RateP, RateQ, RateR) - b_w;
    propagateEKF();

    delay(100);
}