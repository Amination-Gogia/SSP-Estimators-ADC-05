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
Matrix x_prop(7, 1), x_est(7, 1), x_meas_updated(7, 1), dx_meas_updated(7, 1), dx_prop(7, 1), dx_pred(7, 1);
Matrix x_small(6, 1), x_small_est(6, 1), x_small_meas_updated(6, 1), dx_small_prop(6, 1), dx_small_pred(6, 1), dx_small_meas_updated(6, 1);
Matrix P_prop(7, 7), P_est(7, 7); // To be initialised, state covariance
Matrix R = 0.001 * identity(3);   // To be initialised, Process Noise Covariance
Quaternion q_prop, q_est, q_meas_updated;
Matrix Q = 0.01 * identity(6);
Matrix sigma = 0.01 * identity(6); // This Spectral Noise Covariance
double prop_time;

Vector b_w(0.0, 0.0, 0.0);

Vector w_est;
Vector w_measured;

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
    // Serial.print(RateP);
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

void meas_updateEKF2()
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

void update_state()
{
}

void update_reduced()
{
}

void propagateEKF2()
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

void loop()
{
    prop_time = 0.1;

    imuAcceleration();
    meas_updateEKF();

    message = String(q_est.getq4()) + ',' + String(q_est.getq1()) + ',' + String(q_est.getq2()) + ',' + String(q_est.getq3());
    Serial.println(message);

    imuOmega();
    w_est = Vector(RateP, RateQ, RateR) - b_w;
    propagateEKF();

    delay(100);
}