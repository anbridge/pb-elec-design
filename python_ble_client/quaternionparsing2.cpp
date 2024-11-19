#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
using namespace std;

/*
this code parses the data input of format (which was received in one of iterations of read_raw_sensor data file): 

"accel: <-40.52734375,-870.1171875,-774.4140625>
gyro: <42.01526641845703,26.076335906982422,3.503816843032837>
mag: <-85.05000305175781,40.04999923706055,-43.20000076293945>"

"
Paste what is in between the "" as standart input
Don't include temperature!

The code needs some starting quaternion on which the changes will be applied. 
The code doesn't need all history of previous sesor readings, just the most current
sensor reading and the most recent quaternion 




*/


class MadgwickFilter {
public:
    MadgwickFilter(float beta = 0.1f, float sampleFreq = 100.0f)
        : beta(beta), sampleFreq(sampleFreq), q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f) {} //starting quaternion

    void update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
        float qDot1, qDot2, qDot3, qDot4, norm;

        // Convert gyroscope degrees/sec to radians/sec
        gx = gx * (M_PI / 180.0f);
        gy = gy * (M_PI / 180.0f);
        gz = gz * (M_PI / 180.0f);

        // Normalize accelerometer measurements
        norm = sqrt(ax * ax + ay * ay + az * az);
        if (norm == 0.0f) return;
        ax /= norm;
        ay /= norm;
        az /= norm;

        // Normalize magnetometer measurements
        norm = sqrt(mx * mx + my * my + mz * mz);
        if (norm == 0.0f) return;
        mx /= norm;
        my /= norm;
        mz /= norm;

        // Compute rate of change of quaternion
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Apply feedback step
        qDot1 -= beta * (2.0f * (q1 * q3 - q0 * q2) - ax);
        qDot2 -= beta * (2.0f * (q0 * q1 + q2 * q3) - ay);
        qDot3 -= beta * (1.0f - 2.0f * (q1 * q1 + q2 * q2) - az);
        qDot4 -= beta * (2.0f * (q1 * q2 + q0 * q3) - mx);

        // Integrate rate of change of quaternion
        q0 += qDot1 * (1.0f / sampleFreq);
        q1 += qDot2 * (1.0f / sampleFreq);
        q2 += qDot3 * (1.0f / sampleFreq);
        q3 += qDot4 * (1.0f / sampleFreq);

        // Normalize quaternion
        norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    }

    void getQuaternion(float& q0_out, float& q1_out, float& q2_out, float& q3_out) {
        q0_out = q0;
        q1_out = q1;
        q2_out = q2;
        q3_out = q3;
    }

private:
    float beta;             // Algorithm gain
    float sampleFreq;       // Sample frequency in Hz
    float q0, q1, q2, q3;   // Quaternion elements representing the orientation
};

int main() {
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    string inputLine;

    // Prompt user for sensor data
    cout << "Enter sensor data in the following format:" << endl;
    cout << "accel: <x, y, z>" << endl;
    cout << "gyro: <x, y, z>" << endl;
    cout << "mag: <x, y, z>" << endl;

    // Read accelerometer data
    getline(cin, inputLine);
    if (inputLine.rfind("accel:", 0) == 0) {
        stringstream ss(inputLine.substr(7));
        char discard;
        ss >> discard >> ax >> discard >> ay >> discard >> az >> discard;
    }

    // Read gyroscope data
    getline(cin, inputLine);
    if (inputLine.rfind("gyro:", 0) == 0) {
        stringstream ss(inputLine.substr(6));
        char discard;
        ss >> discard >> gx >> discard >> gy >> discard >> gz >> discard;
    }

    // Read magnetometer data
    getline(cin, inputLine);
    if (inputLine.rfind("mag:", 0) == 0) {
        stringstream ss(inputLine.substr(5));
        char discard;
        ss >> discard >> mx >> discard >> my >> discard >> mz >> discard;
    }

    // Create an instance of the Madgwick filter
    MadgwickFilter madgwick(0.1f, 100.0f);  // beta = 0.1, sampleFreq = 100 Hz

    // Update the filter with the sensor data
    madgwick.update(ax, ay, az, gx, gy, gz, mx, my, mz);

    // Retrieve and print the quaternion representing orientation
    float q0, q1, q2, q3;
    madgwick.getQuaternion(q0, q1, q2, q3);

    cout << "Orientation Quaternion: " << endl;
    cout << "q0: " << q0 << endl;
    cout << "q1: " << q1 << endl;
    cout << "q2: " << q2 << endl;
    cout << "q3: " << q3 << endl;

    return 0;
}
