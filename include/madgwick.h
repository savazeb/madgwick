//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
//=====================================================================================================

#ifndef _MADGWICK_H_
#define _MADGWICK_H_

#include <math.h>

namespace mw
{

#define sampleFreq 100.0f // sample frequency in Hz
#define betaDef 0.02f     // 2 * proportional gain

    class madgwick
    {
    public:
        //---------------------------------------------------------------------------------------------------
        // Function declarations
        //---------------------------------------------------------------------------------------------------
        // AHRS algorithm update
        void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
        void update(float gx, float gy, float gz, float ax, float ay, float az);

        //---------------------------------------------------------------------------------------------------
        // Fast inverse square-root
        // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
        float invSqrt(float x);

    private:
        float beta = betaDef;                             // 2 * proportional gain (Kp)
        float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame
    };

};

#endif
