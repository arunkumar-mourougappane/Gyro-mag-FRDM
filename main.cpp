#include "mbed.h"
#include "MPU9250.h"
DigitalOut myled(LED_GREEN);
Serial pc(USBTX, USBRX);
MPU9250 mpu9250;
Timer t;
int main()
{
    float sum = 0;
    uint32_t sumCount = 0;

    pc.baud(19200);
    pc.printf("Hello World!\n");
    i2c.frequency(400000);
    pc.printf("CPU SystemCoreClock is %d Hz\r\n", SystemCoreClock);
    t.start();
        uint8_t whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
       pc.printf("I AM 0x%x\n\r", whoami); pc.printf("I SHOULD BE 0x71\n\r");
       if(whoami==0x71)
        {
            pc.printf("MPU9250 is online...\n\r");
            wait(0.5f); // wait a small period of time
            mpu9250.resetMPU9250(); // Reset registers to default in preparation for device calibration
            mpu9250.calibrateMPU9250(mpu9250.gyroBias, mpu9250.accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
            pc.printf("x gyro bias = %f\n\r", mpu9250.gyroBias[0]);
            pc.printf("y gyro bias = %f\n\r", mpu9250.gyroBias[1]);
            pc.printf("z gyro bias = %f\n\r", mpu9250.gyroBias[2]);
            pc.printf("x accel bias = %f\n\r", mpu9250.accelBias[0]);
            pc.printf("y accel bias = %f\n\r", mpu9250.accelBias[1]);
            pc.printf("z accel bias = %f\n\r", mpu9250.accelBias[2]);
            wait(2);
            mpu9250.initMPU9250(); 
            pc.printf("MPU9250 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
            mpu9250.initAK8963(mpu9250.magCalibration);
            pc.printf("AK8963 initialized for active data mode....\n\r"); // Initialize device for active mode read of magnetometer
            pc.printf("Accelerometer full-scale range = %f  g\n\r", 2.0f*(float)(1<<mpu9250.Ascale));
            pc.printf("Gyroscope full-scale range = %f  deg/s\n\r", 250.0f*(float)(1<<mpu9250.Gscale));
            if(mpu9250.Mscale == 0) pc.printf("Magnetometer resolution = 14  bits\n\r");
            if(mpu9250.Mscale == 1) pc.printf("Magnetometer resolution = 16  bits\n\r");
            if(mpu9250.Mmode == 2) pc.printf("Magnetometer ODR = 8 Hz\n\r");
            if(mpu9250.Mmode == 6) pc.printf("Magnetometer ODR = 100 Hz\n\r");
            wait(2);
        }
        else
        {
                 pc.printf("Could not connect to MPU9250: \n\r");
                 pc.printf("%#x \n",  whoami);
                while(true);
        }
        mpu9250.getAres(); // Get accelerometer sensitivity
        mpu9250.getGres(); // Get gyro sensitivity
        mpu9250.getMres(); // Get magnetometer sensitivity
        pc.printf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f/mpu9250.aRes);
        pc.printf("Gyroscope sensitivity is %f LSB/deg/s \n\r", 1.0f/mpu9250.gRes);
        pc.printf("Magnetometer sensitivity is %f LSB/G \n\r", 1.0f/mpu9250.mRes);
        mpu9250.magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
        mpu9250.magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
        mpu9250.magbias[2] = +125.;  // User environmental x-axis correction in milliGauss
    while (true) {
         if(mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt

    mpu9250.readAccelData(mpu9250.accelCount);  // Read the x/y/z adc values   
    // Now we'll calculate the accleration value into actual g's
    mpu9250.ax = (float)mpu9250.accelCount[0]*mpu9250.aRes - mpu9250.accelBias[0];  // get actual g value, this depends on scale being set
    mpu9250.ay = (float)mpu9250.accelCount[1]*mpu9250.aRes - mpu9250.accelBias[1];   
    mpu9250.az = (float)mpu9250.accelCount[2]*mpu9250.aRes - mpu9250.accelBias[2];  
   
    mpu9250.readGyroData(mpu9250.gyroCount);  // Read the x/y/z adc values
    // Calculate the gyro value into actual degrees per second
    mpu9250.gx = (float)mpu9250.gyroCount[0]*mpu9250.gRes - mpu9250.gyroBias[0];  // get actual gyro value, this depends on scale being set
    mpu9250.gy = (float)mpu9250.gyroCount[1]*mpu9250.gRes - mpu9250.gyroBias[1];  
    mpu9250.gz = (float)mpu9250.gyroCount[2]*mpu9250.gRes - mpu9250.gyroBias[2];   
  
    mpu9250.readMagData(mpu9250.magCount);  // Read the x/y/z adc values   
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mpu9250.mx = (float)mpu9250.magCount[0]*mpu9250.mRes*mpu9250.magCalibration[0] - mpu9250.magbias[0];  // get actual magnetometer value, this depends on scale being set
    mpu9250.my = (float)mpu9250.magCount[1]*mpu9250.mRes*mpu9250.magCalibration[1] - mpu9250.magbias[1];  
    mpu9250.mz = (float)mpu9250.magCount[2]*mpu9250.mRes*mpu9250.magCalibration[2] - mpu9250.magbias[2];   
  }
   
    mpu9250.Now = t.read_us();
    mpu9250.deltat = (float)((mpu9250.Now - mpu9250.lastUpdate)/1000000.0f) ; // set integration time by time elapsed since last filter update
    mpu9250.lastUpdate = mpu9250.Now;
    
    sum += mpu9250.deltat;
    sumCount++;
    
//    if(lastUpdate - firstUpdate > 10000000.0f) {
//     beta = 0.04;  // decrease filter gain after stabilized
//     zeta = 0.015; // increasey bias drift gain after stabilized
 //   }
    
   // Pass gyro rate as rad/s
  mpu9250.MadgwickQuaternionUpdate(mpu9250.ax, mpu9250.ay,mpu9250. az, mpu9250.gx*PI/180.0f,mpu9250. gy*PI/180.0f,mpu9250. gz*PI/180.0f, mpu9250. my,mpu9250.  mx,mpu9250. mz);
 // mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);

    // Serial print and/or display at 0.5 s rate independent of data rates
     mpu9250.delt_t = t.read_ms() -  mpu9250.count;
    if ( mpu9250.delt_t > 500) { // update LCD once per half-second independent of read rate

    pc.printf("ax = %f", 1000* mpu9250.ax); 
    pc.printf(" ay = %f", 1000* mpu9250.ay); 
    pc.printf(" az = %f  mg\n\r", 1000* mpu9250.az); 

    pc.printf("gx = %f",  mpu9250.gx); 
    pc.printf(" gy = %f",  mpu9250.gy); 
    pc.printf(" gz = %f  deg/s\n\r",  mpu9250.gz); 
    
    pc.printf("gx = %f",  mpu9250.mx); 
    pc.printf(" gy = %f",  mpu9250.my); 
    pc.printf(" gz = %f  mG\n\r",  mpu9250.mz); 
    
     mpu9250.tempCount = mpu9250.readTempData();  // Read the adc values
     mpu9250.temperature = ((float)  mpu9250.tempCount) / 333.87f + 21.0f; // Temperature in degrees Centigrade
    pc.printf(" temperature = %f  C\n\r",  mpu9250.temperature); 
    
    pc.printf("q0 = %f\n\r",  mpu9250.q[0]);
    pc.printf("q1 = %f\n\r",  mpu9250.q[1]);
    pc.printf("q2 = %f\n\r",  mpu9250.q[2]);
    pc.printf("q3 = %f\n\r",  mpu9250.q[3]);      
    }

    
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
     mpu9250.yaw   = atan2(2.0f * ( mpu9250.q[1] *  mpu9250.q[2] +  mpu9250.q[0] *  mpu9250.q[3]),  mpu9250.q[0] *  mpu9250.q[0] +  mpu9250.q[1] *  mpu9250.q[1] -  mpu9250.q[2] *  mpu9250.q[2] -  mpu9250.q[3] *  mpu9250.q[3]);   
     mpu9250.pitch = -asin(2.0f * ( mpu9250.q[1] *  mpu9250.q[3] -  mpu9250.q[0] *  mpu9250.q[2]));
     mpu9250.roll  = atan2(2.0f * ( mpu9250.q[0] *  mpu9250.q[1] +  mpu9250.q[2] *  mpu9250.q[3]),  mpu9250.q[0] *  mpu9250.q[0] -  mpu9250.q[1] *  mpu9250.q[1] -  mpu9250.q[2] *  mpu9250.q[2] +  mpu9250.q[3] *  mpu9250.q[3]);
     mpu9250.pitch *= 180.0f / PI;
     mpu9250.yaw   *= 180.0f / PI; 
     mpu9250.yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
     mpu9250.roll  *= 180.0f / PI;

    pc.printf("Yaw, Pitch, Roll: %f %f %f\n\r",  mpu9250.yaw,  mpu9250.pitch, mpu9250. roll);
    pc.printf("average rate = %f\n\r", (float) sumCount/sum);
 
    myled= !myled;
    mpu9250.count = t.read_ms(); 
    sum = 0;
    sumCount = 0; 
}
}
