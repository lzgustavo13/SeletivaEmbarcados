/* Use #define MPU6050_ES before you include this file if you have an engineering sample (older EVBs will have them), to find out if you have one, check your accelerometer output. 
If it is half of what you expected, and you still are on the correct planet, you got an engineering sample
*/
#ifndef MPU6050_H
#define MPU6050_H

/**
 * Includes
 */
#include "mbed.h"

/**
 * Pins
 */
#define SDA_PIN PB_9
#define SCL_PIN PB_8

/**
 * Defines
 */
#ifndef MPU6050_ADDRESS
    #define MPU6050_ADDRESS             0x68 
#endif
/**
 * Registers
 */
#define MPU6050_CONFIG_REG         0x1A
#define MPU6050_GYRO_CONFIG_REG    0x1B
#define MPU6050_INT_PIN_CFG        0x37
#define MPU6050_GYRO_XOUT_H_REG    0x43
#define MPU6050_GYRO_YOUT_H_REG    0x45
#define MPU6050_GYRO_ZOUT_H_REG    0x47
#define MPU6050_PWR_MGMT_1_REG     0x6B
#define MPU6050_WHO_AM_I_REG       0x75

/**
 * Definitions
 */
#define MPU6050_SLP_BIT             6 // repouso

#define MPU6050_GYRO_CONFIG_MASK 0x03 // máscara para intervalo do giroscópio visando melhorar legitibilidade
#define MPU6050_GYRO_CONFIG_SHIFT 3  // deslocamento para config giroscópio

#define MPU6050_GYRO_RANGE_250      0 //range 
#define MPU6050_GYRO_RANGE_500      1
#define MPU6050_GYRO_RANGE_1000     2
#define MPU6050_GYRO_RANGE_2000     3

#define SAMPLE_RATE 5 // ms
#define SAMPLE_OFFSET 1000 // quantidade de amostras para média do offset

/** MPU6050 IMU library.
 */
class MPU6050 {
    public:
     /**
      * Constructor.
      * Sleep mode of MPU6050 is immediatly disabled
      * @param sda - mbed pin to use for the SDA I2C line.
      * @param scl - mbed pin to use for the SCL I2C line.
      */
     MPU6050(PinName sda, PinName scl);
     
     /**
      * Tests the I2C connection by reading the WHO_AM_I register. 
      * @return True for a working connection, false for an error
      */     
     bool testConnection( void );

     /**
      * Sets the Gyro full-scale range
      * Macros: MPU6050_GYRO_RANGE_250 - MPU6050_GYRO_RANGE_500 - MPU6050_GYRO_RANGE_1000 - MPU6050_GYRO_RANGE_2000
      * @param range - The two bits that set the full-scale range (use the predefined macros)
      */
     void setGyroRange(char range);

     /**
      * Reads all gyro data.
      * @param data - pointer to signed integer array with length three: data[0] = X, data[1] = Y, data[2] = Z
      */   
     void getGyroRaw( int16_t *data );  
        
     float change(int16_t raw);

     /**
     * retorna o deslocamento angular acumulado em z (rad).
     */
     float getAng();
     // update
     void updateAng(float giroZ);

     /**
      * Sets the sleep mode of the MPU6050 
      * @param state - true for sleeping, false for wake up
      */     
     void setSleepMode( bool state );
     
     /**
      * Writes data to the device, could be private, but public is handy so you can transmit directly to the MPU. 
      * @param adress - register address to write to
      * @param data - data to write
      */
     void write( char address, char data);
     
     /**
      * Read data from the device, could be private, but public is handy so you can transmit directly to the MPU. 
      * @param adress - register address to write to
      * @return - data from the register specified by RA
      */
     char read( char adress);
     
     /**
      * Read multiple registers from the device, more efficient than using multiple normal reads. 
      * @param adress - register address to write to
      * @param length - number of bytes to read
      * @param data - pointer where the data needs to be written to 
      */
     void read( char adress, char *data, int length);

     int getGyroZ();

     template <class ToDuration, class T = float>
        T timerRead(const Timer& timer) {
        return chrono::duration_cast<chrono::duration<T, typename ToDuration::period>>(timer.elapsed_time()).count();
        }

     double timerMillisRead(const Timer& timer);

     void calibrateOffset(int samples);

     float filtroPassaBaixas(float entrada, float alpha);

     private:
     I2C connection;
     char currentGyroRange;
     float angZ; // deslocamento angular acumulado
     Timer intTimer;     // timer p/ medir intervalo
     double gyroSample = 3.0; // numero de amostras do giroscopio
     int16_t offsetX = 0, offsetY = 0, offsetZ = 0; // offset do giroscopio
};

#endif
