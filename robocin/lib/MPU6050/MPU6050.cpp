#include "MPU6050.h"

MPU6050::MPU6050(PinName sda, PinName scl) : connection(sda, scl) {

    this->setSleepMode(false);
    currentGyroRange = 0;
    angZ = 0.0f; //angulo em z
    intTimer.start();

}

//--------------------------------------------------
//-------------------General------------------------
//--------------------------------------------------

void MPU6050::write(char address, char data) {
    char temp[2];
    temp[0]=address;
    temp[1]=data;
    
    connection.write(MPU6050_ADDRESS * 2,temp,2);
}

char MPU6050::read(char address) {
    char retval;
    connection.write(MPU6050_ADDRESS * 2, &address, 1, true);
    connection.read(MPU6050_ADDRESS * 2, &retval, 1);
    return retval;
}

void MPU6050::read(char address, char *data, int length) {
    connection.write(MPU6050_ADDRESS * 2, &address, 1, true);
    connection.read(MPU6050_ADDRESS * 2, data, length);
}

void MPU6050::setSleepMode(bool state) {
    char temp = this->read(MPU6050_PWR_MGMT_1_REG);
    if (state) {
        temp |= (1 << MPU6050_SLP_BIT); // ativa modo de repouso
    } else {
        temp &= ~(1 << MPU6050_SLP_BIT); // desativa modo de repouso
    }
    this->write(MPU6050_PWR_MGMT_1_REG, temp);
}

bool MPU6050::testConnection( void ) {
    char temp;
    temp = this->read(MPU6050_WHO_AM_I_REG);
    return (temp == (MPU6050_ADDRESS & 0xFE));
}

//--------------------------------------------------
//------------------Gyroscope-----------------------
//-------------------------------------------------- 

void MPU6050::setGyroRange(char range) {
    currentGyroRange = range;
    range &= MPU6050_GYRO_CONFIG_MASK; // aplicando a mascara
    char temp = this->read(MPU6050_GYRO_CONFIG_REG);
    temp &= ~(MPU6050_GYRO_CONFIG_MASK << MPU6050_GYRO_CONFIG_SHIFT); // limpar bits
    temp |= (range << MPU6050_GYRO_CONFIG_SHIFT); // novos bits
    this->write(MPU6050_GYRO_CONFIG_REG, temp);
}

//otimizado
void MPU6050::getGyroRaw(int16_t *data) {
    char rawData[6];
    read(MPU6050_GYRO_XOUT_H_REG, rawData, 6);

    // convertendo os dados para numeros de 16 bits, atenção ao sinal
    data[0] = (int16_t)((rawData[0] << 8) | rawData[1]) - offsetX; // X
    data[1] = (int16_t)((rawData[2] << 8) | rawData[3]) - offsetY; // Y
    data[2] = (int16_t)((rawData[4] << 8) | rawData[5]) - offsetZ; // Z
}

int MPU6050::getGyroZ( void ) {
     short retval;
     char data[2];
     this->read(MPU6050_GYRO_ZOUT_H_REG, data, 2);
     retval = (data[0]<<8) + data[1];
     return (int)retval;
}

//implementado
double MPU6050::timerMillisRead(const Timer& timer) {
    return timerRead<chrono::milliseconds>(timer);
}

float MPU6050::change(int16_t raw) {
    float sensitivity;
    if(currentGyroRange == MPU6050_GYRO_RANGE_250){
        sensitivity = 131.0;
    }else if(currentGyroRange == MPU6050_GYRO_RANGE_500){
        sensitivity = 65.5;
    }else if(currentGyroRange == MPU6050_GYRO_RANGE_1000){
        sensitivity = 32.8;
    }else{
        sensitivity = 16.4;
    }
    // convertendo para °/s e depois para rad/s, obs: no dataset de testes o valor
    //ja está em °/s logo modificar essa função caso queira testar com dataset
    return (raw / sensitivity) * 0.0174533;
}

//implementado
void MPU6050::updateAng(float giroZ) {
    double accumulator = 0.0;
    for (int i = 0; i < gyroSample; i++) { 
        double rawValue = getGyroZ();
        rawValue = change(rawValue);  // converte para rad/s
        if (fabs(rawValue) >= 0.03) {
            accumulator += rawValue;
        }
    }
    accumulator /= gyroSample;  // média dos valores

    double gyroElapsedTime = timerRead<chrono::milliseconds>(intTimer);
    const double elapsedTimeSeconds = gyroElapsedTime * 0.001; // converte ms para s
    angZ += accumulator * elapsedTimeSeconds;

    intTimer.reset(); // reseta o timer
    
    if (angZ > M_PI) { // mantem o angulo limitado entre -pi e pi
        angZ -= 2 * M_PI;
    } else if (angZ < -M_PI) {
        angZ += 2 * M_PI;
    }
}

float MPU6050::getAng() {
    return angZ;
}

float MPU6050::filtroPassaBaixas(float entrada, float alpha) {
    static float saida = 0.0f; 
    saida = alpha * entrada + (1 - alpha) * saida; // aplicando o filtro
    return saida; 
}

void MPU6050::calibrateOffset(int samples) {
    int32_t sumX = 0, sumY = 0, sumZ = 0; //int32 para aumentar a precisão devido a soma de muitos valores

    for (int i = 0; i < samples; i++) {
        int16_t rawData[3]; 
        getGyroRaw(rawData); // obtendo os dados brutos

        sumX += rawData[0]; 
        sumY += rawData[1];
        sumZ += rawData[2];

        ThisThread::sleep_for(5ms); // aguarda entre as leituras
    }

    offsetX = sumX / samples;
    offsetY = sumY / samples;
    offsetZ = sumZ / samples; 
}
