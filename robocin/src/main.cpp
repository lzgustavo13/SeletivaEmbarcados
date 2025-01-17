#include "mbed.h"
#include "MPU6050.h"

#define SDA_PIN PB_9
#define SCL_PIN PB_8

MPU6050 mpu(SDA_PIN, SCL_PIN);

int main() {
    // Testa a conexão com o MPU6050
    if (!mpu.testConnection()) {
        printf("Falha ao conectar com o MPU6050!\n");
        return -1;
    }

    printf("Conexão bem-sucedida com o MPU6050.\n");

    while (true) {
        int16_t gyroData[3];
        mpu.setGyroRange(MPU6050_GYRO_RANGE_500); //MPU6050_GYRO_RANGE_500
        mpu.getGyroRaw(gyroData); // estamos obtendo os dados do giroscopio porem devemos fazer as devidas alterações
        
        float giroX = mpu.change(gyroData[0]);
        float giroY = mpu.change(gyroData[1]);
        float giroZ = mpu.change(gyroData[2]);

        mpu.updateAng(giroZ);

        float deslocamento = mpu.getAng();

        printf("Giro (rad/s) - X: %.3f, Y: %.3f, Z: %.3f\n", giroX, giroY, giroZ);
        printf("Deslocamento Angular: %.3f\n", deslocamento);

        thread_sleep_for(1000); // Aguarda 1 segundo
    }
}
