#include "mbed.h"
#include "MPU6050.h"

MPU6050 mpu(SDA_PIN, SCL_PIN);
Timer timer; // timer para controlar a taxa de amostragem

const float alpha = 0.25; // ajuste do fator de suavização, quanto maior menor suavização, porém melhor responsividade

int main() {
    // testa a conexão com o MPU6050
    if (!mpu.testConnection()) {
        printf("Falha ao conectar com o MPU6050!\n");
        return -1;
    }
    printf("Conexão bem-sucedida com o MPU6050.\n");

    mpu.calibrateOffset(SAMPLE_OFFSET); // calibra o offset do giroscópio
    mpu.setGyroRange(MPU6050_GYRO_RANGE_2000); //MPU6050_GYRO_RANGE_2000

    timer.start();

    while (true){
        if(timer.elapsed_time().count() >= SAMPLE_RATE * 1000){ //count em microsegundos por isso * 1000
            timer.reset();

            int16_t gyroData[3];
            mpu.getGyroRaw(gyroData); // estamos obtendo os dados do giroscopio porem devemos fazer as devidas alterações
            
            float giroX = mpu.change(gyroData[0]); // X
            float giroY = mpu.change(gyroData[1]); // Y
            float giroZ = mpu.change(gyroData[2]); // Z que é o que nos interessa

            float giroZ_suavizado = mpu.filtroPassaBaixas(giroZ, alpha); // aplicando o filtro passa-baixas

            mpu.updateAng(giroZ_suavizado); 
            float deslocamento = mpu.getAng(); // obtendo o deslocamento angular acumulado em z (rad)

            printf("GiroZ (rad/s) - %.3f\n", giroZ_suavizado);
            printf("Deslocamento Angular: %.3f\n", deslocamento);
        }
    }
}
