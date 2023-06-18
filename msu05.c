/*
 * test.c
 *
 *  Created on: 18 avr. 2023
 *      Author: YUYU
 */

#include "msu05.h"
#include "main.h"
#include <stm32h7xx_hal_tim.h>
#include <stm32h7xx_hal_tim_ex.h>
extern TIM_HandleTypeDef htim2;
extern float distance;
float vitesse;

//Fonction Trig
void msu05_Trig (void) {
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, 1);
	HAL_Delay(10); //envoyer une impulsion de 10ms à pin trig
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, 0);
	HAL_Delay(1000);
}
//Fonction Mesure
void msu05_Echo (void) {
	static int startTime=0;
	static int old_distance=0;
	static int old_Time;
	static int timeStamp;

	while (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0){}
	startTime =__HAL_TIM_GET_COUNTER(&htim2);
	while (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==1){}
	timeStamp=__HAL_TIM_GET_COUNTER(&htim2);
	distance = (timeStamp-startTime)*1.7/58.9; //datasheet en cm

	vitesse = (distance-old_distance)/(timeStamp-old_Time);

	old_distance = distance;
	old_Time=timeStamp;
}


//Fonction fréquence cardiaque

#define DEVICE_ADDRESS_WRITE 0xAE  // Adresse du capteur MAX30100 (mode écriture)
#define DEVICE_ADDRESS_READ  0xAF  // Adresse du capteur MAX30100 (mode lecture)
#define READ_MODE_MASK       0x01  // Masque pour le mode lecture
#define FIFO_WR_PTR          0x02  // Adresse de FIFO_WR_PTR
#define FIFO_DATA             0x05  // Adresse de FIFO_DATA
#define FIFO_RD_PTR          0x04  // Adresse de FIFO_RD_PTR
extern I2C_HandleTypeDef hi2c1;

int getComponentID(void){
    char toReceive[1]={0};
    HAL_StatusTypeDef dummyStatus;
    dummyStatus=HAL_I2C_Mem_Read(&hi2c1,0xAE,0xFF,1,toReceive,1,-1);
    if (toReceive[0]==0x11)
    	return 1;
    return 0;
}

void sendConfig(){
    char toSend[1]={0x0a};
	HAL_StatusTypeDef dummyStatus;
	dummyStatus=HAL_I2C_Mem_Write(&hi2c1,0xAE,0x06,1,toSend,1,-1);

}

int getPulse(){
	//on lit le registre interruptstatus
	char interrupt[1]={0};
	HAL_StatusTypeDef dummyStatus;

	do{
		dummyStatus=HAL_I2C_Mem_Read(&hi2c1, 0xAE, 0x00, 1, interrupt, 1, -1);
	}while((interrupt[0]&0x80)!=0x80);

	char irdata[16]={0}; // Tableau pour stocker les données IR

	char data[4]={0}; //// Tableau temporaire pour lire les données depuis FIFO_DATA
	for (int i = 0; i < 16; i++)
		{HAL_StatusTypeDef dummyStatus;
		dummyStatus=HAL_I2C_Mem_Read(&hi2c1, 0xAE, 0x05, 1, data, 4, -1);

			        // Extraction des données IR (16 bits) et stockage dans data
		irdata[i] = (data[0] << 8) | data[1];
		}

		for (int i = 0; i < 15; i++) {
			for (int j = i + 1; j < 16; j++) {
				if (irdata[j] < irdata[i]) {
					int temp = irdata[i];
		            irdata[i] = irdata[j];
		            irdata[j] = temp;
		            }
		        }
		     }

		int mediane = (irdata[7]+irdata[8])/2;
		return mediane;
}
//&hi2c1 pointeur vers la structure de configuration I2C
			        //0xAE adresse du périphérique MAX30100 en mode écriture
			        //0x05 adresse de FIFO_DATA
			        //1 taille de l'adresse du registre
			        //4 nombre d'octets à lire
			        //-1 delay maximal = infini

int getTemperature(){
    char toReceive[2]={0};
    HAL_StatusTypeDef dummyStatus;
    dummyStatus=HAL_I2C_Mem_Read(&hi2c1,0xAE,0x16,1,toReceive,2,-1);
    return toReceive[0];
}






/*void getFIFO_WR_PTR(void) {
	uint16_t fifoWrPtr;

    // Début de la communication I2C
    I2C_Start();

    // Envoi de l'adresse du capteur en mode écriture
    I2C_Send(DEVICE_ADDRESS_WRITE);

    // Envoi de l'adresse de FIFO_WR_PTR
    I2C_Send(FIFO_WR_PTR);

    // Répétition du démarrage pour passer en mode lecture
    I2C_RepeatedStart();

    // Envoi de l'adresse du capteur en mode lecture
    I2C_Send(DEVICE_ADDRESS_READ);

    // Lecture des 8 bits de poids fort de FIFO_WR_PTR
    fifoWrPtr = (I2C_Read() << 8);

    // Lecture des 8 bits de poids faible de FIFO_WR_PTR
    fifoWrPtr |= I2C_Read();

    // Arrêt de la communication I2C
    I2C_Stop();

    return fifoWrPtr;
}

//transaction 2 : lecture des échantillons du FIFO
void readFIFOData(uint16_t numSamplesToRead) {
    // Début de la communication I2C
    I2C_Start();

    // Envoi de l'adresse du capteur en mode écriture
    I2C_Send(DEVICE_ADDRESS_WRITE);

    // Envoi de l'adresse de FIFO_DATA
    I2C_Send(FIFO_DATA);

    // Répétition du démarrage pour passer en mode lecture
    I2C_RepeatedStart();

    // Envoi de l'adresse du capteur en mode lecture
    I2C_Send(DEVICE_ADDRESS_READ);

    // Lecture des échantillons du FIFO
    for (uint16_t i = 0; i < numSamplesToRead; i++) {
        // Lecture des données IR[15:8]
        uint8_t irHigh = I2C_Read();
        // Lecture des données IR[7:0]
        uint8_t irLow = I2C_Read();
        // Lecture des données R[15:8]
        uint8_t rHigh = I2C_Read();
        // Lecture des données R[7:0]
        uint8_t rLow = I2C_Read();

        // Traitement des données lues
    }

    // Arrêt de la communication I2C
    I2C_Stop();
}

//transaction 3 : Écriture dans le registre FIFO_RD_PTR
void writeFIFO_RD_PTR(uint16_t newFIFO_RD_PTR) {
    // Début de la communication I2C
    I2C_Start();

    // Envoi de l'adresse du capteur en mode écriture
    I2C_Send(DEVICE_ADDRESS_WRITE);

    // Envoi de l'adresse de FIFO_RD_PTR
    I2C_Send(FIFO_RD_PTR);

    // Écriture de la nouvelle valeur de FIFO_RD_PTR
    I2C_Send((newFIFO_RD_PTR >> 8) & 0xFF);  // 8 bits de poids fort
    I2C_Send(newFIFO_RD_PTR & 0xFF);         // 8 bits de poids faible

    // Arrêt de la communication I2C
    I2C_Stop();
}*/






