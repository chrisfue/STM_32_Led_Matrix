/**********
 * Generated by Christoph Fuerst
 * 19-12-2022
 */

#include "ledFunctions.h"




//Funktion für statische Darstellung zweier Chars
void twoChars(char * inputWord,SPI_HandleTypeDef *  hspi3 ){
	for(int i =0;i<7;i++){

			  //Mit SPI Text einspielen:
		  HAL_SPI_Transmit_IT(hspi3,(uint8_t*)(font+i+(inputWord[1])*8),1);
		  HAL_SPI_Transmit_IT(hspi3,(uint8_t*) (font+i+(inputWord[0])*8),1);
			if(i==0){
				 HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
			 }

		  //HAL_SPI_Transmit_IT(&hspi3, buffer, 8);

		  //Rising Pin of Latch ->Daten in StoreRegister
		 	  HAL_GPIO_WritePin(Latch_GPIO_Port, Latch_Pin, GPIO_PIN_SET);
		 	 HAL_GPIO_WritePin(Latch_GPIO_Port, Latch_Pin, GPIO_PIN_RESET);


		 	 //Dekadenzähler zum iterieren
		 	if(i!=0){ HAL_GPIO_WritePin(Analog_Click_1_GPIO_Port, Analog_Click_1_Pin, GPIO_PIN_SET);

		 	 	 	  HAL_GPIO_WritePin(Analog_Click_1_GPIO_Port, Analog_Click_1_Pin, GPIO_PIN_RESET);

		 	}
		 	for(int j=0;j<1000;j++){};



		  }

}

//Stellt eine Reihe in der Matrix dar, Wichtig für die Laufschrift, kann nicht ganzes Bild auf einmal Bitshiften
void OneRow(char*inputWord,SPI_HandleTypeDef * hspi3){
	//Transmission via SPI

	HAL_SPI_Transmit(hspi3,(uint8_t*) inputWord, 1, 1);
	HAL_SPI_Transmit(hspi3, (uint8_t*) inputWord+1, 1, 1);

	//Rising Pin of Latch ->Daten in StoreRegister
	HAL_GPIO_WritePin(Latch_GPIO_Port, Latch_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Latch_GPIO_Port, Latch_Pin, GPIO_PIN_RESET);


}



//Funktion für bewegende Characters - nur noch für Nachvollziehbarkeit enthalten
void movingChars(char * inputWord,SPI_HandleTypeDef * hspi3,int WordLen ){
	//Zwischenspeicher für Darstellung
	char output[3];
	int letters=0;
	while(letters<WordLen ){
	//Obere Schleife: Offset fürs Bitshifting für verschobene Darstellung 5 Iterationen	für 5LEDs
	for(int i=0;i<=5;i++){

		for(int time=0;time<30;time++){
		//innere Schleife: Offset fürs Zeilen abrattern und auslesen der font-Einträge
		for(int j=0;j<7;j++){
	//Font aus Tabelle Laden
	output[0]=*(font+j+(inputWord[(1+letters)%WordLen])*8);
	output[1]=*(font+j+(inputWord[(0+letters)%WordLen])*8);

	//Funktionsvariable für Oder Funktion von Buchstaben die außerhalb des Displays einlaufen
	output[2]=*(font+j+(inputWord[(1+1+letters)%WordLen])*8);


	//Bitshift und ODER für Laufschrift:
	output[1]=*(output+1)>>i|output[0]<<(5-i); //linker Buchstabe rückt raus, rechter rückt nach
	output[0]=*(output)>>i|output[2]<<(5-i); //rechter Buchstabe rückt nach links, nächster Buchstabe rückt nach

	//aktuelle Reihe anzeigen
	OneRow(output, hspi3);

	//Reset um wieder bei erster Reihe zu starten
	if(j==0){
				 HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
				 HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
			 }

	 //Dekadenzähler zum iterieren
		 	if(j!=0){ HAL_GPIO_WritePin(Analog_Click_1_GPIO_Port, Analog_Click_1_Pin, GPIO_PIN_SET);

		 	 	 	  HAL_GPIO_WritePin(Analog_Click_1_GPIO_Port, Analog_Click_1_Pin, GPIO_PIN_RESET);

		 	}
		 	for(int k=0;k<1000;k++){};


		}
		}
	}letters++;

	}

}





