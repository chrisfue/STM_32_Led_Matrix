/*
 * ledFunctions.h
 *
 *  Created on: 19.12.2022
 *      Author: fy
 */

#ifndef INC_LEDFUNCTIONS_H_
#define INC_LEDFUNCTIONS_H_
#include "main.h"
#include "c7x10r_font.h"

/*
 *
 * @brief statische Darstellung zweier ASCII chars - ohne Timer, nicht in Nutzung
 * Stellt in jeweils einer ASCII Matrix einen Char aus einem Char Array der Größe 2 dar
 * @param hspi3: SPI handle Pointer
 * @retval None
 */
void twoChars(char * inputWord,SPI_HandleTypeDef * hspi3);
/*
 *
 * @brief Darstellung beweger Strings - ohne Timer, nicht in Nutzung
 * Stellt weitergegebenen String in Laufschrift dar
 *@ param inputWord: String to display
 *@ param hspi3: SPI handle pointer
 *@ param WordLen: Length of the string to be displayed
 */
void movingChars(char * inputWord,SPI_HandleTypeDef * hspi3,int WordLen );


/*
 * @brief Darstellung einer Zeile
 * Stellt Zeile eines Chars nach Definition in Font dar
 * @param inputWord: String to display
 */
void OneRow(char*inputWord,SPI_HandleTypeDef * hspi3);

#endif /* INC_LEDFUNCTIONS_H_ */
