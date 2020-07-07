/*
 * MorseCodeTranslator.h
 *
 *  Created on: Feb 4, 2020
 *      Author: bnezu
 */
 #include <stdint.h>

#ifndef MORSECODETRANSLATOR_H_
#define MORSECODETRANSLATOR_H_

#define SPACE_UNITS_DOTS 0
#define SPACE_UNITS_LETTERS 1//at normal speed should be 3?
#define SPACE_UNITS_SPACE 2//at normal speed should be 9?
#define SPACE_UNITS_END_OF_MESSAGE 3

#define DASH_UNIT 3//if less than a DASH_UNIT its a DOT_UNIT

char* TranslateCharToMorseCode(char c);

char* Translate(uint8_t *morseCode, uint32_t *count);
char* TranslateSelf();

char TranslateChar(uint8_t val);

void ButtonPress(uint32_t timeDiffrence, uint8_t ButtonStatus);

uint8_t GetSpaceUnit(uint8_t pos);
uint8_t GetSpaceUnitModifier();
void SetSpaceUnitModifier(uint8_t newModifier);

#endif /* MORSECODETRANSLATOR_H_ */