/*
 * MorseCodeTranslator.h
 *
 *  Created on: Feb 4, 2020
 *      Author: bnezu
 */
 #include <stdint.h>

#ifndef MORSECODETRANSLATOR_H_
#define MORSECODETRANSLATOR_H_

#define SPACE_UNITS_DOTS 1
#define SPACE_UNITS_LETTERS 5
#define SPACE_UNITS_SPACE 9

#define DASH_UNIT 3//if less than a DASH_UNIT its a DOT_UNIT

char* TranslateCharToMorseCode(char c);

char* Translate(uint32_t *morseCode, uint32_t *count);
char* TranslateSelf();

char TranslateChar(uint8_t val, uint8_t pos);

void ButtonPress(uint32_t timeDiffrence, uint8_t ButtonStatus);

#endif /* MORSECODETRANSLATOR_H_ */