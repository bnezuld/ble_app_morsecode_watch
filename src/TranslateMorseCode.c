/*
 * MorseCodeTranslator.c
 *
 *  Created on: Feb 4, 2020
 *      Author: bnezu
 */

#include "TranslateMorseCode.h"

/*could do this in a hashtable by using the 'position' to calculate the offset needed*/
const char MorseCodeTable0[] = {'E','T'};
const char MorseCodeTable1[] = {'I','N','A','M'};
const char MorseCodeTable2[] = {'S','D','R','G','U','K','W','O'};
const char MorseCodeTable3[] = {'H','B','L','Z','F','C','P','-','V','X','-','Q','-','Y','J','-'};

/*hash table from char to morseCode*/
const char* TranslateToMorseCode[] = {".-","-...","-.-.","-..",".","..-.","--.","....","..",".---","-.-",".-..","--","-.","---",".--.","--.-",".-.","...","-","..-","...-",".--","-..-","-.--","--.."};
//const uint8_t TranslateToMorseCodeLength[] = {2,4,4,3,1,4,3,4,2,4,3,4,2,2,3,4,4,3,3,1,3,4,3,4,4,4};

#define MAX_MORSECODE 300
uint32_t button[MAX_MORSECODE];
uint32_t buttonCount = 0;

char* TranslateCharToMorseCode(char c)
{
	if(c >= 65 && c <= 65 + 25)
		return TranslateToMorseCode[c - 65];
	return (void *) 0;
}

char* TranslateSelf()
{
	return Translate(button,&buttonCount);
}

char* Translate(uint32_t *morseCode, uint32_t *count)
{
	uint32_t tmpCount = 0;
	char *c = malloc((*count/2 + 1) * sizeof(char));
	uint8_t stringCount = 0;
	uint8_t position = 1;
	uint8_t morseCodeValue = 0;
	uint8_t translateChar;
	while(tmpCount != *count)
	{
		translateChar = 0;
		if(tmpCount % 2 == 0){//a beep
			if(morseCode[tmpCount] >= 2){//dash(using 2 to get a better range, as 3 units represents a space/dash)
				morseCodeValue |= position;
			}
			position = position << 1;
		}else{//a space
			uint32_t i = morseCode[tmpCount];
			if(i >= SPACE_UNITS_LETTERS - 1){//next letter(using 2 to get a better range, as 3 units represents a space/dash)
				translateChar++;
			}
			if (i >= SPACE_UNITS_SPACE - 1){//next word
				translateChar++;
			}
		}
		tmpCount++;
		if(translateChar > 0 || tmpCount == *count){
			c[stringCount++] = TranslateChar(morseCodeValue, position >> 1);
			morseCodeValue = 0;
			position = 1;
		}
		if(translateChar > 1)
		{
			c[stringCount++] = ' ';
		}
	}
	c[stringCount] = '\0';
	*count = 0;
	return c;
}

char TranslateChar(uint8_t val, uint8_t pos)
{
	switch(pos)
	{
	case 1:
		return MorseCodeTable0[val];
	case 2:
		return MorseCodeTable1[val];
	case 4:
		return MorseCodeTable2[val];
	case 8:
		return MorseCodeTable3[val];
	}
	return '-';
}

void ButtonPress(uint32_t timeDiffrence, uint8_t buttonStatus)
{
	if(buttonStatus == 0){// button released
		if(buttonCount != 0){
			button[buttonCount++] = timeDiffrence;
			if(buttonCount >= MAX_MORSECODE)
			{
				buttonCount = 0;
			}
		}
	}else{// button pressed
		button[buttonCount++] = timeDiffrence;
		if(buttonCount >= MAX_MORSECODE)
		{
			buttonCount = 0;
		}
	}
}

