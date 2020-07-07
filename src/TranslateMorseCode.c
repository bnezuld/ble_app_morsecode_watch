/*
 * MorseCodeTranslator.c
 *
 *  Created on: Feb 4, 2020
 *      Author: bnezu
 */

#include "TranslateMorseCode.h"

const uint8_t SPACE_UNITS[4] = {1,3,7,11};//{1,8,12,15}
uint8_t spaceUnitModifier = 1u;

const char MorseCodeHashTable[] = {' ','*','E','T',
                                   'I','A','N','M','S','U','R','W','D','K','G','O',
                                   'H','V','F','*','L','*','P','J','B','X','C','Y','Z','Q','*','*',
                                   '5','4','*','3','*','*','*','2','*','*','*','*','*','*','*','1','6','*','/','*','*','*','*','*','7','*','*','*','8','*','9','0',
                                   '*','*','*','*','*','*','*','*','*',',','*','*','?','*','*','*','*','*','"','*','*','.','*','*','*','*','*','*','*','*','\'','*','*','-','*','*','*','*','*','*','*','*','*','*','*',')','*','*','*','*','*','*','*','*','*','*',':'};

/*hash table from char to morseCode
1	*----	Period	*-*-*-
2	**---	Comma	--**--
3	***--	Colon	---***
4	****-	Question Mark	**--**
5	*****	Apostrophe	*----*
6	-****	Hyphen	-****-
7	--***	Fraction Bar	-**-*
8	---**	Parentheses	-*--*-
9	----*	Quotation Marks	*-**-*
0	-----	*/
const char* TranslateToMorseCodeHash[] = {"\0","\0",".-..-.","\0","\0","\0","\0","..--..","-.--.-","-.--.-","\0","\0","--..--","-....-",".-.-.-","-..-.","-----",".----","..---","...--","....-",".....","-....","--...","---..","----.","---...","\0","\0","\0","\0","..--..","\0",".-","-...","-.-.","-..",".","..-.","--.","....","..",".---","-.-",".-..","--","-.","---",".--.","--.-",".-.","...","-","..-","...-",".--","-..-","-.--","--..","\0","-..-."};

//const uint8_t TranslateToMorseCodeLength[] = {2,4,4,3,1,4,3,4,2,4,3,4,2,2,3,4,4,3,3,1,3,4,3,4,4,4};

#define MAX_MORSECODE 300
uint32_t button[MAX_MORSECODE];
uint32_t buttonCount = 0;

uint8_t IncomingMorseCode[MAX_MORSECODE];
uint32_t IncomingMorseCodeCount = 0;

char* TranslateCharToMorseCode(char c)
{
	if(c >= 32 && c <= 92)
		return TranslateToMorseCodeHash[c - 32];
	return (void *) 0;
}

char* TranslateSelf()
{
        IncomingMorseCodeCount++;
	return Translate(IncomingMorseCode,&IncomingMorseCodeCount);
}

char* Translate(uint8_t *morseCode, uint32_t *count)
{
	uint32_t tmpCount = 0;
	char *c = malloc((*count + 1) * sizeof(char));
	uint8_t stringCount = 0;
	uint8_t position = 1;
	uint8_t morseCodeValue = 0;
	uint8_t translateChar;
	while(tmpCount != *count)
	{
                c[stringCount++] = TranslateChar(morseCode[tmpCount++]);
	}
        IncomingMorseCodeCount = 0;
        IncomingMorseCode[0] = 0;
	c[stringCount] = '\0';
	*count = 0;
	return c;
}

char TranslateChar(uint8_t val)
{
        //121 is currect size of the MorseCodeHashTable array
        if(val < 121)
        {
          return MorseCodeHashTable[val];
        }else
        {
          return '*';
        }
}

void ButtonPress(uint32_t timeDiffrence, uint8_t buttonStatus)
{
	if(buttonStatus == 0){// button released
                if(IncomingMorseCode[IncomingMorseCodeCount] != 0)
                {
                        if(timeDiffrence <= GetSpaceUnit(SPACE_UNITS_LETTERS) - 1)//cotinue with current morse code value
                        {
                            IncomingMorseCode[IncomingMorseCodeCount] = IncomingMorseCode[IncomingMorseCodeCount] << 1;
                        }else if(timeDiffrence <= GetSpaceUnit(SPACE_UNITS_SPACE))//this is the next letter
                        {
                            IncomingMorseCodeCount++;
                            IncomingMorseCode[IncomingMorseCodeCount] = 0;
                        }else//this is a space and then the next letter
                        {
                            IncomingMorseCodeCount++;
                            IncomingMorseCode[IncomingMorseCodeCount] = 0;
                            IncomingMorseCodeCount++;
                            IncomingMorseCode[IncomingMorseCodeCount] = 0;
                        }
                }
	}else{// button pressed
                if(IncomingMorseCode[IncomingMorseCodeCount] == 0)
                {
                     IncomingMorseCode[IncomingMorseCodeCount] = 2;   
                }
                if(timeDiffrence >= 2)
                {
                        IncomingMorseCode[IncomingMorseCodeCount] |= 1;
                }
	}
}

uint8_t GetSpaceUnit(uint8_t pos)
{
    if(pos == 0)
    {
        return SPACE_UNITS[pos];
    }
    return SPACE_UNITS[pos] * spaceUnitModifier;
}

uint8_t GetSpaceUnitModifier()
{
    return spaceUnitModifier;
}

void SetSpaceUnitModifier(uint8_t newModifier)
{
    if(newModifier < 0u)
    {
        newModifier = 1u;
    }
    else if(newModifier > SPACE_UNIT_MAX)
    {
        newModifier = SPACE_UNIT_MAX;
    }
    spaceUnitModifier = newModifier;
}


