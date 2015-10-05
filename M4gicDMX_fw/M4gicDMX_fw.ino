/**************************************************************************************************

	M4gicDMX
		Simple 4ch DMX-controller for RGBW-LED-Fixtures
		debugout on usbserial interface: 115200baud

	hardware:
		Board:
			Arduino Leonardo compatible

	libraries used:
		~ DMXSerial
			Copyright (c) 2005-2012 by Matthias Hertel,
			http://www.mathertel.de
		~ slight_ButtonInput
			written by stefan krueger (s-light),
				stefan@s-light.eu, http://s-light.eu, https://github.com/s-light/
			cc by sa, Apache License Version 2.0, MIT

	written by vincent maurer (BrixFX),
		github@s-light.eu, https://github.com/brixfx/
	written by stefan krueger (s-light),
		github@s-light.eu, http://s-light.eu, https://github.com/s-light/


	changelog / history
		03.10.2015 10:10 created


	TO DO:
		~ xx


**************************************************************************************************/
/**************************************************************************************************
	The MIT License (MIT)

	Copyright (c) 2015 Vincent Maurer & Stefan Krüger

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
**************************************************************************************************/

/**************************************************************************************************/
/** Includes:  (must be at the beginning of the file.)                                           **/
/**************************************************************************************************/
// use "" for files in same directory as .ino
//#include "file.h"

// #include <Wire.h> // TWI / I2C lib

#include <LiquidCrystal.h>

#include <DMXSerial.h>

#include <slight_ButtonInput.h>


/**************************************************************************************************/
/** info                                                                                         **/
/**************************************************************************************************/
void print_info(Print &pOut) {
	pOut.println();
	//             "|~~~~~~~~~|~~~~~~~~~|~~~..~~~|~~~~~~~~~|~~~~~~~~~|"
	pOut.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
	pOut.println(F("|                       ^ ^                      |"));
	pOut.println(F("|                      (0,0)                     |"));
	pOut.println(F("|                      ( _ )                     |"));
	pOut.println(F("|                       \" \"                      |"));
	pOut.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
	pOut.println(F("| M4gicDMX.ino"));
	pOut.println(F("|   Simple 4ch DMX-controller for RGBW-LED-Fixtures"));
	pOut.println(F("|"));
	pOut.println(F("| This Sketch has a debug-menu:"));
	pOut.println(F("| send '?'+Return for help"));
	pOut.println(F("|"));
	pOut.println(F("| dream on & have fun :-)"));
	pOut.println(F("|"));
	pOut.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
	pOut.println(F("|"));
	//pOut.println(F("| Version: Nov 11 2013  20:35:04"));
	pOut.print(F("| version: "));
	pOut.print(F(__DATE__));
	pOut.print(F("  "));
	pOut.print(F(__TIME__));
	pOut.println();
	pOut.println(F("|"));
	pOut.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
	pOut.println();

	//pOut.println(__DATE__); Nov 11 2013
	//pOut.println(__TIME__); 20:35:04
}


/** Serial.print to Flash: Notepad++ Replace RegEx
	Find what:		Serial.print(.*)\("(.*)"\);
	Replace with:	Serial.print\1\(F\("\2"\)\);
**/


/**************************************************************************************************/
/** definitions (gloabl)                                                                         **/
/**************************************************************************************************/


/************************************************/
/**  DebugOut                                  **/
/************************************************/

boolean bLEDState = 0;
const byte cbID_LED_Info = 9; //D9

unsigned long ulDebugOut_LiveSign_TimeStamp_LastAction	= 0;
const uint16_t cwDebugOut_LiveSign_UpdateInterval		= 1000; //ms

boolean bDebugOut_LiveSign_Serial_Enabled	= 0;
boolean bDebugOut_LiveSign_LED_Enabled		= 1;


/************************************************/
/** Menu Input                                 **/
/************************************************/

// a string to hold new data
char  sMenu_Input_New[]				= "x:TestValue";
// flag if string is complete
bool bMenu_Input_Flag_BF		= false; // BufferFull
bool bMenu_Input_Flag_EOL		= false;
bool bMenu_Input_Flag_CR		= false;
bool bMenu_Input_Flag_LF		= false;
bool bMenu_Input_Flag_LongLine	= false;
bool bMenu_Input_Flag_SkipRest	= false;

// string for Currently to process Command
char  sMenu_Command_Current[]		= "x:TestValue ";


/**********************************************/
/**  Output system                           **/
/**********************************************/
// DualWrite from pYro_65 read more at: http://forum.arduino.cc/index.php?topic=200975.0
class DualWriter : public Print{
	public:
		DualWriter( Print &p_Out1, Print &p_Out2 ) : OutputA( p_Out1 ), OutputB( p_Out2 ){}

		size_t write( uint8_t u_Data ) {
			OutputA.write( u_Data );
			OutputB.write( u_Data );
			return 0x01;
		}
	protected:
		Print &OutputA;
		Print &OutputB;
};

// DualWriter dwOUT( Serial, Serial1);


/**************************************************/
/**  slight ButtonInput                          **/
/**************************************************/

// slight_ButtonInput myButtonFixture(
// 	42, // uint8_t cbID_New
// 	6, // uint8_t cbPin_New,
// 	myButton_getInput, // tCbfuncGetInput cbfuncGetInput_New,
// 	myButton_onEvent, // tcbfOnEvent cbfCallbackOnEvent_New,
// 	  30, // const uint16_t cwDuration_Debounce_New = 30,
// 	2000, // const uint16_t cwDuration_HoldingDown_New = 1000,
// 	  50, // const uint16_t cwDuration_ClickSingle_New =   50,
// 	3000, // const uint16_t cwDuration_ClickLong_New =   3000,
// 	 100  // const uint16_t cwDuration_ClickDouble_New = 1000
// );


/*slight_ButtonInput(
	byte cbID_New,
	byte cbPin_New,
	tCbfuncGetInput cbfuncGetInput_New,
	tcbfOnEvent cbfCallbackOnEvent_New,
	const uint16_t cwDuration_Debounce_New = 30,
	const uint16_t cwDuration_HoldingDown_New = 1000,
	const uint16_t cwDuration_ClickSingle_New = 50,
	const uint16_t cwDuration_ClickLong_New = 3000,
	const uint16_t cwDuration_ClickDouble_New = 500
);
*/
const uint16_t cwButton_Debounce		=   30;
const uint16_t cwButton_HoldingDown		= 2000;
const uint16_t cwButton_ClickSingle		=   50;
const uint16_t cwButton_ClickLong		= 5000;
const uint16_t cwButton_ClickDouble		=  300;

const uint8_t myButtons_COUNT = 4;
slight_ButtonInput myButtons[myButtons_COUNT] = {
	slight_ButtonInput(
		0,
		6,
		myButton_getInput,
		myButton_onEvent,
		cwButton_Debounce,
		cwButton_HoldingDown,
		cwButton_ClickSingle,
		cwButton_ClickLong,
		cwButton_ClickDouble
	),
	slight_ButtonInput(
		1,
		5,
		myButton_getInput,
		myButton_onEvent,
		cwButton_Debounce,
		cwButton_HoldingDown,
		cwButton_ClickSingle,
		cwButton_ClickLong,
		cwButton_ClickDouble
	),
	slight_ButtonInput(
		2,
		4,
		myButton_getInput,
		myButton_onEvent,
		cwButton_Debounce,
		cwButton_HoldingDown,
		cwButton_ClickSingle,
		cwButton_ClickLong,
		cwButton_ClickDouble
	),
	slight_ButtonInput(
		3,
		3,
		myButton_getInput,
		myButton_onEvent,
		cwButton_Debounce,
		cwButton_HoldingDown,
		cwButton_ClickSingle,
		cwButton_ClickLong,
		cwButton_ClickDouble
	),
};


/************************************************/
/** Display                                    **/
/************************************************/

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

/************************************************/
/** fader                                      **/
/************************************************/

const uint8_t fader_COUNT = 4;

const uint8_t ciPin_Fader[fader_COUNT] = {A0, A1, A2, A3};
// const uint8_t ciPin_Fader_Count = sizeof(ciPin_Fader)/ sizeof(uint8_t);

uint8_t fader_value[fader_COUNT] = {0,0,0,0};
const uint8_t fader_names[fader_COUNT] = {'R', 'G', 'B', 'W'};
uint8_t fader_value_live = B00000000;


/**************************************************/
/**  DMXSerial                                   **/
/**************************************************/
const byte cbPIN_DMX_direction = 2;

/**************************************************/
/**  Fixtures                                    **/
/**************************************************/
const uint8_t fixture_COUNT = 3;
uint8_t fixture_values[fixture_COUNT][fader_COUNT];
uint8_t fixture_selected = B00000000;
uint8_t fixture_current = 0;
// fixture_current == 0 means no fixture. we will start to count by 1.
// this helps to get the special case of 'no fixture selected.'

/************************************************/
/** other things...                            **/
/************************************************/


/**************************************************************************************************/
/** functions                                                                                    **/
/**************************************************************************************************/

/************************************************/
/**  Debug things                              **/
/************************************************/

// http://forum.arduino.cc/index.php?topic=183790.msg1362282#msg1362282
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}


void printBinary8(byte bIn)  {

	for (unsigned int mask = 0b10000000; mask; mask >>= 1) {
		// check if this bit is set
		if (mask & bIn) {
			Serial.print('1');
		}
		else {
			Serial.print('0');
		}
	}
}

void printBinary12(uint16_t bIn)  {
	//                       B12345678   B12345678
	//for (unsigned int mask = 0x8000; mask; mask >>= 1) {
	for (unsigned int mask = 0b100000000000; mask; mask >>= 1) {
		// check if this bit is set
		if (mask & bIn) {
			Serial.print('1');
		}
		else {
			Serial.print('0');
		}
	}
}

void printBinary16(uint16_t wIn)  {
	for (unsigned int mask = 0b1000000000000000; mask; mask >>= 1) {
	// check if this bit is set
		if (mask & wIn) {
			Serial.print('1');
		}
		else {
			Serial.print('0');
		}
	}
}



/************************************************/
/**  Menu System                               **/
/************************************************/

// Modes for Menu Switcher
const uint8_t cbMenuMode_MainMenu	= 1;
const uint8_t cbMenuMode_SubMenu1	= 2;
uint8_t bMenuMode = cbMenuMode_MainMenu;


// SubMenu SetValues
void handleMenu_Sub1(Print &pOut, char *caCommand) {
	pOut.println(F("SubMenu1:"));
	pOut.println(F("\t nothing here."));
	pOut.println(F("\t finished."));
	// exit submenu
	// reset state manschine of submenu
	// jump to main
	bMenuMode = cbMenuMode_MainMenu;
	bMenu_Input_Flag_EOL = true;
}


// Main Menu
void handleMenu_Main(Print &pOut, char *caCommand) {
	/* pOut.print("sCommand: '");
	pOut.print(sCommand);
	pOut.println("'"); */
	switch (caCommand[0]) {
		case 'h':
		case 'H':
		case '?': {
			// help
			pOut.println(F("____________________________________________________________"));
			pOut.println();
			pOut.println(F("Help for Commands:"));
			pOut.println();
			pOut.println(F("\t '?': this help"));
			pOut.println(F("\t 'i': sketch info"));
			pOut.println(F("\t 'y': toggle DebugOut livesign print"));
			pOut.println(F("\t 'Y': toggle DebugOut livesign LED"));
			pOut.println(F("\t 'x': tests"));
			pOut.println();
			pOut.println(F("\t 'A': Show 'HelloWorld' "));
			pOut.println(F("\t 'f': select next fixture 'f'"));
			pOut.println(F("\t 'F': toggle fixture 'F0'..'F2'"));
			pOut.println();
			pOut.println(F("\t 'set:' enter SubMenu1"));
			pOut.println();
			pOut.println(F("____________________________________________________________"));
		} break;
		case 'i': {
			print_info(pOut);
		} break;
		case 'y': {
			pOut.println(F("\t toggle DebugOut livesign Serial:"));
			bDebugOut_LiveSign_Serial_Enabled = !bDebugOut_LiveSign_Serial_Enabled;
			pOut.print(F("\t bDebugOut_LiveSign_Serial_Enabled:"));
			pOut.println(bDebugOut_LiveSign_Serial_Enabled);
		} break;
		case 'Y': {
			pOut.println(F("\t toggle DebugOut livesign LED:"));
			bDebugOut_LiveSign_LED_Enabled = !bDebugOut_LiveSign_LED_Enabled;
			pOut.print(F("\t bDebugOut_LiveSign_LED_Enabled:"));
			pOut.println(bDebugOut_LiveSign_LED_Enabled);
		} break;
		case 'x': {
			// get state
			pOut.println(F("__________"));
			pOut.println(F("Tests:"));

			pOut.println(F("nothing to do."));

			// uint16_t wTest = 65535;
			uint16_t wTest = atoi(&caCommand[1]);
			pOut.print(F("wTest: "));
			pOut.print(wTest);
			pOut.println();

			pOut.print(F("1: "));
			pOut.print((byte)wTest);
			pOut.println();

			pOut.print(F("2: "));
			pOut.print((byte)(wTest>>8));
			pOut.println();

			pOut.println();

			pOut.println(F("__________"));
		} break;
		//--------------------------------------------------------------------------------
		case 'A': {
			pOut.println(F("\t Hello World! :-)"));
		} break;
		case 'f': {
			pOut.println(F("\t select next fixture "));
			fixtureSelectNext();
		} break;
		case 'F': {
			pOut.print(F("\t toggle fixture "));

			uint8_t bID = atoi(&caCommand[1]);

			pOut.print(bID);
			// pOut.print(F(" : "));
			// uint16_t wValue = atoi(&caCommand[3]);
			// pOut.print(wValue);
			pOut.println();

			fixtureToggle(bID);
			// pOut.print(F("\t demo for parsing values --> finished."));
		} break;
		//--------------------------------------------------------------------------------
		case 's': {
			// SubMenu1
			if ( (caCommand[1] == 'e') && (caCommand[2] == 't') && (caCommand[3] == ':') ) {
				//if full command is 'set:' enter submenu
				bMenuMode = cbMenuMode_SubMenu1;
				if(1){	//if ( caCommand[4] != '\0' ) {
					//full length command
					//handle_SetValues(pOut, &caCommand[4]);
				} else {
					bMenu_Input_Flag_EOL = true;
				}
			}
		} break;
		//--------------------------------------------------------------------------------
		default: {
			pOut.print(F("command '"));
			pOut.print(caCommand);
			pOut.println(F("' not recognized. try again."));
			sMenu_Input_New[0] = '?';
			bMenu_Input_Flag_EOL = true;
		}
	} //end switch

	//end Command Parser
}


// Menu Switcher
void menuSwitcher(Print &pOut, char *caCommand) {
	switch (bMenuMode) {
			case cbMenuMode_MainMenu: {
				handleMenu_Main(pOut, caCommand);
			} break;
			case cbMenuMode_SubMenu1: {
				handleMenu_Sub1(pOut, caCommand);
			} break;
			default: {
				// something went wronge - so reset and show MainMenu
				bMenuMode = cbMenuMode_MainMenu;
			}
		} // end switch bMenuMode
}

// Check for NewLineComplete and enter menuSwitcher
// sets Menu Output channel (pOut)
void check_NewLineComplete() {
	// complete line found:
		if (bMenu_Input_Flag_EOL) {
			// Serial.println(F("check_NewLineComplete"));
			// Serial.println(F("  bMenu_Input_Flag_EOL is set. "));
			// Serial.print  (F("    sMenu_Input_New: '"));
			// Serial.print(sMenu_Input_New);
			// Serial.println(F("'"));

			// Serial.println(F("  Flags:"));
			// Serial.print  (F("    bMenu_Input_Flag_BF: '"));
			// Serial.println(bMenu_Input_Flag_BF);
			// Serial.print  (F("    bMenu_Input_Flag_CR: '"));
			// Serial.println(bMenu_Input_Flag_CR);
			// Serial.print  (F("    bMenu_Input_Flag_LF: '"));
			// Serial.println(bMenu_Input_Flag_LF);
			// Serial.print  (F("    bMenu_Input_Flag_EOL: '"));
			// Serial.println(bMenu_Input_Flag_EOL);


			// Serial.println(F("  copy sMenu_Input_New to sMenu_Command_Current."));
			// copy to current buffer
			strcpy(sMenu_Command_Current, sMenu_Input_New);

			// Serial.println(F("  clear sMenu_Input_New"));
			// reset memory
			memset(sMenu_Input_New,'\0',sizeof(sMenu_Input_New)-1);

			// Serial.println(F("  clear bMenu_Input_Flag_EOL"));
			// reset flag
			bMenu_Input_Flag_EOL = false;
			bMenu_Input_Flag_LF = false;

			// print info if things were skipped.
			if (bMenu_Input_Flag_BF) {
				Serial.println(F("input was to long. used first part - skipped rest."));
				bMenu_Input_Flag_BF = false;
			}

			// parse line / run command
			menuSwitcher(Serial, sMenu_Command_Current);


			// Serial.print  (F("    sMenu_Input_New: '"));
			// Serial.print(sMenu_Input_New);
			// Serial.println(F("'"));
			// Serial.print  (F("    sMenu_Command_Current: '"));
			// Serial.print(sMenu_Command_Current);
			// Serial.println(F("'"));


			// // Serial.println(F("  check bMenu_Input_Flag_SkipRest"));
			// if ( !bMenu_Input_Flag_SkipRest) {
				// // Serial.println(F("   parse Line:"));

				// if (bMenu_Input_Flag_BF) {
					// Serial.println(F("input was to long. used first part - skipped rest."));
					// bMenu_Input_Flag_BF = false;
				// }

				// // parse line / run command
				// menuSwitcher(Serial, sMenu_Command_Current);

				// if(bMenu_Input_Flag_LongLine) {
					// bMenu_Input_Flag_SkipRest = true;
					// bMenu_Input_Flag_LongLine = false;
				// }
			// } else {
				// // Serial.println(F("   skip rest of Line"));
				// bMenu_Input_Flag_SkipRest = false;
			// }

		}// if Flag complete
}

/************************************************/
/**  Serial Receive Handling                   **/
/************************************************/

void handle_SerialReceive() {
	// collect next input text
	while ((!bMenu_Input_Flag_EOL) && (Serial.available())) {
	// while (Serial.available()) {
		// get the new byte:
		char charNew = (char)Serial.read();
		/*Serial.print(F("charNew '"));
		Serial.print(charNew);
		Serial.print(F("' : "));
		Serial.println(charNew, DEC);*/

		// collect next full line
		/* http://forums.codeguru.com/showthread.php?253826-C-String-What-is-the-difference-between-n-and-r-n
			'\n' == 10 == LineFeed == LF
			'\r' == 13 == Carriage Return == CR
			Windows: '\r\n'
			Linux: '\n'
			Apple: '\r'
		*/
		// check for line end
		switch (charNew) {
			case '\r': {
				// Serial.println(F("incoming char is \\r: set EOL"));
				bMenu_Input_Flag_EOL = true;
				bMenu_Input_Flag_CR = true;
				// bMenu_Input_Flag_LF = false;
			} break;
			case '\n': {
				// Serial.println(F("incoming char is \\n: set EOL"));
				// Serial.println(F("  Flags:"));
					// Serial.print  (F("    bMenu_Input_Flag_BF: '"));
					// Serial.println(bMenu_Input_Flag_BF);
					// Serial.print  (F("    bMenu_Input_Flag_CR: '"));
					// Serial.println(bMenu_Input_Flag_CR);
					// Serial.print  (F("    bMenu_Input_Flag_LF: '"));
					// Serial.println(bMenu_Input_Flag_LF);
					// Serial.print  (F("    bMenu_Input_Flag_EOL: '"));
					// Serial.println(bMenu_Input_Flag_EOL);



				bMenu_Input_Flag_LF = true;

				// Serial.println(F("  check for CR"));
				// check if last char was not CR - if this is true set EOL - else ignore.
				if(!bMenu_Input_Flag_CR) {
					bMenu_Input_Flag_EOL = true;
				} else {
					bMenu_Input_Flag_CR = false;
				}

				// Serial.println(F("  Flags:"));
					// Serial.print  (F("    bMenu_Input_Flag_BF: '"));
					// Serial.println(bMenu_Input_Flag_BF);
					// Serial.print  (F("    bMenu_Input_Flag_CR: '"));
					// Serial.println(bMenu_Input_Flag_CR);
					// Serial.print  (F("    bMenu_Input_Flag_LF: '"));
					// Serial.println(bMenu_Input_Flag_LF);
					// Serial.print  (F("    bMenu_Input_Flag_EOL: '"));
					// Serial.println(bMenu_Input_Flag_EOL);


				// this check also works for windows double line ending
				//if (strlen(sMenu_Input_New) > 0) {
					// bMenu_Input_Flag_EOL = true;
				// }
			} break;
			default: {
				// normal char -
				// add it to the sMenu_Input_New:
				//check for length
				if (strlen(sMenu_Input_New) < (sizeof(sMenu_Input_New)-1) ) {
					sMenu_Input_New[strlen(sMenu_Input_New)] = charNew;
				} else {
					//Serial.println(F(" line to long! ignoring rest of line"));
					// set complete flag so line will be parsed
					// Serial.println(F("Buffer is full: set EOL; set LongLine"));
					//bMenu_Input_Flag_EOL = true;
					bMenu_Input_Flag_BF = true;
					// skip rest of line
					bMenu_Input_Flag_LongLine = true;
				}
			}// default
		}// switch charNew

		//check_NewLineComplete();
	}
}


/************************************************/
/** input handler                              **/
/************************************************/

void faderRead(){
	for (uint8_t i = 0; i < fader_COUNT; i++) {
		uint16_t iFader_raw =  analogRead(ciPin_Fader[i]);
		fader_value[i] = map(iFader_raw, 0, 1023, 0, 255);
	}
}

// void mapFader2Fixture() {
// 	for (uint8_t indexCh = 0; indexCh < fader_COUNT; indexCh++) {
// 		fixture_values[fixture_current][indexCh] = fader_value[indexCh];
// 	}
// }

void faderCheckLive(uint8_t faderID, uint8_t fixtureID) {
    if(fixtureID == fixture_current) {
    	if(fixture_values[fixtureID][faderID] == fader_value[faderID]) {
        	// set this fader live
			fader_value_live = fader_value_live | (1 << faderID);
        }
    }
}

void mapFader2Fixture() {
    for (uint8_t indexFixture = 0; indexFixture < fixture_COUNT; indexFixture++) {
	    // check if fixture is selected
	    if( (fixture_selected & (1 << indexFixture)) > 0) {
		    // update fixture values
		    for (uint8_t indexFader = 0; indexFader < fader_COUNT; indexFader++) {
			    if( (fader_value_live & (1 << indexFader)) > 0) {
			    	fixture_values[indexFixture][indexFader] = fader_value[indexFader];
			    } else {
				    // first check fader values!!!
				    faderCheckLive(indexFader, indexFixture);
			    }
		    }
        }
    }
}

void fixtureSelectNext() {
	uint8_t newId = fixture_current +1;

	if (newId > fixture_COUNT) {
		newId = 1;
	}

	fixture_current = newId;
}

void fixtureToggle(uint8_t fixtureID) {
	if (fixtureID < fixture_COUNT) {
		// How do you set, clear and toggle a single bit in C/C++?
		// http://stackoverflow.com/a/47990/574981
		// https://www.arduino.cc/en/Reference/BitwiseAnd
		// http://playground.arduino.cc/Code/BitMath#bitwise_xor

		// Test 4 XOR (toggle bit)
		//    B100100000 Input
		//  ^ B100000100 Mask
		//    B000100100 Output

		// toggle fixture
		// fixture_selected = fixture_selected ^ (1 << fixtureID);
		fixture_selected = fixture_selected ^ (B00000001 << fixtureID);

		// check if fixture is selected (= bit set)
		if( (fixture_selected & (1 << fixtureID)) > 0) {
			// move focus to this fixtureID:
			fixture_ID_new = fixtureID+1;
		} else {
			// set current to 0 (= no fixture)
			fixture_ID_new = 0;
		}

		if(fixture_ID_new != fixture_current) {
			fixture_current = fixture_ID_new;
			// reset fader_value_live
			fader_value_live = B00000000;
		}

	}
}

/************************************************/
/**  slight_ButtonInput things                 **/
/************************************************/

void myButtons_init() {
	for (byte bIndex = 0; bIndex < myButtons_COUNT; bIndex++) {
		pinMode(myButtons[bIndex].getPin(), INPUT_PULLUP);
		myButtons[bIndex].begin();
	}
}

void myButtons_update() {
	for (byte bIndex = 0; bIndex < myButtons_COUNT; bIndex++) {
		myButtons[bIndex].update();
	}
}

boolean myButton_getInput(uint8_t bID, uint8_t bPin) {
	// read input invert reading - button closes to GND.
	// check HWB
	// return ! (PINE & B00000100);
	return ! digitalRead(bPin);
}


void myButton_onEvent(slight_ButtonInput *pInstance, uint8_t bEvent) {

	// Serial.print(F("Instance ID:"));
	// Serial.println((*pInstance).getID());

	Serial.print(F("Event: "));
	(*pInstance).printEvent(Serial, bEvent);
	Serial.println();

	uint8_t bButtonIndex = (*pInstance).getID();

	// show event additional infos:
	switch (bEvent) {
		/*case slight_ButtonInput::event_StateChanged : {
			Serial.print(F("\t state: "));
			(*pInstance).printState(Serial);
			Serial.println();
		} break;*/
		// click
		/*case slight_ButtonInput::event_Down : {
			Serial.println(F("the button is pressed down! do something.."));
		} break;*/
		/*case slight_ButtonInput::event_HoldingDown : {
			Serial.print(F("duration active: "));
			Serial.println((*pInstance).getDurationActive());
		} break;*/
		/*case slight_ButtonInput::event_Up : {
			Serial.println(F("up"));
		} break;*/
		case slight_ButtonInput::event_Click : {
			Serial.println(F("click"));
			// fixtureSelectNext();

			// toggle Fixture selection
			switch (bButtonIndex) {
				// toggle fixtures
				case 0:
				case 1:
				case 2:
				case 3: {
					fixtureToggle(bButtonIndex);
				} break;
				// menu
				case 4: {
					// enter
				} break;
				case 5: {
					// back
				} break;
				case 6: {
					// up
				} break;
				case 7: {
					// down
				} break;
				default: {

				}
			}

		} break;
		// case slight_ButtonInput::event_ClickLong : {
		// 	Serial.println(F("click long"));
		// } break;
		/*case slight_ButtonInput::event_ClickDouble : {
			Serial.println(F("click double"));
		} break;*/
		/*case slight_ButtonInput::event_ClickTriple : {
			Serial.println(F("click triple"));
		} break;*/
		/*case slight_ButtonInput::event_ClickMulti : {
			Serial.print(F("click count: "));
			Serial.println((*pInstance).getClickCount());
		} break;*/
	} //end switch
}


/************************************************/
/** Display                                    **/
/************************************************/

void printByteAlignRight(Print &pOut, uint8_t bValue) {
	//uint8_t bOffset = 0;
	if (bValue < 100) {
		if (bValue < 10) {
			//bOffset = 2;
			pOut.print(F("  "));
		} else {
			//bOffset = 1;
			pOut.print(F(" "));
		}
	}
	pOut.print(bValue);
}

void printByteAsPercentValueAlignRight(Print &pOut, uint8_t bValue) {
	uint8_t valueP = map(bValue, 0, 255, 0, 100);
	if (valueP == 100) {
		// 100 = FF
		pOut.print(F("FF"));
	} else {
		if (valueP < 10) {
			pOut.print(F(" "));
		}
		pOut.print(valueP);
	}
}


void displayFaderValues() {
	// R00*G 1 BFF W50
	for (uint8_t indexFader = 0; indexFader < fader_COUNT; indexFader++) {
		uint8_t xPos = indexFader * 4;

		lcd.setCursor(xPos, 1);
		lcd.print((char)fader_names[indexFader]);

		lcd.setCursor(xPos+1, 1);
		// lcd.print(iFader);
		// printByteAlignRight(lcd, fader_value[indexFader]);
		// printByteAsPercentValueAlignRight(lcd, fader_value[indexFader]);
        // check for a active fixture
		if(fixture_current > 0) {
			printByteAsPercentValueAlignRight(lcd, fixture_values[fixture_current][indexFader]);
		} else {
			lcd.print(F("--"));
		}

		lcd.setCursor(xPos+3, 1);
		// check if fader is live (= bit set)
		if( (fader_value_live & (1 << indexFader)) > 0) {
			lcd.print(F(" "));
		} else {
			lcd.print(F("*"));
		}
	}
}

void displayFixture() {
	// F2:1234
	uint8_t xPos = 16-(3+fixture_COUNT);

	lcd.setCursor(xPos, 0);
	lcd.print(F("F"));
	xPos = xPos +1;

	lcd.setCursor(xPos, 0);
	lcd.print(fixture_current);
	xPos = xPos +1;

	lcd.setCursor(xPos, 0);
	lcd.print(F(":"));
	xPos = xPos +1;

	for (uint8_t index = 0; index < fixture_COUNT; index++) {
		uint8_t xPos2 = xPos + (index * 1);
		lcd.setCursor(xPos2, 0);
		// check if fixture is selected (= bit set)
		if( (fixture_selected & (1 << index)) > 0) {
			lcd.print(index+1);
		} else {
			lcd.print(F(" "));
		}
	}

}

void displayUpdate() {
	displayFixture();
	displayFaderValues();
}

/************************************************/
/** DMX handling                               **/
/************************************************/

void dmxUpdateFaderValuesDirect() {
	// simple direct fader send
	for (uint8_t i = 0; i < fader_COUNT; i++) {
		DMXSerial.write(i+1, fader_value[i]);
	}
}

void dmxUpdateFixtureValues() {
	for (uint8_t indexFixture = 0; indexFixture < fixture_COUNT; indexFixture++) {
		for (uint8_t indexCh = 0; indexCh < fader_COUNT; indexCh++) {
			DMXSerial.write(
				indexFixture + indexCh + 1,
				fixture_values[indexFixture][indexCh]
			);
		}
	}
}

void dmxUpdate() {
	// dmxUpdateFaderValuesDirect();
	dmxUpdateFixtureValues();
}

/**************************************************/
/**                                              **/
/**************************************************/






/****************************************************************************************************/
/** Setup                                                                                          **/
/****************************************************************************************************/
void setup() {

	/************************************************/
	/** Initialise PINs                            **/
	/************************************************/

		//LiveSign
		pinMode(cbID_LED_Info, OUTPUT);
		digitalWrite(cbID_LED_Info, HIGH);

		// as of arduino 1.0.1 you can use INPUT_PULLUP

	/************************************************/
	/** init serial                                **/
	/************************************************/

		// for ATmega32U4 devices:
		#if defined (__AVR_ATmega32U4__)
			//wait for arduino IDE to release all serial ports after upload.
			delay(2000);
		#endif

		Serial.begin(115200);

		// for ATmega32U4 devices:
		#if defined (__AVR_ATmega32U4__)
			// Wait for Serial Connection to be Opend from Host or 6second timeout
			unsigned long ulTimeStamp_Start = millis();
			while( (! Serial) && ( (millis() - ulTimeStamp_Start) < 6000 ) ) {
				1;
			}
		#endif

		Serial.println();

		Serial.print(F("# Free RAM = "));
		Serial.println(freeRam());

	/************************************************/
	/** Welcom                                     **/
	/************************************************/

		print_info(Serial);

	/************************************************/
	/** setup DMXSerial                            **/
	/************************************************/

		Serial.print(F("# Free RAM = "));
		Serial.println(freeRam());

		Serial.println(F("setup DMXSerial:"));{

			// Serial.println(F("\t set direction pin as 'SEND' "));
			// pin for direction
			// pinMode(cbPIN_DMX_direction, OUTPUT);
			// set to send mode
			// digitalWrite(cbPIN_DMX_direction, HIGH);

			Serial.println(F("\t init as DMXController"));
			DMXSerial.init(DMXController);

			// Serial.println(F("\t set first Pixel to Yellow"));
			// DMXSerial.write(0, 255);
			// DMXSerial.write(1, 255);
		}
		Serial.println(F("\t finished."));


	/************************************************/
	/** setup Display                              **/
	/************************************************/

		Serial.print(F("# Free RAM = "));
		Serial.println(freeRam());

		Serial.println(F("setup Display:")); {

			// set up the LCD's number of columns and rows:
			Serial.println(F("\t init to 2x16"));
			lcd.begin(16, 2);
			// Print a message to the LCD.
			lcd.print("M4gicDMX");
		}
		Serial.println(F("\t finished."));

	/************************************************/
	/** start slight_ButtonInput                   **/
	/************************************************/
		Serial.print(F("# Free RAM = "));
		Serial.println(freeRam());

		Serial.println(F("slight_ButtonInput:"));
		{
			// Serial.println(F("\t pinMode INPUT_PULLUP"));
			// // pinMode(myButtonFixture.getPin(), INPUT_PULLUP);
			// pinMode(myButtonFixture.getPin(), INPUT);
			// digitalWrite(myButtonFixture.getPin(), HIGH);
			//
			// Serial.println(F("\t myButtonFixture.begin();"));
			// myButtonFixture.begin();

			Serial.println(F("\t init ButtonInput system"));
			myButtons_init();

		}
		Serial.println(F("\t finished."));

	/************************************************/
	/** initialize fixture array                   **/
	/************************************************/

		Serial.print(F("# Free RAM = "));
		Serial.println(freeRam());

		Serial.println(F("initialize fixture array:")); {

			// Serial.println(F("\t init to 2x16"));
			// http://www.cplusplus.com/reference/cstring/memset/
			// memset(fixture_values, 0, fixture_COUNT*fader_COUNT);
			memset(fixture_values, 0, sizeof(fixture_values)/sizeof(uint8_t));
		}
		Serial.println(F("\t finished."));


	/************************************************/
	/** show Serial Commands                       **/
	/************************************************/

		// reset Serial Debug Input
		memset(sMenu_Input_New, '\0', sizeof(sMenu_Input_New)-1);
		//print Serial Options
		sMenu_Input_New[0] = '?';
		bMenu_Input_Flag_EOL = true;


	/************************************************/
	/** GO                                         **/
	/************************************************/

		Serial.println(F("Loop:"));



} /** setup **/


/****************************************************************************************************/
/** Main Loop                                                                                      **/
/****************************************************************************************************/
void loop() {

	/**************************************************/
	/** Menu Input                                   **/
	/**************************************************/
		// Serial
		handle_SerialReceive();
		check_NewLineComplete();

		// Ohter Input Things:
		//handle_EthTelnet_Server();
		//check_NewLineComplete();


	/**************************************************/
	/** handle input                                 **/
	/**************************************************/
		faderRead();
		mapFader2Fixture();

	/**************************************************/
	/** my Button                                    **/
	/**************************************************/
		// myButtonFixture.update();
		myButtons_update();

	/**************************************************/
	/** update dispay                                **/
	/**************************************************/
		displayUpdate();

	/**************************************************/
	/** send DMX                                     **/
	/**************************************************/
		dmxUpdate();

	/**************************************************/
	/** Timed things                                 **/
	/**************************************************/


		// every XXXXms
		// if ( ( millis() - ulTimeStamp_LastAction ) > cwUpdateInterval) {
		// 	ulTimeStamp_LastAction =  millis();
		// 	//
		// }



	/**************************************************/
	/** Debug Out                                    **/
	/**************************************************/

		if ( (millis() - ulDebugOut_LiveSign_TimeStamp_LastAction) > cwDebugOut_LiveSign_UpdateInterval) {
			ulDebugOut_LiveSign_TimeStamp_LastAction = millis();

			if ( bDebugOut_LiveSign_Serial_Enabled ) {
				Serial.print(millis());
				Serial.print(F("ms;"));
				Serial.print(F("  free RAM = "));
				Serial.println(freeRam());
			}

			if ( bDebugOut_LiveSign_LED_Enabled ) {
				bLEDState = ! bLEDState;
				if (bLEDState) {
					//set LED to HIGH
					digitalWrite(cbID_LED_Info, HIGH);
				} else {
					//set LED to LOW
					digitalWrite(cbID_LED_Info, LOW);
				}
			}

		}

	/**************************************************/
	/**                                              **/
	/**************************************************/

} /** loop **/


/****************************************************************************************************/
/** THE END                                                                                        **/
/****************************************************************************************************/
