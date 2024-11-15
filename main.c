
/*
 * IMP PROJECT 23/24

 * RTC BASED DIGITAL ALARM CLOCK
 * author: xpolia05
 */

/* Header file with all the essential definitions for a given type of MCU */
#include "MK60D10.h"
#include <stdio.h>

/* Macros for bit-level registers manipulation */
#define GPIO_PIN_MASK 0x1Fu
#define GPIO_PIN(x) (((1)<<(x & GPIO_PIN_MASK)))

/* Mapping of LEDs and buttons to specific port pins: */
#define LED_D9  0x20      // Port B, bit 5
#define LED_D10 0x10      // Port B, bit 4
#define LED_D11 0x8       // Port B, bit 3
#define LED_D12 0x4       // Port B, bit 2

#define BTN_SW2 0x400     // Right button, Port E, bit 10
#define BTN_SW3 0x1000    // Down button, Port E, bit 12
#define BTN_SW4 0x8000000 // Left button, Port E, bit 27
#define BTN_SW5 0x4000000 // Up button, Port E, bit 26
#define BTN_SW6 0x800     // Other button, Port E, bit 11

#define SPK 0x10          // Speaker is on PTA4

/* GLOBAL VARIABLES */
char* menuItems[] = {
	"",
    "Set Time",
    "Set Alarm Time",
    "Unset Alarm",
    "Set Repetitions Count",
    "Set Repetitions Delay",
    "Set Alarm Melody",
    "Set Alarm Lighting Mode",
    "Exit Menu"
};
int numMenuItems = sizeof(menuItems) / sizeof(menuItems[0]);
int currItem = 0;
int exitMenu = 0;

int alarmOn = 0;
int alarmRinging = 0;
int alarmRepetitionsPerformed = 0;
int setRepetitions = 0;
int setDelay = 1;
int setMelody = 1;
int setLightMode = 1;
int flashState = 0;


int pressed_up = 0, pressed_down = 0, pressed_left = 0, pressed_right = 0, pressed_action = 0;

uint32_t SecondsInDay = 86400;


/* Function implementing active waiting */
void delay(long long bound) {

  long long i;
  for(i=0;i<bound;i++);
}

void delayOneSecond(void) {
    uint32_t startTime = RTC->TSR;
    while ((RTC->TSR - startTime) < 1) {
        // do nothing
    }
}

/* Function realizing a short beep of the buzzer */
void beep(void) {
    for (uint32_t q=0; q<200; q++) {
    	GPIOA->PDOR ^= SPK; delay(400);
    	GPIOA->PDOR ^= SPK; delay(400);
    }
}

void sirenAlarm(void) {
    int lowToneDelay = 1600;
    int highToneDelay = 400;
    for (int j = 0; j < 2; j++) {

        for (int i = 0; i < 600; i++) {
            GPIOA->PDOR ^= SPK;
            delay(highToneDelay);
            GPIOA->PDOR ^= SPK;
            delay(highToneDelay);
        }

        for (int i = 0; i < 150; i++) {
            GPIOA->PDOR ^= SPK;
            delay(lowToneDelay);
            GPIOA->PDOR ^= SPK;
            delay(lowToneDelay);
        }
    }
}

void playSetMelody(){
	switch(setMelody){
		case 0:
			break;
		case 1:
		    for (uint32_t q=0; q<800; q++) {
		    	GPIOA->PDOR ^= SPK; delay(400);
		    	GPIOA->PDOR ^= SPK; delay(400);
		    }
			break;
		case 2:
			for (int i = 0; i < 4; i++){
				beep();
				delay(250000);
			}
			break;
		case 3:
			sirenAlarm();
			break;
	}
}

void flashLEDs(int led1, int led2) {
    // Turn on both LEDs
    GPIOB_PDOR ^= (led1 | led2);
    delay(240000);
    // Turn off both LEDs
    GPIOB_PDOR ^= (led1 | led2);
    delay(240000);
}

void flashOneLED(int led) {
	GPIOB_PDOR ^= (led);
	delay(120000);
	GPIOB_PDOR ^= (led);
}

void flashLightMode() {
    switch (setLightMode) {
    	case 0:
    		break;
        case 1:
        	// flash LEDs sequentially
        	flashOneLED(LED_D12);
        	flashOneLED(LED_D11);
        	flashOneLED(LED_D10);
        	flashOneLED(LED_D9);
        	break;
        case 2:
        	// Flash all LEDs at once, twice
        	for (int i = 0; i < 4; i ++){
        		flashLEDs(LED_D9 | LED_D10, LED_D11 | LED_D12);
        	}
            break;
        case 3:
            // Flash one pair of leds, then another pair, repeat 2 times
            for (int i = 0; i < 2; i++) {
                flashLEDs(LED_D9, LED_D10);
                flashLEDs(LED_D11, LED_D12);
            }
            break;
    }
}


/* Initialize the MCU - basic clock settings, turning the watchdog off */
void MCUInit(void)  {
    MCG_C4 |= ( MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x01) ); // 48 MHz
    SIM_CLKDIV1 |= SIM_CLKDIV1_OUTDIV1(0x00);
    WDOG_STCTRLH &= ~WDOG_STCTRLH_WDOGEN_MASK;
}

/* Initialize UART */
void UARTInit(void) {

    // Disable UARTx (transmitter) for configuration
    UART5->C2 &= ~(UART_C2_TE_MASK);

    UART5->BDH = 0x00;
    UART5->BDL = 0x1A;		// Baud rate 115 200 Bd, 1 stop bit
    UART5->C4 = 0x0F;		// Oversampling ratio 16, match address mode disabled

    // No parity, 8 data bits, one stop bit (default settings)
    UART5->C1 = 0x00;
    UART5->C3 = 0x00;

    UART5->MA1 = 0x00;		// no match address (mode disabled in C4)
	UART5->MA2 = 0x00;		// no match address (mode disabled in C4)

    // Enable the transmitter
    UART5->C2 |= (UART_C2_TE_MASK);

}

void PortsInit(void)
{
    /* Turn on all port clocks */
	SIM->SCGC1 |= SIM_SCGC1_UART5_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTE_MASK ;
    SIM->SCGC6 |= SIM_SCGC6_RTC_MASK;

    /* Set corresponding PTB pins (connected to LED's) for GPIO functionality */
    PORTB->PCR[5] = PORT_PCR_MUX(0x01); // D9
    PORTB->PCR[4] = PORT_PCR_MUX(0x01); // D10
    PORTB->PCR[3] = PORT_PCR_MUX(0x01); // D11
    PORTB->PCR[2] = PORT_PCR_MUX(0x01); // D12

    PORTE->PCR[10] = PORT_PCR_MUX(0x01); // SW2
    PORTE->PCR[12] = PORT_PCR_MUX(0x01); // SW3
    PORTE->PCR[27] = PORT_PCR_MUX(0x01); // SW4
    PORTE->PCR[26] = PORT_PCR_MUX(0x01); // SW5
    PORTE->PCR[11] = PORT_PCR_MUX(0x01); // SW6

    PORTA->PCR[4] = PORT_PCR_MUX(0x01);  // Speaker

    PORTE->PCR[8] = ( 0 | PORT_PCR_MUX(0x03) );
    PORTE->PCR[9] = ( 0 | PORT_PCR_MUX(0x03) );

    /* Change corresponding PTB port pins as outputs */
    PTB->PDDR = GPIO_PDDR_PDD(0x3C);     // LED ports as outputs
    PTA->PDDR = GPIO_PDDR_PDD(SPK);     // Speaker as output
    PTB->PDOR |= GPIO_PDOR_PDO(0x3C);    // turn all LEDs OFF
    PTA->PDOR &= GPIO_PDOR_PDO(~SPK);   // Speaker off
}

void RTCInit(void) {
	// Set and clear reset bit
	RTC->CR = RTC_CR_SWR_MASK;
	RTC->CR &= ~RTC_CR_SWR_MASK;

	// Enable RTC oscillator
	RTC->CR |= RTC_CR_OSCE_MASK;
	delay(0x6AD000); // stabilization delay, ~150ms( ~7,000,000 iterations @ 48MHz )

    // Disable RTC to set time
    RTC->SR &= ~RTC_SR_TCE_MASK;

    // Set RTC time
	RTC->TAR = SecondsInDay+1; // Set Time Alarm Register
	RTC->TSR = 1; // Set Time Seconds Register

    // Enable the RTC
    RTC->SR |= RTC_SR_TCE_MASK;

    NVIC_EnableIRQ(RTC_IRQn); // Nested Vector Interrupt Controller used for alarm(triggers interrupt)
}

void RTCSetTime(uint32_t seconds) {
	// Disable RTC to set time
    RTC_SR &= ~RTC_SR_TCE_MASK;

    // Set RTC time
    RTC->TSR = seconds;

    // Enable the RTC
    RTC_SR |= RTC_SR_TCE_MASK;
}

/* sending one character (ch) via UART - the function waits until the transmit buffer is empty, then sends */
void SendCh(char ch)  {
    while(!(UART5->S1 & UART_S1_TDRE_MASK) && !(UART5->S1 & UART_S1_TC_MASK) );
    UART5->D = ch;
}

/* transmission of a string terminated by 0 */
void SendStr(char *s)  {
	int i = 0;
	while (s[i]!=0) {
		SendCh(s[i++]);
	}
}

void clearScreen() {
    SendStr("\033[2J"); // Clears the entire screen
    SendStr("\033[H");  // Moves the cursor to the home position (top left corner)
}

void handleSetupButtonPress(int *part, int *hours, int *minutes, int *seconds) {

    while (1) {
        // Check for up button press
        if (!pressed_up && !(GPIOE_PDIR & BTN_SW5)) {
            pressed_up = 1;
            if (*part == 0) {
                *hours = (*hours + 1) % 24;
            } else if (*part == 1) {
                *minutes = (*minutes + 1) % 60;
            } else if (*part == 2) {
                *seconds = (*seconds + 1) % 60;
            }
            break;
        } else if (GPIOE_PDIR & BTN_SW5) pressed_up = 0;

        // Check for down button press
        if (!pressed_down && !(GPIOE_PDIR & BTN_SW3)) {
            pressed_down = 1;
            if (*part == 0) {
                *hours = (*hours - 1 + 24) % 24;
            } else if (*part == 1) {
                *minutes = (*minutes - 1 + 60) % 60;
            } else if (*part == 2) {
                *seconds = (*seconds - 1 + 60) % 60;
            }
            break;
        } else if (GPIOE_PDIR & BTN_SW3) pressed_down = 0;

        // Check for action button press
        if (!pressed_action && !(GPIOE_PDIR & BTN_SW6)) {
            pressed_action = 1;
            *part = (*part + 1);
            break;
        } else if (GPIOE_PDIR & BTN_SW6) pressed_action = 0;

        // Left button, beep to signalize wwrong input
        if (!pressed_left && !(GPIOE_PDIR & BTN_SW4)) {
        	beep();
        } else if (GPIOE_PDIR & BTN_SW4) pressed_left = 0;

        // Right button, beep to signalize wwrong input
        if (!pressed_right && !(GPIOE_PDIR & BTN_SW2)) {
			beep();
        } else if (GPIOE_PDIR & BTN_SW2) pressed_right = 0;

        delay(250000);
    }
    delay(500000);
}

void TimeInit(void) {
    int hours = 0, minutes = 0, seconds = 0;
    int part = 0; // 0 for hours, 1 for minutes, 2 for seconds

    char buffer[4];

    while (1) {
        clearScreen();
        SendStr("\033[1;32m\rWelcome to ARM@FITkit3 RTC Alarm Clock\r\n\n\033[0m");
        SendStr("\rUP(SW5)\033[90m, \033[0mDOWN(SW3)\033[90m to cycle time\r\n\033[0m");
        SendStr("\rACTION(SW6)\033[90m to confirm selection\r\n\n\033[0m");



        if (part == 0) {
        	SendStr("\033[38;5;208m\rSet Hours: \033[0m");

        } else if (part == 1) {
        	SendStr("\033[38;5;208m\rSet Minutes: \033[0m");

        } else if (part == 2) {
        	SendStr("\033[38;5;208m\rSet Seconds: \033[0m");

        }

        char buffer[4];
        sprintf(buffer, "%02d", (part == 0) ? hours : (part == 1) ? minutes : seconds);
        SendStr(buffer);

        handleSetupButtonPress(&part, &hours, &minutes, &seconds);

        if (part > 2) {
            break; // Exit loop after setting seconds
        }

    }
    // Set RTC Time with the configured time
    uint32_t totalSeconds = hours * 3600 + minutes * 60 + seconds;
    RTCSetTime(totalSeconds);

    clearScreen();
}

void handleAlarmButtonPress(int *part, int *hours, int *minutes) {

    while (1) {
        // Check for up(SW5) button press
        if (!pressed_up && !(GPIOE_PDIR & BTN_SW5)) {
            pressed_up = 1;
            if (*part == 0) {
                *hours = (*hours + 1) % 24;
            } else {
                *minutes = (*minutes + 1) % 60;
            }
            break;
        } else if (GPIOE_PDIR & BTN_SW5) pressed_up = 0;

        // Check for down(SW3) button press
        if (!pressed_down && !(GPIOE_PDIR & BTN_SW3)) {
            pressed_down = 1;
            if (*part == 0) {
                *hours = (*hours - 1 + 24) % 24;
            } else {
                *minutes = (*minutes - 1 + 60) % 60;
            }
            break;
        } else if (GPIOE_PDIR & BTN_SW3) pressed_down = 0;

        // Check for action(SW6) button press
        if (!pressed_action && !(GPIOE_PDIR & BTN_SW6)) {
            pressed_action = 1;
            *part = (*part + 1);
            break;
        } else if (GPIOE_PDIR & BTN_SW6) pressed_action = 0;

        // Left button, beep to signalize wwrong input
        if (!pressed_left && !(GPIOE_PDIR & BTN_SW4)) {
        	beep();
        } else if (GPIOE_PDIR & BTN_SW4) pressed_left = 0;

        // Right button, beep to signalize wrong input
        if (!pressed_right && !(GPIOE_PDIR & BTN_SW2)) {
			beep();
        } else if (GPIOE_PDIR & BTN_SW2) pressed_right = 0;

        delay(250000);
    }
    delay(500000);
}

void AlarmInit(void) {
	uint32_t currentSeconds = RTC->TSR; // grab current time to set it as initial values (+1 minute)
	int hours = currentSeconds / 3600;
	int minutes = ((currentSeconds % 3600) / 60) + 1;
	if (minutes > 59) {
		minutes = 0;
		hours = (hours == 23) ? 0 : hours++;
	}
    int part = 0; // 0 for hours, 1 for minutes

    char buffer[4];

    while (1) {
        clearScreen();
        SendStr("\033[1;32m\rSet Alarm Time\r\n\n\033[0m");
        SendStr("\rUP(SW5)\033[90m, \033[0mDOWN(SW3)\033[90m to cycle time\r\n\033[0m");
        SendStr("\rACTION(SW6)\033[90m to confirm selection\r\n\n\033[0m");

        if (part == 0) {
        	SendStr("\033[38;5;208m\rSet Hours: \033[0m");

        } else {
        	SendStr("\033[38;5;208m\rSet Minutes: \033[0m");

        }

        char buffer[4];
        sprintf(buffer, "%02d", (part == 0) ? hours : minutes);
        SendStr(buffer);

        handleAlarmButtonPress(&part, &hours, &minutes);

        if (part > 1) {
            break; // Exit loop after setting minutes
        }

    }
    // Set RTC Alarm Time with the configured time
    uint32_t alarmInSeconds = hours * 3600 + minutes * 60;
    RTC->TAR = alarmInSeconds;
    alarmOn = 1;
}

void displayCurrentTime(void) {
	if (RTC->TSR >= 86400) { // rollover check
		RTCSetTime(0);
	}

    int hours;
    int minutes;
    int seconds;

    uint32_t totalSeconds = RTC->TSR;
    if (totalSeconds == 86400) {
        hours = 0;
        minutes = 0;
        seconds = 0;
    } else {
        hours = totalSeconds / 3600;
        minutes = (totalSeconds % 3600) / 60;
        seconds = totalSeconds % 60;
    }


    char timeString[10]; // "HH:MM:SS" + null terminator = 10
    sprintf(timeString, "%02d:%02d:%02d", hours, minutes, seconds);


    clearScreen();

    if (alarmRinging) {
    	flashState = !flashState;
    	SendStr(flashState ? "\033[7m" : "\033[0m");
    } else {
    	SendStr("\033[0m");
    }

    SendStr(timeString);

    if (alarmOn) {
    	SendStr("\033[33m \U0001F514"); // bell unicode
    	if (!setMelody) {SendStr("\033[91m \U0001F507");} // crossed speaker unicode
    	SendStr("\033[0m");
    }

    SendStr("\r\n");

    if (alarmRinging) {
    	SendStr("\033[90mhold DOWN(SW3) to ");
    	if (setRepetitions > 0 && alarmRepetitionsPerformed < setRepetitions) {
    		SendStr("snooze\033[0m\r\n");
    	} else {
    		SendStr("turn alarm off\033[0m\r\n");
    	}
    }
}

void drawMenu(void) {

    char selectCount[2];
    char alarmTimeStr[20];

    exitMenu = 0;

    while (!exitMenu) {
        clearScreen();
        displayCurrentTime(); // Display current time
        SendStr("\033[1;32m######## MENU ########\033[0m\r\n");
        for (int i = 1; i < numMenuItems; i++) {
            if (i == currItem) {
                SendStr("\033[33m"); // yellow text for selected item
            } else {
            	SendStr("\033[0m");
            }

            if (i == 2 && alarmOn) { // Replaces "Set Alarm Time" with "Alarm set: HH:MM" when alarm is on
                        uint32_t alarmTime = RTC->TAR;
                        int alarmHours = alarmTime / 3600;
                        int alarmMinutes = (alarmTime % 3600) / 60;
                        sprintf(alarmTimeStr, "Alarm set: %02d:%02d", alarmHours, alarmMinutes);
                        SendStr(alarmTimeStr);
            } else {
            	SendStr(menuItems[i]);
            }

            switch (i) {
                case 4: // "Set Repetitions Count"
                	if (setRepetitions != 0) {
                		sprintf(selectCount, ": %d", setRepetitions);
                		SendStr(selectCount);
                	} else {
                		SendStr(": OFF");
                	}
                    break;
                case 5: // "Set Repetitions Delay"
                    sprintf(selectCount, ": %d min", setDelay);
                    SendStr(selectCount);
                	break;
                case 6: // "Set Alarm Melody"
                	if (setMelody != 0) {
                		sprintf(selectCount, ": %d", setMelody);
                		SendStr(selectCount);
                	} else {
                		SendStr(": OFF");
                	}
                    break;
                case 7: // "Set Alarm Lighting Mode"
                	if (setLightMode != 0) {
                		sprintf(selectCount, ": %d", setLightMode);
                		SendStr(selectCount);
                	} else {
                		SendStr(": OFF");
                	}
                    break;
            }

            SendStr("\r\n");
        }
        SendStr("\033[1;32m######################\033[0m");

        // Check for up(SW5) button press for menu navigation
        if (!pressed_up && !(GPIOE_PDIR & BTN_SW5)) {
            pressed_up = 1;
            currItem = (currItem <= 1) ? 8 : currItem - 1;
        } else if (GPIOE_PDIR & BTN_SW5) pressed_up = 0;

        // Check for down(SW3) button press
        if (!pressed_down && !(GPIOE_PDIR & BTN_SW3)) {
            pressed_down = 1;
            currItem = (currItem >= 8) ? 1 : currItem + 1;
        } else if (GPIOE_PDIR & BTN_SW3) pressed_down = 0;

        // Check for left(SW4) button press
		if (!pressed_left && !(GPIOE_PDIR & BTN_SW4)) {
			pressed_left = 1;
            switch (currItem) {
            	case 1:
            	case 2:
            	case 3:
            	case 8:
            		beep();
            		break;
                case 4: // "Set Repetitions Count"
                    setRepetitions = (setRepetitions <= 0) ? 5 : setRepetitions - 1;
                    break;
                case 5: // "Set Repetitions Delay"
                    if (setDelay >= 1 && setDelay <= 5) {
                        setDelay = (setDelay - 1 == 0) ? 60 : setDelay - 1;
                    } else if (setDelay > 5) {
                        setDelay = setDelay - 5;
                    }
                    break;
                case 6: // "Set Alarm Melody"
                    setMelody = (setMelody == 0) ? 3 : setMelody - 1;
                    break;
                case 7: // "Set Alarm Lighting Mode"
                    setLightMode = (setLightMode == 0) ? 3 : setLightMode - 1;
                    break;
            }
		} else if (GPIOE_PDIR & BTN_SW4) pressed_left = 0;

		// Check for right(SW2) button press
		if (!pressed_right && !(GPIOE_PDIR & BTN_SW2)) {
			pressed_right = 1;
            switch (currItem) {
				case 1:
				case 2:
				case 3:
				case 8:
					beep();
					break;
                case 4: // "Set Repetitions Count"
                    setRepetitions = (setRepetitions >= 5) ? 0 : setRepetitions + 1;
                    break;
                case 5: // "Set Repetitions Delay"
                    if (setDelay >= 1 && setDelay < 5) {
                        setDelay += 1;
                    } else if (setDelay >= 5) {
                        setDelay = (setDelay + 5 >= 60) ? 1 : setDelay + 5;
                    }
                	break;
                case 6: // "Set Alarm Melody"
                    setMelody = (setMelody == 3) ? 0 : setMelody + 1;
                    break;
                case 7: // "Set Alarm Lighting Mode"
                    setLightMode = (setLightMode == 3) ? 0 : setLightMode + 1;
                    break;
            }
		} else if (GPIOE_PDIR & BTN_SW2) pressed_right = 0;

        // Check for action(SW6) button press
        if (!pressed_action && !(GPIOE_PDIR & BTN_SW6)) {
            pressed_action = 1;

            switch (currItem) {
            	case 0: // exists purely to prevent accidental selecting of first item 'set time' when opening menu
            		break;
                case 1: // "Set Time"
                	delay(500000); // ~10ms (rebounce)
                	RTCSetTime(0); // prevents bug if someone wants to set time closely before midnigt
                    TimeInit();
                    break;
                case 2: // "Set Alarm Time"
                	if (!alarmOn){
                		delay(500000); // ~10ms (rebounce)
                		AlarmInit();
                	} else {
                		beep();
                	}
                    break;
                case 3: // "Unset Alarm"
                	alarmOn = 0;
                	if (RTC->TSR == 0){
                		delayOneSecond();
                	}
                	RTC->TAR = SecondsInDay+1;
                    break;
                case 4: // "Set Repetitions Count"
                	break;
                case 5: // "Set Repetitions Delay"
                	break;
                case 6: // "Set Alarm Melody"
                	playSetMelody();
                	break;
                case 7: // "Set Alarm Lighting Mode"
                	flashLightMode();
                	break;
                case 8: // "Exit Menu"
                    exitMenu = 1;
                    currItem = 0;
                    break;
            }
        } else if (GPIOE_PDIR & BTN_SW6) pressed_action = 0;

        delay(500000); // refresh screen ratio
    }
}

void playAlarm(void) {
    alarmRinging = 1;
    while (alarmRinging) {
    	displayCurrentTime();
        // Play sound and light mode
        playSetMelody();
        flashLightMode();
        if (setMelody == 0 || setLightMode == 0){
        	delayOneSecond();
        }

        // Check for user input to stop or snooze the alarm
        if (!pressed_down && !(GPIOE_PDIR & BTN_SW3)) {
            alarmRinging = 0;
            alarmOn = (setRepetitions > 0) ? 1 : 0; // Turn off alarm if no more repetitions
            delay(500000); // Debounce delay
        } else if (GPIOE_PDIR & BTN_SW6) pressed_down = 0;
    }
}

void RTC_IRQHandler(void) {
    if (((RTC->SR & RTC_SR_TAF_MASK) == RTC_SR_TAF_MASK) && alarmOn) {
        exitMenu = 1;
        currItem = 0;
        if (setRepetitions > 0 && alarmRepetitionsPerformed < setRepetitions) {
            // Perform the alarm
            playAlarm();

            // Set the next alarm time(snooze)
            uint32_t nextAlarmTime = RTC->TSR + (setDelay * 60);
            RTC->TAR = nextAlarmTime;

            alarmRepetitionsPerformed++;
        } else {
            // Last repetition or no repetitions
            playAlarm();
            alarmOn = 0; // Turn off the alarm
            alarmRepetitionsPerformed = 0;
            RTC->TAR = SecondsInDay+1; // Reset the alarm time register
        }
    }
}

void initSystem(void) {
	MCUInit();
	PortsInit();
	UARTInit();
	RTCInit();
	TimeInit(); // User initial time setup
}

int main(void) {
	initSystem();

    while (1) {
        if (!(GPIOE_PDIR & BTN_SW6)) { // Check if SW6(action) is pressed to open menu
            drawMenu();
            delay(500000); // ~10ms (rebounce)
        } else {
            displayCurrentTime();
            delayOneSecond();
        }
    }

    return 0;
}
