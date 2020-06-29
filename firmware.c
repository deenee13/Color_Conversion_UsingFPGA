/**
*
* @file ece544ip_test.c
*
* @author Deepen Parmar (parmar@pdx.edu)

*
* This file reads the value of the Switches, Pushbuttons and the rotary encoder and
* change the value of the HSV which is getting displayed on the PMOD RGB which is
* also displaying the Color wheel also, it also displays the calculated and the
* detected duty cycle of the RGB channel on the Seven Segment
*

******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "platform.h"
#include "xparameters.h"
#include "xstatus.h"
#include "microblaze_sleep.h"
#include "nexys4IO.h"
#include "PmodOLEDrgb.h"
#include "PmodENC.h"
#include "xgpio.h"
#include "xintc.h"
#include "xtmrctr.h"

/************************** Constant Definitions ****************************/

// Clock frequencies
#define CPU_CLOCK_FREQ_HZ		XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ		XPAR_CPU_M_AXI_DP_FREQ_HZ

// AXI timer parameters
#define AXI_TIMER_DEVICE_ID		XPAR_AXI_TIMER_0_DEVICE_ID
#define AXI_TIMER_BASEADDR		XPAR_AXI_TIMER_0_BASEADDR
#define AXI_TIMER_HIGHADDR		XPAR_AXI_TIMER_0_HIGHADDR
#define TmrCtrNumber			0


// Definitions for peripheral NEXYS4IO
#define NX4IO_DEVICE_ID		XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR		XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR		XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

// Definitions for peripheral PMODOLEDRGB
#define RGBDSPLY_DEVICE_ID		XPAR_PMODOLEDRGB_0_DEVICE_ID
#define RGBDSPLY_GPIO_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_BASEADDR
#define RGBDSPLY_GPIO_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_HIGHADD
#define RGBDSPLY_SPI_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_BASEADDR
#define RGBDSPLY_SPI_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_HIGHADDR

// Definitions for peripheral PMODENC
#define PMODENC_DEVICE_ID		XPAR_PMODENC_0_DEVICE_ID
#define PMODENC_BASEADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_BASEADDR
#define PMODENC_HIGHADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_HIGHADDR

// Fixed Interval timer - 100 MHz input clock, 40KHz output clock
// FIT_COUNT_1MSEC = FIT_CLOCK_FREQ_HZ * .001
#define FIT_IN_CLOCK_FREQ_HZ	CPU_CLOCK_FREQ_HZ
#define FIT_CLOCK_FREQ_HZ		40000
#define FIT_COUNT				(FIT_IN_CLOCK_FREQ_HZ / FIT_CLOCK_FREQ_HZ)
#define FIT_COUNT_1MSEC			40

// GPIO parameters
#define GPIO_0_DEVICE_ID			XPAR_AXI_GPIO_0_DEVICE_ID
#define GPIO_0_INPUT_0_CHANNEL		1
#define GPIO_0_INPUT_1_CHANNEL		2

// Interrupt Controller parameters
#define INTC_DEVICE_ID			XPAR_INTC_0_DEVICE_ID
#define FIT_INTERRUPT_ID		XPAR_MICROBLAZE_0_AXI_INTC_FIT_TIMER_0_INTERRUPT_INTR


// Function Parameters
#define HB_MASK	(0x0F00)
#define MASK_RGB_SW	(0x3)
#define GET_RGB_SW_VAL(val)	((val >> 2) & MASK_RGB_SW)


/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/

/************************** Variable Definitions ****************************/
// Microblaze peripheral instances
uint64_t 	timestamp = 0L;
PmodOLEDrgb	pmodOLEDrgb_inst;
PmodENC 	pmodENC_inst;
XGpio		GPIOInst0;					// GPIO instance
XIntc 		IntrptCtlrInst;				// Interrupt Controller instance
XTmrCtr		AXITimerInst;				// PWM timer instance


// The following variables are shared between non-interrupt processing and
// interrupt processing such that they must be global(and declared volatile)
// These variables are controlled by the FIT timer interrupt handler
volatile uint32_t			gpio_in;
volatile uint32_t			gpio_in_1;

enum color_mask {
	PWM_GREEN_MASK = 0x1,
	PWM_BLUE_MASK = 0x2,
	PWM_RED_MASK = 0x4,
};
enum color_mask g_mask = PWM_RED_MASK;
uint32_t g_duty_cycle, previous_state;
uint32_t on, off ;

// These variable are used in hsv_rgb_conversion for calculation purpose
u8 R, G, B;
unsigned char bcd[10];
unsigned char bcd1[10];
uint32_t g_counter = 0;
uint32_t sw_value;


/************************** Function Prototypes *****************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix);
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num);
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix);
int	 do_init(void);											// initialize system
void FIT_Handler(void);										// fixed interval timer interrupt handler
int AXI_Timer_initialize(void);

void PMDIO_counter( int *num  );
void seven_seg_counter( uint8_t *num  );
void LEDrgb_BuildHSV(u8 hue, u8 sat, u8 val);
void hsv_rgb_conversion(void);




/************************** MAIN-PROGRAM ************************************/
int main(void)
{
    init_platform();
    uint32_t sts;


    // Initializing Timer and Interrupt Handler
	sts = do_init();

	if (XST_SUCCESS != sts)
	{
		exit(1);
	}

	microblaze_enable_interrupts();

	// Function call
	hsv_rgb_conversion();



	// announce that we're done
	xil_printf("\nThat's All Folks!\n\n\r");

	// Displaying
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 2);
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildRGB(0, 255, 255));
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"BYE BYE");

	usleep(5000 * 1000);

	// clear the displays and power down the pmodOLEDrbg
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_B, CC_LCY, CC_E, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGLO, CC_B, CC_LCY, CC_E, CC_BLANK, DP_NONE);
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_end(&pmodOLEDrgb_inst);

	// cleanup and exit
    cleanup_platform();
    exit(0);
}







/****************************************************************************/
/**
* hsv_rgb_conversion- This function monitors the value of Switch, Pushbutton and
* Rotary Encoder and depending upon values of them implements the specific functionality
*
*  @param	*NONE*
*
* @return	*NONE*
*
*****************************************************************************/
void hsv_rgb_conversion(void)
{
	uint32_t state, laststate;

	int ticks_hue = 0, lastticks_hue = 1;
	int ticks_sat = 0, lastticks_sat = 1;
	int ticks_value = 0, lastticks_value = 1;


	// Temporary storage variable
	uint8_t temp_r;
	uint8_t temp_g;
	uint8_t temp_b;

	uint32_t red_value;
	uint32_t blue_value;
	uint32_t green_value;
	uint32_t counter_hb;

	// Heart beat counter
	counter_hb = 0;

	char s[] = " End Test";

	xil_printf("Starting Project\n\r");


	LEDrgb_BuildHSV(0, 0, 0);

	// Set up the display output
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst,OLEDrgb_BuildRGB(0, 0, 255));

	// Passing the Cursor position and the Text to display at that position
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 1);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"H:");

	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 3);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"S:");

	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 5);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"V:");

	// get the previous state of PmodEnc
	laststate = ENC_getState(&pmodENC_inst);



	while(1)
	{

		//get the PmodENC state;
		state = ENC_getState(&pmodENC_inst);


		// Switch logic for selecting the particular switch
		enum sw_rgb_setting {
			switch_red = 0,
			switch_green = 1,
			switch_blue = 2,
			switch_reserved = 3,
		};
		 sw_value = NX4IO_getSwitches();
		enum sw_rgb_setting rgb_sw_setting = GET_RGB_SW_VAL(sw_value);


		// Implement Blinking of the LED's when entering while loop
		if ((counter_hb % 100) == 0)
		{
			NX4IO_setLEDs(NX4IO_getLEDS_DATA() ^ HB_MASK);
		}
		counter_hb++;



		// Reading the Two GPIO Channel
		gpio_in_1 = XGpio_DiscreteRead(&GPIOInst0, GPIO_0_INPUT_1_CHANNEL);
		gpio_in = XGpio_DiscreteRead(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL);

		// Displaying the value of the GPIO Channel
		xil_printf("value at gpio_HW: 0x%08x\n",gpio_in_1);
		xil_printf("value at gpio_SW: 0x%08x\n",gpio_in);

		// check if the rotary encoder pushbutton is pressed
		// exit the loop if either one is pressed.
		if (ENC_buttonPressed(state) && !ENC_buttonPressed(laststate))//only check on button posedge
		{
			break;
		}


		// Check Current State of rotary encoder with Previous State
		if (state != laststate)
		{
			ticks_hue += ENC_getRotation(state, laststate);
		}
		else
		{
			ticks_hue += ENC_getRotation(state, laststate);
		}



		//  Reading the Buttons values and Incrementing the Counter to Display HSV Value
		if (NX4IO_isPressed(BTNU))
		{
			ticks_value += 1;
		}
		else if(NX4IO_isPressed(BTND))
		{
			ticks_value = ticks_value - 1;
		}
		else if(NX4IO_isPressed(BTNR))
		{
			ticks_sat += 1;
		}
		else if(NX4IO_isPressed(BTNL))
		{
			ticks_sat = ticks_sat - 1;
		}

		// Checking if the respected Counter is Incremented, If incremented display the new value
		if(ticks_hue != lastticks_hue || ticks_value != lastticks_value || ticks_sat != lastticks_sat )
				{
						// Updating the Value of HUE
						OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 1);
						OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
						OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 1);
						PMDIO_counter(&ticks_hue);
						PMDIO_putnum(&pmodOLEDrgb_inst, ticks_hue, 10);


						// Updating the Value of the Brightness
						usleep(20*1500);
						OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 5);
						OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
						OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 5);
						PMDIO_counter(&ticks_value);
						PMDIO_putnum(&pmodOLEDrgb_inst, ticks_value, 10);

						// Updating the Value of the Saturation
						usleep(20*1500);
						OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 3);
						OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
						OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 3);
						PMDIO_counter(&ticks_sat);
						PMDIO_putnum(&pmodOLEDrgb_inst, ticks_sat, 10);

						// Drawing Rectangle and passing HSV value to it
						OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,45,5,85,50,OLEDrgb_BuildRGB(0, 255, 255),true,OLEDrgb_BuildHSV( ticks_hue, ticks_sat, ticks_value));
						usleep(1000);

						// Passing the HSV value to the RGB LED-1
						NX4IO_RGBLED_setChnlEn(RGB1, true, true, true);
						LEDrgb_BuildHSV(ticks_hue, ticks_sat, ticks_value);

				}

		// Update the Previous Values of PMODENC, Hue, Saturation, Value
		lastticks_hue = ticks_hue;
		lastticks_sat = ticks_sat;
		lastticks_value = ticks_value;
		laststate = state;



		/*
		 * 		SW 3		SW 2		Output
		 * 		0			0			RED
		 * 		0			1			GREEN
		 * 		1			0			BLUE
		 * 		1			1			RESERVED
		 *
		 */



		// Bit 0 if 1 it means we are in HW mode for PWM detection
		if(sw_value & 0x0001)
		{
			// Write H on the Seven Segment Display
			NX4IO_SSEG_setDigit(SSEGHI,DIGIT3,CC_UCH );
			NX4IO_setLEDs((NX4IO_getLEDS_DATA() & 0xFFF0) | 0x0001);

			// Hardware PWM Detection is on
			switch (rgb_sw_setting)
			{
				case switch_red:
					g_mask = PWM_RED_MASK;

					// Read the value from the GPIO
					red_value = ((gpio_in_1 >> 16) & 0xFF);

					// Logic to display value on the Seven Segment
					red_value = red_value * 100;
					temp_r = ((R*100)/(255));
					seven_seg_counter(&temp_r);
					red_value = red_value + temp_r;
					bin2bcd(red_value, &bcd );

					// Display Value on the Seven Segment
					NX410_SSEG_setAllDigits(SSEGLO, bcd[6], bcd[7] ,bcd[8] , bcd[9], DP_NONE);
					xil_printf("duty cycle of red= %u\n", g_duty_cycle);
					break;

				case switch_green:

					g_mask = PWM_GREEN_MASK;
					NX4IO_setLEDs((NX4IO_getLEDS_DATA() & 0xFFF0) | 0x0005);

					// Read the value from the GPIO
					green_value =((gpio_in_1 >> 0) & 0xFF);

					// Logic to display value on the Seven Segment
					green_value = green_value * 100;
					temp_g = ((G*100)/(255));
					seven_seg_counter(&temp_g);
					green_value = green_value + temp_g;
					bin2bcd(green_value, &bcd );

					// Display Value on the Seven Segment
					NX410_SSEG_setAllDigits(SSEGLO, bcd[6], bcd[7] ,bcd[8] , bcd[9], DP_NONE);
					xil_printf("duty cycle of green= %u\n", g_duty_cycle);
					break;

				case switch_blue:

					g_mask = PWM_BLUE_MASK;
					NX4IO_setLEDs((NX4IO_getLEDS_DATA() & 0xFFF0) | 0x0009);

					// Read the value from the GPIO
					blue_value =((gpio_in_1 >> 8) & 0xFF);

					// Logic to display value on the Seven Segment
					blue_value = blue_value * 100;
					temp_b = ((B*100)/(255));
					seven_seg_counter(&temp_b);
					blue_value = blue_value + temp_b;
					bin2bcd(blue_value, &bcd );

					// Display Value on the Seven Segment
					NX410_SSEG_setAllDigits(SSEGLO, bcd[6], bcd[7] ,bcd[8] , bcd[9], DP_NONE);
					xil_printf("duty cycle of blue = %u\n", g_duty_cycle);
					break;
				default:
					break;
			} // Exit the Hardware loop
		}
		else
		{
			NX4IO_SSEG_setDigit(SSEGHI,DIGIT3,CC_5 );
			NX4IO_setLEDs((NX4IO_getLEDS_DATA() & 0xFFF0));
			// Software PWM Detection is on
			switch (rgb_sw_setting)
			{
				case switch_red:
					g_mask = PWM_RED_MASK;

					// Logic to display value on the Seven Segment
					temp_r = ((R*100)/(255));
					seven_seg_counter(&temp_r);
					bin2bcd(g_duty_cycle, &bcd );
					bin2bcd(temp_r, &bcd1 );

					// Display Value on the Seven Segment
					NX410_SSEG_setAllDigits(SSEGLO, bcd[8], bcd[9] ,bcd1[8] , bcd1[9], DP_NONE);
					xil_printf("duty cycle of red= %u\n", g_duty_cycle);
					break;

				case switch_green:
					g_mask = PWM_GREEN_MASK;
					NX4IO_setLEDs((NX4IO_getLEDS_DATA() & 0xFFF0) | 0x0004);

					// Logic to display value on the Seven Segment
					temp_g = ((G*100)/(255));
					seven_seg_counter(&temp_g);
					bin2bcd(g_duty_cycle, &bcd );
					bin2bcd(temp_g, &bcd1 );

					// Display Value on the Seven Segment
					NX410_SSEG_setAllDigits(SSEGLO, bcd[8], bcd[9] ,bcd1[8] , bcd1[9], DP_NONE);
					xil_printf("duty cycle of green= %u\n", g_duty_cycle);
					break;

				case switch_blue:
					g_mask = PWM_BLUE_MASK;
					NX4IO_setLEDs((NX4IO_getLEDS_DATA() & 0xFFF0) | 0x0008);

					// Logic to display value on the Seven Segment
					temp_b = ((B*100)/(255));
					seven_seg_counter(&temp_b);
					bin2bcd(g_duty_cycle, &bcd );
					bin2bcd(temp_b, &bcd1 );

					// Display Value on the Seven Segment
					bin2bcd(g_duty_cycle, &bcd );
					NX410_SSEG_setAllDigits(SSEGLO, bcd[8], bcd[9] ,bcd1[8] , bcd1[9], DP_NONE);
					xil_printf("duty cycle of blue = %u\n", g_duty_cycle);
			}

		} // Exit the software loop

	} // rotary button has been pressed - exit the While loop



	// Write one final string
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 4);
	OLEDrgb_PutString(&pmodOLEDrgb_inst, s);

	return;
}

/**************************** HELPER FUNCTIONS ******************************/

/****************************************************************************/
/**
* initialize the system
*
* This function is executed once at start-up and after resets.  It initializes
* the peripherals and registers the interrupt handler(s)
*****************************************************************************/

int	 do_init(void)
{
	uint32_t status;				// status from Xilinx Lib calls

	// initialize the Nexys4 driver and (some of)the devices
	status = (uint32_t) NX4IO_initialize(NX4IO_BASEADDR);
	if (status != XST_SUCCESS)
	{
		//NX4IO_setLEDs(0x00005555);
		return XST_FAILURE;
	}

	// set all of the display digits to blanks and turn off
	// the decimal points using the "raw" set functions.
	// These registers are formatted according to the spec
	// and should remain unchanged when written to Nexys4IO...
	// something else to check w/ the debugger when we bring the
	// drivers up for the first time
	NX410_SSEG_setAllDigits(SSEGHI,CC_BLANK,CC_BLANK ,CC_BLANK , CC_BLANK, DP_NONE);


	OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);

	// initialize the pmodENC and hardware
	ENC_begin(&pmodENC_inst, PMODENC_BASEADDR);

	// initialize the GPIO instances
	status = XGpio_Initialize(&GPIOInst0, GPIO_0_DEVICE_ID);

	if (status != XST_SUCCESS)
	{

		return XST_FAILURE;
	}
	// GPIO0 channel 1 is an 8-bit input port.
	// GPIO0 channel 2 is an 8-bit output port.
	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL, 0xFF);
	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_INPUT_1_CHANNEL, 0x5555aaaa);//XGpio_SetDataDirection(&GPIOInst0, GPIO_0_INPUT_1_CHANNEL, 0x00);  // Debug


	status = AXI_Timer_initialize();
	if (status != XST_SUCCESS)
	{

		return XST_FAILURE;
	}

	// initialize the interrupt controller
	status = XIntc_Initialize(&IntrptCtlrInst, INTC_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
	   return XST_FAILURE;
	}

	// connect the fixed interval timer (FIT) handler to the interrupt
	status = XIntc_Connect(&IntrptCtlrInst, FIT_INTERRUPT_ID,
						   (XInterruptHandler)FIT_Handler,
						   (void *)0);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;

	}

	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts.
	status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// enable the FIT interrupt
	XIntc_Enable(&IntrptCtlrInst, FIT_INTERRUPT_ID);
	return XST_SUCCESS;
}
/*
 * AXI timer initializes it to generate out a 4Khz signal, Which is given to the Nexys4IO module as clock input.
 * DO NOT MODIFY
 */
int AXI_Timer_initialize(void){

	uint32_t status;				// status from Xilinx Lib calls
	uint32_t		ctlsts;		// control/status register or mask

	status = XTmrCtr_Initialize(&AXITimerInst,AXI_TIMER_DEVICE_ID);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	status = XTmrCtr_SelfTest(&AXITimerInst, TmrCtrNumber);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	ctlsts = XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_EXT_GENERATE_MASK | XTC_CSR_LOAD_MASK |XTC_CSR_DOWN_COUNT_MASK ;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber,ctlsts);

	//Set the value that is loaded into the timer counter and cause it to be loaded into the timer counter
	XTmrCtr_SetLoadReg(AXI_TIMER_BASEADDR, TmrCtrNumber, 24998);
	XTmrCtr_LoadTimerCounterReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts &= (~XTC_CSR_LOAD_MASK);
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts |= XTC_CSR_ENABLE_TMR_MASK;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	XTmrCtr_Enable(AXI_TIMER_BASEADDR, TmrCtrNumber);
	return XST_SUCCESS;

}

/*********************** DISPLAY and RGBLED -RELATED FUNCTIONS ***********************************/

/****************************************************************************/
/**
* Converts an integer to ASCII characters
*
* algorithm borrowed from ReactOS system libraries
*
* Converts an integer to ASCII in the specified base.  Assumes string[] is
* long enough to hold the result plus the terminating null
*
* @param 	value is the integer to convert
* @param 	*string is a pointer to a buffer large enough to hold the converted number plus
*  			the terminating null
* @param	radix is the base to use in conversion,
*
* @return  *NONE*
*
* @note
* No size check is done on the return string size.  Make sure you leave room
* for the full string plus the terminating null in string
*****************************************************************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix)
{
	char tmp[33];
	char *tp = tmp;
	int32_t i;
	uint32_t v;
	int32_t  sign;
	char *sp;

	if (radix > 36 || radix <= 1)
	{
		return;
	}

	sign = ((10 == radix) && (value < 0));
	if (sign)
	{
		v = -value;
	}
	else
	{
		v = (uint32_t) value;
	}

  	while (v || tp == tmp)
  	{
		i = v % radix;
		v = v / radix;
		if (i < 10)
		{
			*tp++ = i+'0' ;
		}
		else
		{
			*tp++ = i + 'a' - 10;
		}
	}
	sp = string;

	if (sign)
		*sp++ = '-';

	while (tp > tmp)
		*sp++ = *--tp;
	*sp = 0;

  	return;
}


/****************************************************************************/
/**
* Write a 32-bit unsigned hex number to PmodOLEDrgb in Hex
*
* Writes  32-bit unsigned number to the pmodOLEDrgb display starting at the current
* cursor position.
*
* @param num is the number to display as a hex value
*
* @return  *NONE*
*
* @note
* No size checking is done to make sure the string will fit into a single line,
* or the entire display, for that matter.  Watch your string sizes.
*****************************************************************************/
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num)
{
  char  buf[9];
  int32_t   cnt;
  char  *ptr;
  int32_t  digit;

  ptr = buf;
  for (cnt = 7; cnt >= 0; cnt--) {
    digit = (num >> (cnt * 4)) & 0xF;

    if (digit <= 9)
	{
      *ptr++ = (char) ('0' + digit);
	}
    else
	{
      *ptr++ = (char) ('a' - 10 + digit);
	}
  }

  *ptr = (char) 0;
  OLEDrgb_PutString(InstancePtr,buf);

  return;
}


/****************************************************************************/
/**
* Write a 32-bit number in Radix "radix" to LCD display
*
* Writes a 32-bit number to the LCD display starting at the current
* cursor position. "radix" is the base to output the number in.
*
* @param num is the number to display
*
* @param radix is the radix to display number in
*
* @return *NONE*
*
* @note
* No size checking is done to make sure the string will fit into a single line,
* or the entire display, for that matter.  Watch your string sizes.
*****************************************************************************/
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix)
{
  char  buf[16];

  PMDIO_itoa(num, buf, radix);
  OLEDrgb_PutString(InstancePtr,buf);

  return;
}

/****************************************************************************/
/**
*  PMDIO_counter
*
* Caps the Value of the HSV in particular range
* @param num is the number to display
*
*
*
* @return *NONE*

*****************************************************************************/
void PMDIO_counter( int *num  )
{
	if(*num >= 255)
	{
		*num = 255;
	}
	else if (*num < 0 )
	{
		*num = 0;
	}

}

/****************************************************************************/
/**
*  seven_seg_counter
*
* Caps the Value of the Seven Segment Duty Cycle in particular range
* @param num is the number to display
*
*
*
* @return *NONE*
*
*****************************************************************************/
void seven_seg_counter( uint8_t *num  )
{
	if(*num >= 99)
	{
		*num = 99;
	}
}




/* ------------------------------------------------------------ */
/*** LEDrgb_BuildHSV
**
**   Parameters:
**      hue - Hue of color
**      sat - Saturation of color
**      val - Value of color
**
**   Return Value:
**      RGB representation of input color in 16-bit (565) color format
**
**   Errors:
**      none
**
**   Description:
**      Converts an HSV value into a 565 RGB color used by the OLEDrgb
*/

void LEDrgb_BuildHSV(u8 hue, u8 sat, u8 val) {
   u8 region, remain, p, q, t;
   region = hue / 43;
   remain = (hue - (region * 43)) * 6;
   p = (val * (255 - sat)) >> 8;
   q = (val * (255 - ((sat * remain) >> 8))) >> 8;
   t = (val * (255 - ((sat * (255 - remain)) >> 8))) >> 8;

   switch (region) {
   case 0:
      R = val;
      G = t;
      B = p;
      break;
   case 1:
      R = q;
      G = val;
      B = p;
      break;
   case 2:
      R = p;
      G = val;
      B = t;
      break;
   case 3:
      R = p;
      G = q;
      B = val;
      break;
   case 4:
      R = t;
      G = p;
      B = val;
      break;
   default:
      R = val;
      G = p;
      B = q;
      break;
   }
   NX4IO_RGBLED_setDutyCycle(RGB1, R, G, B);
}



/**************************** INTERRUPT HANDLERS ******************************/
/*Heart beat  and reset */
/****************************************************************************/
/**
* Fixed interval timer interrupt handler
*
* Reads the GPIO port which reads back the hardware generated PWM wave for the RGB Leds
*
* @note
* ECE 544 students - When you implement your software solution for pulse width detection in
* Project 1 this could be a reasonable place to do that processing.
 *****************************************************************************/
bool g_bit_toggle = 0;
uint32_t g_toggle_counter = 100;
void FIT_Handler(void)
{
	g_toggle_counter--;
	if (g_toggle_counter == 0) {
		NX4IO_SSEG_setDecPt(SSEGLO, DIGIT6, g_bit_toggle);
		g_bit_toggle = !g_bit_toggle;
		g_toggle_counter = 100;
	}

	// Read the GPIO port to read back the generated PWM signal for RGB led's
	gpio_in = XGpio_DiscreteRead(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL) & g_mask;

	if((previous_state != gpio_in) && (gpio_in))
	{
		g_duty_cycle = ((on * 2 * 100) / (on + off) );
		on = 0;
		off = 0;
	}
	if (gpio_in)
	{
		on++;
	}
	else
	{
		off++;
	}

	previous_state = gpio_in ;
}

