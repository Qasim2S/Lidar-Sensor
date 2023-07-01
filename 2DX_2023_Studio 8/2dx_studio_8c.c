/*  Room measurement - Qasim Saadat

*/
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}
void PortH0H1H2H3_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                 // Activate the clock for Port L
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};        // Allow time for clock to stabilize 
	GPIO_PORTH_DIR_R = 0b00001111;  //0b00000000    								      // Make PM0 and PM1 inputs 
  GPIO_PORTH_DEN_R = 0b00001111;  //0b00000011
	return;
}

// Global variable visible in Watch window of debugger
// increments at least once per button press
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}

// Low power wait
void WaitForInt(void)
{    __asm("    wfi\n");
}

// Global variable visible in Watch window of debugger
// increments at least once per button press
volatile unsigned long FallingEdges = 0;

void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x03;    										// Make PJ1 input 
  GPIO_PORTJ_DEN_R |= 0x03;     										// Enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 								//? Configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x03;											//??Disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x03;													//	Enable weak pull up resistor
}

void PortJ_Interrupt_Init(void){
	
		FallingEdges = 0;             			// initialize counter

	
		GPIO_PORTJ_IS_R = 0;     						// (Step 1) PJ1 is Edge-sensitive 
		GPIO_PORTJ_IBE_R = 0;    						//     			PJ1 is not triggered by both edges 
		GPIO_PORTJ_IEV_R = 0;    						//     			PJ1 is falling edge event 
		GPIO_PORTJ_ICR_R = 0x02;      			// 					Clear interrupt flag by setting proper bit in ICR register
		GPIO_PORTJ_IM_R = 0x02;      				// 					Arm interrupt on PJ1 by setting proper bit in IM register
    
		NVIC_EN1_R = 0x00080000;            // (Step 2) Enable interrupt 51 in NVIC (which is in Register EN1)
	
		NVIC_PRI12_R = 0xA0000000; 					// (Step 4) Set interrupt priority to 5

		EnableInt();           							// (Step 3) Enable Global Interrupt. lets go!
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}



int val = 0;
int flag = 0;
//uint16_t degree = 0;
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;
double degree = 0;
void GPIOJ_IRQHandler(void){
	SysTick_Wait10ms(70);
	FallingEdges = FallingEdges + 1;	// Increase the global counter variable ;Observe in Debug Watch Window
	
	flag^=1;
	//val=0;
	//degree = 0;
														//Flash the LED D2 one time
	GPIO_PORTJ_ICR_R = 0x02;     					// Acknowledge flag by setting proper bit in ICR register
}


int main(void) {
	uint8_t byteDate = 0x00;
	uint8_t id, name = 0x00;
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint8_t dataReady;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	PortH0H1H2H3_Init();
	I2C_Init();
	UART_Init();
	//PortF_Init();
	PortJ_Init();							// Initialize the onboard push button on PJ1
	PortJ_Interrupt_Init();
	
	GPIO_PORTF_DATA_R = 0b00000000;

	
	// hello world!
	UART_printf("Program Begins\r\n");
	//int mynumber = 1;
/* Those basic I2C read functions can be used to check your own I2C functions */
	status = VL53L1_RdByte(dev, 0x010F, &byteData); //for model ID (0xEA)
	UART_printf(printf_buffer);
	sprintf(printf_buffer,"(Model_ID)=0x%x\r\n",byteData);
	status = VL53L1_RdByte(dev, 0x0110, &byteData); //for module type (0xCC
	UART_printf(printf_buffer);
	sprintf(printf_buffer,"(Module_Type)=0x%x\r\n",byteData);
	UART_printf(printf_buffer);
	status = VL53L1_RdWord(dev, 0x010F, &wordData); //for both model ID and typ
	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	status = VL53L1X_GetSensorId(dev, &wordData);

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

  status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging
	int delay = 1;
	int val = 0;
	int step = 8;
	int flag2 = 0;
	while(1) {
		
		if((GPIO_PORTJ_DATA_R&0x1)==0){
			SysTick_Wait10ms(80);
			GPIO_PORTN_DATA_R ^= 0b00000010;
			flag2 ^= 1;
			degree = 0;
			val = 0;
		}
	  
		while (dataReady == 0){
		  status = VL53L1X_CheckForDataReady(dev, &dataReady);
          VL53L1_WaitMs(dev, 5);
	  }
		dataReady = 0;
	  
		if (flag == 1){
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait10ms(delay);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait10ms(delay);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait10ms(delay);
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait10ms(delay);
			
			if (val%step == 0 && val !=0 && (flag2==1)){
				
				degree = (double)((val*360)/512);
				status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value
				FlashLED3(delay);
				status = VL53L1X_ClearInterrupt(dev); // clxear interrupt has to be called to enable next interrupt
				
				// print the resulted readings to UART
				sprintf(printf_buffer,"%u, %f\r\n", Distance, degree);
				UART_printf(printf_buffer);
				SysTick_Wait10ms(50);
			}
			if (val == 512){
				FlashLED3(delay);
				flag ^=1;
				val = 0;
				degree = 0;
				for (int i = 0; i<512; i++) {
					GPIO_PORTH_DATA_R = 0b00001100;
					SysTick_Wait10ms(delay);
					GPIO_PORTH_DATA_R = 0b00000110;
					SysTick_Wait10ms(delay);
					GPIO_PORTH_DATA_R = 0b00000011;
					SysTick_Wait10ms(delay);
					GPIO_PORTH_DATA_R = 0b00001001;
					SysTick_Wait10ms(delay);
				}
				
			}
			val++;
		}
		
  }
  
	//VL53L1X_StopRanging(dev);
  //while(1) {}

}

