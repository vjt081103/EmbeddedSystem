#include "MKL46Z4.h"
#include "I2C.h"
#include "board.h"
#include "fsl_i2c.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "LED.h"
#include "SYSTICKTIMER.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "float.h"
#include "SWITCH.h"
#define RUN 1
#define STOP 0

uint32_t msTicks = 0;
uint32_t Ticks10 = 0;
uint32_t Ticks= 0;
uint32_t ledTicks = 0;

uint8_t resetcount = 0;

void SysTick_Handler() {
	Ticks10++;
	ledTicks++;
	if(Ticks10++ >= 1000) {
		Ticks++;
		Ticks10 = 0;
	}
	if(Ticks > 60) {
		resetcount = 1;
	}
	msTicks++;
}
void delay(uint32_t tick) {
	msTicks = 0;
	while(msTicks < tick) {
		__nop();
	}
	msTicks = 0;
}

#define MAG_I2C_CLK_SRC I2C0_CLK_SRC
#define MAG_I2C_CLK_FREQ CLOCK_GetFreq(I2C0_CLK_SRC)

#define LCD_I2C_CLK_SRC I2C1_CLK_SRC
#define LCD_I2C_CLK_FREQ CLOCK_GetFreq(I2C1_CLK_SRC);

#define MAG_ADDRESS 0x0E
#define LCD_ADDRESS 0x27
#define I2C_RELEASE_BUS_COUNT 100U
#define I2C_BAUDRATE 100000U
#define MAG_WHOAMI 0xC4U
#define MAG_STATUS 0x00U
#define MAG_CTRL_REG1 0x10U
#define MAG_CTRL_REG2 0x11U
#define MAG_WHOAMI_REG 0x07U
#define MAG_READ_TIMES 10U
#define SYS_REG 0x08U

#define X_MSB_REG 0x01
#define X_LSB_REG 0x02
#define Y_MSB_REG 0x03
#define Y_LSB_REG 0x04
#define Z_MSB_REG 0x05
#define Z_LSB_REG 0x06



static bool I2C_ReadMagWhoAmI(void);
static bool I2C_WriteMagReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value);
static bool I2C_ReadMagRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);


void blink_led_red() {
	if(ledTicks >= 500) {
		PTE->PTOR |= 1 << 29;
		ledTicks = 0;
	}
}

void blink_led_green() {
	if(ledTicks >= 1000) {
		PTD->PTOR |= 1 << 5;
		ledTicks = 0;
	}
}

void delay_ms1(uint32_t ms) {
	volatile uint32_t count;
	while(ms--) {
		count = 4800;
		while(count--) {
		}
	}
}

status_t LCD_WriteBytes(uint8_t * data,size_t size) {
	i2c_master_transfer_t masterXfer;
	
	 memset(&masterXfer, 0, sizeof(masterXfer));
   masterXfer.slaveAddress = LCD_ADDRESS;
   masterXfer.direction = kI2C_Write;
   masterXfer.subaddress = 0;
   masterXfer.subaddressSize = 0;
   masterXfer.data = data;
   masterXfer.dataSize = size;
   masterXfer.flags = kI2C_TransferDefaultFlag;
	
	return I2C_MasterTransferBlocking(I2C1,&masterXfer);
}


void LCD_send_cmd(uint8_t cmd) {
	uint8_t data[4];
	data[2] = (((cmd << 4) & 0xf0) | 0x0C);
	data[3] = (((cmd << 4) & 0xf0) | 0x08);
	data[0] = ((cmd & 0xf0) | 0x0C);
	data[1] = ((cmd & 0xf0) | 0x08);
	LCD_WriteBytes(data,4);
}

void LCD_send_value(uint8_t value) {
	uint8_t data[4];
	data[2] = (((value << 4) & 0xf0) | 0x0D);
	data[3] = (((value << 4) & 0xf0) | 0x09);
	data[0] = ((value & 0xf0) | 0x0D);
	data[1] = ((value & 0xf0) | 0x09);
	LCD_WriteBytes(data,4);
}

void LCD_Init() {
	i2c_master_config_t masterConfig;
	uint32_t sourceClock;
	
	I2C_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate_Bps = I2C_BAUDRATE;
	
	sourceClock = CLOCK_GetFreq(I2C1_CLK_SRC);
	
	I2C_MasterInit(I2C1,&masterConfig,sourceClock);
	
	delay(50);
	LCD_send_cmd(0x30);
	delay(10);
	LCD_send_cmd(0x30);
	delay(1);
	LCD_send_cmd(0x30);
	delay(1);
	LCD_send_cmd(0x20);
	
	delay(1);
	LCD_send_cmd(0x28);
	delay(1);
	LCD_send_cmd(0x08);
	delay(1);
	LCD_send_cmd(0x01);
	delay(20);
	LCD_send_cmd(0x06);
	delay(1);
	LCD_send_cmd(0x0C);
	delay(1);
}

void LCD_location(int row,int col);
void LCD_clear() {
	LCD_location(0,0);
	int i = 0;
	for (i = 0; i < 16 ;i++) {
		LCD_send_value(' ');
	}
	LCD_location(1,0);
	for(i = 0; i < 16; i++) {
		LCD_send_value(' ');
	}
	LCD_location(0,0);
	
}

void LCD_location(int row, int col) {
	switch(row) {
		case 0:
			col |= 0x80;
		case 1:
			col |= 0xC0;
		default:
			col |= 0x80;
	}
	LCD_send_cmd(col);
}

void LCD_send_string(char * str) {
	while(*str) 
	{
		LCD_send_value(*str);
		str++;
	}
}

i2c_master_handle_t g_m_handle;
uint8_t g_mag_addr_found = 0x00;

volatile bool completionFlag = false;
volatile bool nakFlag = false;

static int16_t xmax;
static int16_t xmin;
static int16_t ymax;
static int16_t ymin;
static int16_t xbias;
static int16_t ybias;
static float scalexy;


static uint8_t chedo = RUN;


static void caculate(int16_t x, int16_t y) {
	if ( x > xmax) {
		xmax = x;
	}
	if ( y > ymax) {
		ymax = y;
	}
	if ( x < xmin) {
		xmin = x;
	}
	if ( y < ymin) {
		ymin = y;
	}
}

void PORTC_PORTD_IRQHandler() {
	if (PORTC->ISFR & (1 <<3) ) {
		if(chedo == RUN) {
			chedo = STOP;
			PTD->PDOR |= (1<<5);
			PTE->PDOR &= ~(1<<29);
			LCD_clear();
			delay_ms1(50);
			LCD_send_string("  Che do dung");
		} else if (chedo == STOP) {
			chedo = RUN;
			PTD->PDOR &= ~(1 << 5);
			PTE->PDOR |= 1<<29;
			LCD_clear();
			delay_ms1(50);
			if(resetcount == 0) {
				Ticks = 0;
				LCD_send_string("Collecting datas");
			}
			
		}
		PORTC->PCR[3] |= PORT_PCR_ISF_MASK;
	}

	if (PORTC->ISFR & ( 1 << 12)) {
		PORTC->PCR[12] |= PORT_PCR_ISF_MASK;
		NVIC_SystemReset();
	}
}




static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    if (status == kStatus_Success)
    {
        completionFlag = true;
    }
    if ((status == kStatus_I2C_Nak) || (status == kStatus_I2C_Addr_Nak))
    {
        nakFlag = true;
    }
}

static bool I2C_ReadMagWhoAmI(void)
{
    uint8_t who_am_i_reg = MAG_WHOAMI_REG;
    uint8_t who_am_i_value = 0x00;
    bool find_device = false;
    uint8_t i = 0;
    uint32_t sourceClock = 0;

    i2c_master_config_t masterConfig;

    I2C_MasterGetDefaultConfig(&masterConfig);

    masterConfig.baudRate_Bps = I2C_BAUDRATE;

    sourceClock = MAG_I2C_CLK_FREQ;

    I2C_MasterInit(I2C0, &masterConfig, sourceClock);

    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = MAG_ADDRESS;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = &who_am_i_reg;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferNoStopFlag;

    I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
        while ((!nakFlag) && (!completionFlag))
        {
        }

        nakFlag = false;

        if (completionFlag == true)
        {
            completionFlag = false;
            find_device = true;
        }
				g_mag_addr_found = masterXfer.slaveAddress;

    if (find_device == true)
    {
        masterXfer.direction = kI2C_Read;
        masterXfer.subaddress = 0;
        masterXfer.subaddressSize = 0;
        masterXfer.data = &who_am_i_value;
        masterXfer.dataSize = 1;
        masterXfer.flags = kI2C_TransferRepeatedStartFlag;

        I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
        while ((!nakFlag) && (!completionFlag))
        {
        }

        nakFlag = false;

        if (completionFlag == true)
        {
            completionFlag = false;
            if (who_am_i_value == MAG_WHOAMI)
            {
                PRINTF("Found an MAG3110 on board , the device address is 0x%x . \r\n", masterXfer.slaveAddress);
                return true;
            }
            else
            {
                PRINTF("Found a device, the WhoAmI value is 0x%x\r\n", who_am_i_value);
                PRINTF("The device address is 0x%x. \r\n", masterXfer.slaveAddress);
                return false;
            }
        }
        else
        {
            PRINTF("Not a successful i2c communication \r\n");
            return false;
        }
    }
    else
    {
        PRINTF("\r\n Do not find an accelerometer device ! \r\n");
        return false;
    }
}

static bool I2C_WriteMagReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = &value;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);

    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
			PRINTF("Hello\r\n");
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}


static bool I2C_ReadMagRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = rxBuff;
    masterXfer.dataSize = rxSize;
    masterXfer.flags = kI2C_TransferDefaultFlag;
		

    I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);

    while ((!nakFlag) && (!completionFlag))
    {
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}


int main(void)
{
    bool isThereMag = false;
    BOARD_InitPins();
    BOARD_InitDebugConsole();
		SysTick_Config(SystemCoreClock/1000);
		I2C0_Init();
		I2C1_Init();
		Init_SW1();
		Init_SW2();
		LCD_Init();
		LCD_clear();
		LCD_location(1,0);
		LCD_send_string("Collecting datas");
		Init_Red_Led();
		PTE->PDOR |= 1 << 29;
		Init_Green_Led();
		

    I2C_MasterTransferCreateHandle(I2C0, &g_m_handle, i2c_master_callback, NULL);
    isThereMag = I2C_ReadMagWhoAmI();

		uint8_t databyte = 0;
        uint8_t write_reg = 0;
        uint8_t readBuff[7];
        int16_t x, y, z;
        uint8_t status0_value = 0;
        uint32_t i = 0U;

					
        write_reg = MAG_CTRL_REG1;
        databyte = 0x00;
        I2C_WriteMagReg(I2C0, g_mag_addr_found, write_reg, databyte);
			
				write_reg = MAG_CTRL_REG1;
        databyte = 0x01;
        I2C_WriteMagReg(I2C0, g_mag_addr_found, write_reg, databyte);
				
		
    while(1)
    {
			delay(500);
			if(chedo == RUN) {
				blink_led_green();
				float x1,y1;
        uint8_t status0_value = 0;
        uint32_t i = 0U;
					
				I2C_ReadMagRegs(I2C0, g_mag_addr_found, MAG_STATUS, readBuff, 7);
						
        status0_value = readBuff[0];
        x = ((int16_t)(((readBuff[1] * 256U) | readBuff[2]))) ;
        y = ((int16_t)(((readBuff[3] * 256U) | readBuff[4]))) ;
        z = ((int16_t)(((readBuff[5] * 256U) | readBuff[6]))) ;
			
				if(Ticks < 60 && resetcount == 0) {
					caculate(x,y);
				} else {
				  xbias = (xmax + xmin)/2;
				  ybias = (ymax + ymin)/2;
					
					int16_t deltax = xmax - xmin;
					int16_t deltay = ymax - ymin;
					
					
					if (deltay == 0) {
						break;
					} else {
					scalexy = ((float)deltax) / ((float) deltay);
					}
				if(scalexy) {
					
				  x1 = (float)( x - xbias );
					if ( x1 > (deltax/2)) {
						x1 = deltax/2;
					} else if (x1 < -(deltax/2)) {
						x1 = -deltax/2;
					}
				  y1 = (float)( y - ybias );
				  y1 = scalexy * y1;
					if (y1 > (deltax/2)) {
						y1 = deltax/2;
					} else if (y1 < -(deltax/2)) {
						y1 = -deltax/2;
					}
					float degree = 180 * acosf((x1/sqrt(pow(y1,2) + pow(x1,2))))/3.14;
					if ( y1 < 0) {
						degree = - degree;
					}
					
					degree += 180;
					
					char buf[16];
					sprintf(buf,"%.5f",degree);
					LCD_clear();
					delay(40);
					LCD_send_string(buf);
					if(degree < 22.5 || degree >= 337.5) {
						LCD_send_string(" B");
					} else if (degree >= 22.5 && degree < 67.5) {
						LCD_send_string(" DB");
					} else if (degree >= 67.5 && degree < 112.5) {
						LCD_send_string(" D");
					} else if (degree >= 112.5 && degree < 157.5) {
						LCD_send_string(" DN");
					} else if (degree >= 157.5 && degree < 202.5) {
						LCD_send_string(" N");
					} else if (degree >= 202.5 && degree < 247.5) {
						LCD_send_string(" TN");
					} else if (degree >= 247.5 && degree < 292.5) {
						LCD_send_string(" T");
					} else if (degree >= 292.5 && degree < 337.5) {
						LCD_send_string(" TB");
					}
			}
		}	
		} else if (chedo == STOP) {
			PRINTF("%d\r\n",ledTicks);
			blink_led_red();
		}		
	}
}

