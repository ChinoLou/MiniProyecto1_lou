#include <stdint.h>                 //Librerías Principales
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_i2c.h"

// Libreria "Driverlib"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"         //Librería UART
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/i2c.h"          //Librería i2C
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/timer.h"        //Librería TMR
#include "driverlib/pwm.h"          //Librería PWM

// Libreria "Sensorlib"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/hw_mpu6050.h"
#include "sensorlib/mpu6050.h"
//#include "mpu6050_rm.h"

// Libreria "Utilities"
#include "utils/uartstdio.h"        //prints
#include "PwmLib_cas.h"             //Libreria Creada para el PWM
#include  <math.h>

// -------------------------  DEFINICIONES -------------------------------
#define PWM_FREQUENCY (50)
#define FILTRO        (0.901)
#define N_PI          (3.14159265359)
#define D_T           (0.001)

#define COTA_SUP      (90.0) //Cota superior del sistema (en pwm)
#define COTA_INF      (-65.0)  //Cota Inferior del sistema --> GRADOS = Grad_to_PWMPulse(-65); // COTA INFERIOR val exacto para los 180°!!!

// ---------------------------- VARIABLES --------------------------------
volatile uint32_t ui32Load;         //Variables PWM
volatile uint32_t ui32PWMClock;
volatile uint8_t  color = 2;        //Prueba Interrupcion TMR0
volatile bool     g_bMPU6050Done;   // A boolean that is set when a MPU6050 command has completed.

float   fAccel[3], fGyro[3];        //Variables MPU
float   Acc_X, Acc_Y, Acc_Z;
float   Giro_X, Giro_Y, Giro_Z;

float   filtro_ang_x;
float   filtro_ang_y;
float   pitch_x, pitchX_previous_error;
float   roll__y, rollY__previous_error;
float   rad_to_deg = 180/N_PI;
float   dt = 0.001; //0.0005;

uint32_t    GRADOS;

// ************** PID *****************************
float   roll_set_angle;     //roll_set_angle --> set point del PID
float   salida_grados_y;
float   error_roll   = 0;   //error actual
float   error_roll_1 = 0;   //error previo
float   E_k = 0 ;           //error acumulado
float   E_k_1 = 0 ;         //error acumulado_previo
float   eD = 0;
float   ang_gyro_x = 0;
float   ang_gyro_y = 0;
float   roll__y_n_1;

//uint8_t bandera_TMR; //bandera interrupción TIMER0

// ------------------------- Constantes PID ---------------------------------
float roll_pid_P = 0;
float roll_pid_I = 0;
float roll_pid_D = 0;

float  roll_Kp = 0.58;
float  roll_Ki = 0.00875/3.0;
float  roll_Kd = 0.9;

/*
double  roll_Kp =0.35;
double  roll_Ki =0.0035;
double  roll_Kd =0.000175;

float  roll_Kp =0.7;
float  roll_Ki =0.035;
float  roll_Kd =0.0004;

float  roll_Kp =0.7;                //funciona bastante bien con angulos pequeños y con velocidad baja
float  roll_Ki =0.00875/4.0;
float  roll_Kd =0.9;
*/

// --------------------------- Instancias ---------------------------------
// Instancias a utilizar
tI2CMInstance   g_sI2CMSimpleInst; // I2C master instance
tMPU6050        sMPU6050;          // Instancia del MPU6050.
//tI2CMInstance   sI2CInst;        // This code assumes that the I2C master instance

// --------------------------- PROTOTIPOS -------------------------------
void Init_GPIOs(void);
void Init_Timer0(void);
void Init_Interrups(void);
void Init_UART(void);
void Init_I2C0(void);
void Init_MPU6050(void);
void MPU6050Callback(void *pvCallbackData, uint_fast8_t ui8Status);
void I2CMSimpleIntHandler(void);
// -----------------------------------------------------------------------

void Timer0IntHandler(void)
{
    // Clear the timer interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    //bandera_TMR = 1;

    /*
    if (color < 8){color = color*2;} else{color = 2;}
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, color); //Prueba de la Interrupción TMR0
    */

    Acc_X = fAccel[0];
    Acc_Y = fAccel[1];
    Acc_Z = fAccel[2];

    Giro_X = fGyro[0];
    Giro_Y = fGyro[1];
    Giro_Z = fGyro[2];

    //Ángulo
    pitch_x   = (atan2(Acc_Y, (sqrt(Acc_X*Acc_X + Acc_Z*Acc_Z))))*(180.0/N_PI);  //giro eje x --> angulo eje y
    roll__y   = (atan2(Acc_X, (sqrt(Acc_Y*Acc_Y + Acc_Z*Acc_Z))))*(-180.0/N_PI); //giro eje y --> angulo eje x

    roll__y = roll__y - FILTRO*roll__y + FILTRO*roll__y_n_1;
    roll__y_n_1 = roll__y;

    //Filtro Complemento para obtener ANGULOS (en grados)
    filtro_ang_x = 0.98*(filtro_ang_x + Giro_X*D_T) + 0.02*pitch_x;  //Giro_X*D_T integracion cada dt seg. dt=1ms dado el TMR0
    filtro_ang_y = 0.98*(filtro_ang_y + Giro_Y*D_T) + 0.02*roll__y;

    //bandera_TMR = 0x00; //se apaga la bandera de la interrupción
}


int main(void)

{
    // Clock de 40MHz --> SYSCTL_SYSDIV_5
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    // ----------------------------------------  INICIALIZACIONES -----------------------------------------------------
    Init_GPIOs();
    Init_Timer0();
    Init_Interrups();
    Init_UART();
    Init_I2C0();
    Init_MPU6050();
    init_PWM_1(PWM1_PERIPH, PWM_MOD1_EN, PWM0_MOD1, PUERTOD, PWMPIN0, PWM_GEN_0); //------------- Funcion Para Inicializar PWMs :DDDD


    ui32PWMClock = SysCtlClockGet() / 64;
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load); //  PWM_GEN_n
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);

    while (1)
    {
        // Request another reading from the MPU6050.
        g_bMPU6050Done = false;
        MPU6050DataRead(&sMPU6050, MPU6050Callback, 0);                          //MPU6050DataRead(&sMPU6050, MPU6050Callback, &sMPU6050);
        while (!g_bMPU6050Done)
        {
        }

        MPU6050DataAccelGetFloat(&sMPU6050, &fAccel[0], &fAccel[1], &fAccel[2]); // Get the new accelerometer and gyroscope readings.
        MPU6050DataGyroGetFloat(&sMPU6050, &fGyro[0], &fGyro[1], &fGyro[2]);



        // ************************* CONTROLADOR PID *********************************
        roll_set_angle = 0.0;                         // SET POINT DE 0° [REFERENCIA]
        //---------------------------------------------------------------------------
        error_roll =  roll_set_angle - filtro_ang_y;  //e_k = r - y
        eD = error_roll - rollY__previous_error;      //eD = e_k - e_k_1;
        //E_k = E_k + error_roll;                     /E_k = E_k + e_k; INTEGRATIVO

        roll_pid_P = roll_Kp*error_roll;
        roll_pid_I = roll_pid_I + roll_Ki*error_roll;
        roll_pid_D = roll_Kd*eD;
        // --------------------------------------------------------------------------
        salida_grados_y = roll_pid_P + roll_pid_I + roll_pid_D;  //salida_grados_y = roll_Kp*error_roll + roll_Ki*E_k + roll_Kd*eD; [SALIDA PID en FLOAT]
        rollY__previous_error = error_roll;

        if(salida_grados_y > COTA_SUP){salida_grados_y = COTA_SUP;}
        if(salida_grados_y < COTA_INF){salida_grados_y = COTA_INF;}

        GRADOS = (uint32_t)Grad_to_PWMPulse(salida_grados_y);    //Se utiliza la función creada para mapear el valor de float(-90@90) de la salida del PID a valores de pwm (215@1750)
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, GRADOS);


        //Envío de datos por UART
        UARTprintf("EjeY %02d | PID %02d | GRADOS %02d \n", (int)filtro_ang_y, (int)salida_grados_y, (int)GRADOS ); //lectura datos MPU


        /*
        if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00)   //GPIO_PIN_4 --> PF4 => SW1 (Prueba pwm con botones TivaC!!!)
        {
            GRADOS = Grad_to_PWMPulse(90);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, GRADOS);
        }

        if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)==0x00)   //GPIO_PIN_0 --> PF0 => SW2
        {
            GRADOS = Grad_to_PWMPulse(-65); // COTA INFERIOR val exacto para los 180°!!!
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, GRADOS);
            //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 430);  //430 VAL HACIA LA DER 180° GRADOS
            //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 440);  //1000 servo a  90° GRADOS
        }
        */



    }

}

//---------------- CONFIG GPIOs -----------------------
void Init_GPIOs(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                              // Enable the GPIO port that is used for the on-board LED (PUERTO F). //PROBAR INTERRUPCION CON LED
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3); // Los pines de los 3 leds como salidas digitales
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);                              //GPIO FOR PWM1
}

//------------ CONFIG INTERRUPCIONES ------------------
void Init_Interrups(void){
    IntEnable(INT_TIMER0A); // Se habilita la interrupcion del TIMER
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();     // Se habilitan las interrupciones
}

//--------------  CONFIG TIMER_0 -----------------------
void Init_Timer0(void){
    //uint32_t ui32Period;
    //ui32Period = (SysCtlClockGet()/1000) / 2;

    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);         // Enable the Timer0 peripheral
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))   // Wait for the Timer0 module to be ready.
    {
    }
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);      // Se configura el TMR0_A con full width periodico

    //TimerLoadSet(TIMER0_BASE, TIMER_A, 40000000*2);     // Prueba Leds  TRM0 cada 2seg. --> frec = SysCtlClock/(40000000*2) = 0.5Hz => T_RM = 1/0.5Hz = 2seg
    TimerLoadSet(TIMER0_BASE, TIMER_A, 40000);            // Prueba MPU   TRM0 cada 1ms.  --> frec = SysCtlClock/(40000) = 40,000000/(40000) =1000Hz => T_RM = 1/1000Hz = 1ms

    TimerEnable(TIMER0_BASE, TIMER_A);                    // Enable the timers.

}

void Init_I2C0(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);  //Se habilita el I2C module 0
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);   //Se resetea el periférico asociado

    // Wait for the I2C0 module to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0));
    {
    }

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for the I2C0 module.
    // I2C data transfer rate set to 400kbps. --> I2CMasterInitExpClk(I2C0_BASE, clockFreq, true);
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true); //true --> data a 400Kbps, false --> data a 100kbps

    // Initialize the I2C master driver.
    I2CMInit(&g_sI2CMSimpleInst, I2C0_BASE, INT_I2C0, 0xff, 0xff, SysCtlClockGet()); //g_sI2CMSimpleInst ********************************

    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000; //clear I2C FIFOs

    I2CMasterIntEnable(I2C0_BASE);
    IntEnable(INT_I2C0);
}


void
Init_UART(void)
{
    // Enable the GPIO Peripheral used by the UART.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, 16000000);
    //UARTprintf("Hola MPU6050 \n");
}


void Init_MPU6050(void)
{
    g_bMPU6050Done = false;
    MPU6050Init(&sMPU6050, &g_sI2CMSimpleInst, 0x68, MPU6050Callback, 0); //0x68 dirección MPU6050  tenia MPU6050Init(&sMPU6050, &g_sI2CMSimpleInst, 0x68, MPU6050Callback, 0);
    while (!g_bMPU6050Done){
    }

    // Configure the MPU6050 for +/- 4 g accelerometer range.
    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_ACCEL_CONFIG, ~MPU6050_ACCEL_CONFIG_AFS_SEL_M, MPU6050_ACCEL_CONFIG_AFS_SEL_8G, MPU6050Callback, &sMPU6050); //MPU6050_ACCEL_CONFIG-_AFS_SEL_4G &sMPU6050--> 0
    while (!g_bMPU6050Done){
    }

    //****************************************** RESETEO  ********************************************************

    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_1, 0x00, 0b00000010 & MPU6050_PWR_MGMT_1_DEVICE_RESET, MPU6050Callback, &sMPU6050);  //Reset IMU
    while (!g_bMPU6050Done){
    }

    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_2, 0x00, 0x00, MPU6050Callback, &sMPU6050);  //Termino el reinicio con el Power MGMT 2
    while (!g_bMPU6050Done){
    }

}

// ----------------------- HANDLERS Interrups ---------------------------------

// The function that is provided by this example as a callback when MPU6050
void MPU6050Callback(void *pvCallbackData, uint_fast8_t ui8Status) // transactions have completed.
{
    // See if an error occurred.
    if (ui8Status != I2CM_STATUS_SUCCESS)
    {
        // An error occurred, so handle it here if required.
    }
    // Indicate that the MPU6050 transaction has completed.
    g_bMPU6050Done = true;
}

// The interrupt handler for the I2C module.
void I2CMSimpleIntHandler(void)
{
    I2CMIntHandler(&g_sI2CMSimpleInst); // Call the I2C master driver interrupt handler.
}


/*
ejex = fGyro[0];
ejey = fGyro[1];
ejez = fGyro[2];

pitch_x   = (atan2(fAccel[1], sqrt(fAccel[1] * fAccel[1] + fAccel[2] * fAccel[2])))*rad_to_deg; //giro eje y --> angulo eje x
roll__y   = (atan2(fAccel[0], sqrt(fAccel[1] * fAccel[1] + fAccel[2] * fAccel[2])))*rad_to_deg; //giro eje x --> angulo eje y

//Filtro Complemento
filtro_ang_x = 0.98*((fGyro[0]*rad_to_deg)*dt + pitch_x_prev) + 0.02*pitch_x;
filtro_ang_y = 0.98*((fGyro[1]*rad_to_deg)*dt + roll__y_prev) + 0.02*roll__y;

pitch_x_prev = filtro_ang_x;
roll__y_prev = filtro_ang_y;


//filtro_ang_x = 0.98*(filtro_ang_x + (Giro_X/32.8)*dt) + 0.02*pitch_x; // filtro_ang_x = 0.98*((Giro_X*rad_to_deg)*dt + pitch_x_prev) + 0.02*pitch_x
//filtro_ang_y = 0.98*(filtro_ang_y + (Giro_Y/32.8)*dt) + 0.02*roll__y; // filtro_ang_y = 0.98*((Giro_Y*rad_to_deg)*dt + pitch_ang_y_prev) + 0.02*roll__y

//pitch_x   = (atan2(fAccel[1], sqrt(fAccel[1] * fAccel[1] + fAccel[2] * fAccel[2])))*rad_to_deg; //giro eje y --> angulo eje x
//roll__y   = (atan2(fAccel[0], sqrt(fAccel[1] * fAccel[1] + fAccel[2] * fAccel[2])))*rad_to_deg; //giro eje x --> angulo eje y



    ang_gyro_x += Giro_X*dt;  //interrupcion TMR0
    ang_gyro_y += Giro_Y*dt;
    filtro_ang_x = 0.98*(filtro_ang_x + ang_gyro_x) + 0.02*pitch_x;
    filtro_ang_y = 0.98*(filtro_ang_y + ang_gyro_y) + 0.02*roll__y;

    pitchX_previous_error = filtro_ang_x; //errores previos
    rollY__previous_error = filtro_ang_y;


*/








