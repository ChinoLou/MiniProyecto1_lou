/*
 * PwmLib_cas.h
 *
 *  Created on: 6/02/2021
 *      Author: Steven Josué Castillo Lou - 17169
 */

#ifndef PWMLIB_CAS_H_
#define PWMLIB_CAS_H_


//1. ----------------- Includes --------------------------------
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"      //defines the base port addresses for the general purpose input output (GPIO) ports and the base memory address for the PWM output
#include "inc/hw_GPIO.h"        //has the set of values needed to unlock the pins that are locked to specific functions on the microcontroller.
#include "driverlib/gpio.h"     //has the functional values for the GPIO pins as well as the functions needed to do the enabling of the PWM on the pins.
#include "driverlib/pin_map.h"  //has the functions available per pin for many Texas Instruments microcontrollers, include the TM4C123.
#include "driverlib/pwm.h"      //Pwm has the functions needed to enable the PWM module within the chip.
#include "driverlib/sysctl.h"   //Sysctl, or system control, has the drivers needed by the chip to access the modules, such as the GPIO modules or the pwm modules.
#include "driverlib/rom.h"


//2. ----------------- Defines ---------------------------------
#define PWM1_PERIPH     SYSCTL_PERIPH_GPIOD //PD0 y PD1 -> macro para habilitar periferico asociado [PortPerif]
#define PWM2_PERIPH     SYSCTL_PERIPH_GPIOF //PF0 y PF1 -> macro para habilitar periferico asociado [PortPerif]

#define PWM_MOD1_EN     SYSCTL_PERIPH_PWM1  //PWM1 PERIPHERAL MODULE ENABLE [mod1_en]

#define PWM0_MOD1       GPIO_PD0_M1PWM0     //1ER. PWM del Modulo1 (MOD1) - Generador 0 [M1pwmN] *****
#define PWM1_MOD1       GPIO_PD1_M1PWM1     //2DO. PWM del Modulo1 (MOD1) - Generador 0 [M1pwmN]
//#define PWM2_MOD1       GPIO_PA6_M1PWM0   //3ER. PWM del Modulo1 (MOD1) - Generador 1 [M1pwmN]
//#define PWM3_MOD1       GPIO_PE4_M1PWM1   //4TO. PWM del Modulo1 (MOD1) - Generador 1 [M1pwmN]
#define PWM4_MOD1       GPIO_PF0_M1PWM4     //5TO. PWM del Modulo1 (MOD1) - Generador 2 [M1pwmN] **
#define PWM5_MOD1       GPIO_PF1_M1PWM5     //6TO. PWM del Modulo1 (MOD1) - Generador 2 [M1pwmN]
#define PWM6_MOD1       GPIO_PF2_M1PWM6     //7MO. PWM del Modulo1 (MOD1) - Generador 3 [M1pwmN]
#define PWM7_MOD1       GPIO_PF3_M1PWM7     //8VO. PWM del Modulo1 (MOD1) - Generador 3 [M1pwmN]

#define PUERTOD         GPIO_PORTD_BASE     //Puerto GPIO a asociar con el PWM [pwm_portN]
#define PUERTOF         GPIO_PORTF_BASE     //Puerto GPIO a asociar con el PWM [pwm_portN]

#define PWMPIN0         GPIO_PIN_0          //Pin del Puerto GPIO asociado [pwm_pinN]
#define PWMPIN1         GPIO_PIN_1          //Pin del Puerto GPIO asociado [pwm_pinN] (led rojo PF1)
#define PWMPIN2         GPIO_PIN_2          //Pin del Puerto GPIO asociado [pwm_pinN] (led azul PF2)
#define PWMPIN3         GPIO_PIN_3          //Pin del Puerto GPIO asociado [pwm_pinN] (led verd PF3)





//3. ----------- Estructuras de Datos --------------------------




//4. ---------------- Prototipos -------------------------------

void
init_PWM_1(uint32_t PortPerif,
           uint32_t mod1_en,
           uint32_t M1pwmN,
           uint32_t pwm_portN,
           uint32_t pwm_pinN,
           uint32_t sel_gen); //NO TENIAAAA ; !!!!

uint32_t
Grad_to_PWMPulse(float grados);    //int16_t va de -32768 to 32767 (necesitamos de -90 a 90 grados que es lo que nos entrega el MPU6050)
                                   // Esta funcion mapea valores de -90@90 grados a valores de 430 @ 2000 del ui32Width de la función "PWMPulseWidthSet" del drivers lib
                                   // EC. DE LA RECTA DEL SERVO USADO --> Y = (157/18)X + 1215; donde X es el valor de -90 a 90 del MPU


#endif /* PWMLIB_CAS_H_ */
