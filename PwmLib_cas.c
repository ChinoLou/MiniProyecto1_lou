/*
 * PwmLib_cas.c
 *
 *  Created on: 6/02/2021
 *      Author: Steven Josué Castillo Lou - 17169
 */


#include "PwmLib_cas.h"

void
init_PWM_1(uint32_t PortPerif, uint32_t mod1_en, uint32_t M1pwmN, uint32_t pwm_portN, uint32_t pwm_pinN, uint32_t sel_gen)
{
    //1 SET PWM CLOCK AS SYSTEM CLOCK DIVIDED BY 64 ----------------------------------------------------------------------------------------------------------------
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //allows the lock placed on the pins of the GPIO module to be removed
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    ROM_GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
    ROM_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //2 Se habilita el Periferico para el PWM--> en este caso el puerto F --> GPIOF asociado al pwm escogido
    SysCtlPeripheralEnable(PortPerif);              //GPIO FOR PWM1
    SysCtlPeripheralEnable(mod1_en);                //PWM1 PERIPHERAL ENABLE

    //3. //Configuración de los Periféricos Asociados
    GPIOPinConfigure(M1pwmN);              //PD0 AS M1PWM5
    GPIOPinTypePWM(pwm_portN, pwm_pinN);   //GPIO PF1 FOR PWM1 ************** cambiar *********

    //4. Tipo de Modo PWM
    PWMGenConfigure(PWM1_BASE, sel_gen, PWM_GEN_MODE_DOWN);   //CAMBIAR EL GENERADO ASOCIADO --> PWM_GEN_0
    PWMGenEnable(PWM1_BASE, sel_gen);

    //4. Tipo de Modo PWM
    //PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN); //CAMBIAR EL GENERADO ASOCIADO
    //PWMGenEnable(PWM1_BASE, PWM_GEN_0);


}


uint32_t
Grad_to_PWMPulse(float grados) // Esta funcion mapea valores de -90@90 grados a valores de 430 @ 2000 para el ui32Width de la función "PWMPulseWidthSet" del drivers lib
{
    float   pwm_pulse = 0.0;
    uint32_t     pwm_pulse_out;
    pwm_pulse =  grados*(8.722222222222222) + 1000; // 215-1785 pendiente m=(157.0/18.0)
    pwm_pulse_out = (uint32_t)pwm_pulse;            // cast de float @ uint32_t
    return pwm_pulse_out;
}


//pwm_pulse =  (77.0/18.0)*grados + 430.0;
//pwm_pulse =  (157.0/18.0)*grados + 1215.0; 430-2000


