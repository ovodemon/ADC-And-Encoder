// ENGR-2350 Lab 4
// Yijia Zhou

#include "engr2350_msp432.h"

void GPIOInit();
void TimerInit();
void ADCInit();
void Encoder_ISR();
void T1_100ms_ISR();

Timer_A_UpModeConfig TA0cfg;
Timer_A_UpModeConfig TA1cfg;
Timer_A_ContinuousModeConfig TA3cfg;
Timer_A_CompareModeConfig TA0_ccr3;
Timer_A_CompareModeConfig TA0_ccr4;
Timer_A_CaptureModeConfig TA3_ccr0;
Timer_A_CaptureModeConfig TA3_ccr1;


// Encoder total events
uint32_t enc_total_L,enc_total_R;
// Speed measurement variables
int32_t Tach_L_count,Tach_L,Tach_L_sum,Tach_L_sum_count,Tach_L_avg; // Left wheel
int32_t Tach_R_count,Tach_R,Tach_R_sum,Tach_R_sum_count,Tach_R_avg; // Right wheel

uint8_t run_control = 0; // Flag to denote that 100ms has passed and control should be run.

uint16_t adc_out[2]; // variable to store the ADC output

uint16_t pwm_max = 720; // Maximum limit on PWM output
uint16_t pwm_min = 10; // Minimum limit on PWM output
uint16_t pwm_left_set = 120; // Calculated PWM output (control output)
uint16_t pwm_right_set = 120;
float left_error_sum = 0;
float right_error_sum = 0;

uint16_t speed_L;
uint16_t speed_R;
uint8_t turn = 0;

uint32_t desired_speed;
uint32_t desired_speed_left;
uint32_t desired_speed_right;
uint32_t speed_diff = 0;

uint32_t raduis = 0; // unit in mm. Range 381 to 2972.
uint8_t dw = 75;

float ki = 0.01; // integral control gain

int main(void)
{
    SysInit();
    GPIOInit();
    ADCInit();
    TimerInit();

    __delay_cycles(24e6);

    while(1){
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN6|GPIO_PIN7);
        if(run_control){    // If 100 ms has passed
            run_control = 0;    // Reset the 100 ms flag

            if (!GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN0)){
                turn = 0;
            }else if (!GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN7)){
                turn = 1;
            }

            ADC14_toggleConversionTrigger();
            while (ADC14_isBusy()){
            }
            ADC14_getMultiSequenceResult(adc_out);

            desired_speed = 400*adc_out[0] / 16384; // mm/s speed
            raduis = 381 + 2400*adc_out[1] / 16384; // mm

            //<-- Start Wheel Speed Control -->
            if (!turn){
                desired_speed_left = desired_speed-2;
                desired_speed_right = desired_speed;
            }else if (turn){
                desired_speed_left = desired_speed - desired_speed*dw/raduis; // calculate desired wheel speed as desired speed +/- differential speed
                desired_speed_right = desired_speed + desired_speed*dw/raduis;
            }

            printf("%u\t%u\t%u\r\n", desired_speed_left, desired_speed_right,raduis);
            if (desired_speed_left < 20){
                desired_speed_left = 0;
                pwm_left_set = 0;
                Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, 0);
            }else{
                if (ki*left_error_sum >= 1){
                    if (speed_L > desired_speed_left){
                        pwm_left_set = pwm_left_set - ki * left_error_sum;
                        left_error_sum = 0;
                    }
                    if (speed_L < desired_speed_left){
                        pwm_left_set += ki * left_error_sum;
                        left_error_sum = 0;
                    }
                }

                if(pwm_left_set > pwm_max) pwm_left_set = pwm_max;  // Set limits on the pwm control output
                if(pwm_left_set < pwm_min) pwm_left_set = pwm_min;
                Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, pwm_left_set);
            }

            if (desired_speed_right < 20){
                desired_speed_right = 0;
                pwm_right_set = 0;
                Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, 0);
            }else{
                if (ki*right_error_sum>=1){
                    if (speed_R > desired_speed_right){
                        pwm_right_set -= ki * right_error_sum;
                        right_error_sum = 0;
                    }else if (speed_R < desired_speed_right){
                        pwm_right_set += ki * right_error_sum;
                        right_error_sum = 0;
                    }
                }

                if(pwm_right_set > pwm_max) pwm_right_set = pwm_max;  // Set limits on the pwm control output
                if(pwm_right_set < pwm_min) pwm_right_set = pwm_min;
                Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, pwm_right_set);
            }

            printf("%u\t%u\t%u\t%u\t%u\r\n", desired_speed, speed_L, pwm_left_set, speed_R, pwm_right_set);
        }
    }
}

void ADCInit(){
    // Add your ADC initialization code here.
    //  Don't forget the GPIO, either here or in GPIOInit()!!
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_SMCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_4, ADC_NOROUTE);
    ADC14_setResolution(ADC_14BIT);
    // Select a GPIO pin'
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A12, false);
    ADC14_configureSingleSampleMode(ADC_MEM0, false);
    ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A9, false);
    ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, false);

    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);
    ADC14_enableConversion();
}

void GPIOInit(){
    GPIO_setAsOutputPin(GPIO_PORT_P5,GPIO_PIN4|GPIO_PIN5);   // Motor direction pins
    GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN6|GPIO_PIN7);   // Motor enable pins
        // Motor PWM pins
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN6|GPIO_PIN7,GPIO_PRIMARY_MODULE_FUNCTION);
        // Motor Encoder pins
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P10,GPIO_PIN4|GPIO_PIN5,GPIO_PRIMARY_MODULE_FUNCTION);

    GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN4|GPIO_PIN5);   // Motors set to forward
    GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN6|GPIO_PIN7);   // Motors are OFF

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4, GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION); // P4.1 (ADC14) A12
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION); // P4.4 (ADC14) A9

    GPIO_setAsInputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN7);// BMP0 and BMP5
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN0);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN7);
}

void TimerInit(){
    // Configure PWM timer for 30 kHz
    TA0cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA0cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    TA0cfg.timerPeriod = 799;
    Timer_A_configureUpMode(TIMER_A0_BASE,&TA0cfg);
    // Configure TA0.CCR3 for PWM output
    TA0_ccr3.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;
    TA0_ccr3.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    TA0_ccr3.compareValue = pwm_left_set;
    Timer_A_initCompare(TIMER_A0_BASE,&TA0_ccr3);
    // Configure TA0.CCR4 for PWM output
    TA0_ccr4.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;
    TA0_ccr4.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    TA0_ccr4.compareValue = pwm_right_set;
    Timer_A_initCompare(TIMER_A0_BASE,&TA0_ccr4);
    // Configure Encoder timer in continuous mode
    TA3cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA3cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    TA3cfg.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    Timer_A_configureContinuousMode(TIMER_A3_BASE,&TA3cfg);
    // Configure TA3.CCR0 for Encoder measurement
    TA3_ccr0.captureRegister = TIMER_A_CAPTURECOMPARE_REGISTER_0;
    TA3_ccr0.captureMode = TIMER_A_CAPTUREMODE_RISING_EDGE;
    TA3_ccr0.captureInputSelect = TIMER_A_CAPTURE_INPUTSELECT_CCIxA;
    TA3_ccr0.synchronizeCaptureSource = TIMER_A_CAPTURE_SYNCHRONOUS;
    TA3_ccr0.captureInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    Timer_A_initCapture(TIMER_A3_BASE,&TA3_ccr0);
    // Configure TA3.CCR1 for Encoder measurement
    TA3_ccr1.captureRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    TA3_ccr1.captureMode = TIMER_A_CAPTUREMODE_RISING_EDGE;
    TA3_ccr1.captureInputSelect = TIMER_A_CAPTURE_INPUTSELECT_CCIxA;
    TA3_ccr1.synchronizeCaptureSource = TIMER_A_CAPTURE_SYNCHRONOUS;
    TA3_ccr1.captureInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    Timer_A_initCapture(TIMER_A3_BASE,&TA3_ccr1);
    // Register the Encoder interrupt
    Timer_A_registerInterrupt(TIMER_A3_BASE,TIMER_A_CCR0_INTERRUPT,Encoder_ISR);
    Timer_A_registerInterrupt(TIMER_A3_BASE,TIMER_A_CCRX_AND_OVERFLOW_INTERRUPT,Encoder_ISR);
    // Configure 10 Hz timer
    TA1cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA1cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_16;
    TA1cfg.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    TA1cfg.timerPeriod = 37499;
    Timer_A_configureUpMode(TIMER_A1_BASE,&TA1cfg);
    Timer_A_registerInterrupt(TIMER_A1_BASE,TIMER_A_CCRX_AND_OVERFLOW_INTERRUPT,T1_100ms_ISR);
    // Start all the timers
    Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);
    Timer_A_startCounter(TIMER_A1_BASE,TIMER_A_UP_MODE);
    Timer_A_startCounter(TIMER_A3_BASE,TIMER_A_CONTINUOUS_MODE);
}



void Encoder_ISR(){
    // If encoder timer has overflowed...
    if(Timer_A_getEnabledInterruptStatus(TIMER_A3_BASE) == TIMER_A_INTERRUPT_PENDING){
        Timer_A_clearInterruptFlag(TIMER_A3_BASE);
        Tach_L_count += 65536;
        Tach_R_count += 65536;
    // Otherwise if the Left Encoder triggered...
    }else if(Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0)&TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG){
        Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
        enc_total_L++;   // Increment the total number of encoder events for the left encoder
        // Calculate and track the encoder count values
        Tach_L = Tach_L_count + Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
        Tach_L_count = -Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
        // Sum values for averaging
        Tach_L_sum_count++;
        Tach_L_sum += Tach_L;
        speed_L = 70 * 3.1415926 * 24000000/(360 * Tach_L);
        left_error_sum += abs(desired_speed_left - (70 * 3.1415926 * 24000000/(360 * Tach_L)));
        // If 6 values have been received, average them.
        if(Tach_L_sum_count == 6){
            Tach_L_avg = Tach_L_sum/6;
            Tach_L_sum_count = 0;
            Tach_L_sum = 0;
        }
    // Otherwise if the Right Encoder triggered...
    }else if(Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1)&TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG){
        Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
        enc_total_R++;
        Tach_R = Tach_R_count + Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
        Tach_R_count = -Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
        Tach_R_sum_count++;
        Tach_R_sum += Tach_R;
        speed_R = 70 * 3.1415926 * 24000000/(360 * Tach_R);
        right_error_sum += abs(desired_speed_right - (70 * 3.1415926 * 24000000/(360 * Tach_R)));
        if(Tach_R_sum_count == 6){
            Tach_R_avg = Tach_R_sum/6;
            Tach_R_sum_count = 0;
            Tach_R_sum = 0;
        }
    }
}

void T1_100ms_ISR(){
    Timer_A_clearInterruptFlag(TIMER_A1_BASE);
    run_control = 1;
}
