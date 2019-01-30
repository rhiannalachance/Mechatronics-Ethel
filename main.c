/* Name: main.c 2.3.2 Motor control
 * Design of Mechatronic Systems - University of Pennsylvania
 * Fall 2018
 * Author: Rhianna La Chance
 */

#include "teensy_general.h"  // includes the resources included in the teensy_general.h file
#include "t_usb.h"
// void setupADC(void); //declaring a function 
// void readADC(int n);

int main(void)
{

    teensy_clockdivide(0); // set the clock speed
    teensy_led(ON);         // turn on the on board LED
    teensy_wait(1000);// wait 1000 ms when at 16 MHz 

    /*initializing variables*/
    ICR1 = 1250; //sets motor 1 frequency (not actual freq. val)
    ICR3 = 1250; //sets motor 2 frequency (not actual freq. val)

    int rampA = 1;
    int rampB = 1;
    int dir1 = 0;
    int dir2 = 0;

    int error1; // motor 1
    int error2; // motor 2
    int oldADC1;
    int obsADC1;
    int oldADC2;
    int obsADC2;
    int control;

    /* sets timer 1 prescale to 256 */
    set(TCCR1B, CS12); //may need to change this
    clear(TCCR1B, CS11); 
    clear(TCCR1B, CS10); 

    /* sets mode to 14 timer 1 */
    set(TCCR1B, WGM13); 
    set(TCCR1B, WGM12);
    set(TCCR1A, WGM11);
    clear(TCCR1A, WGM10);

    /* sets timer 3 prescale to 256 */
    set(TCCR3B, CS32);
    clear(TCCR3B, CS31);
    clear(TCCR3B, CS30);


    /* sets mode to 14 timer 3 */
    set(TCCR3B, WGM33);
    set(TCCR3B, WGM32);
    set(TCCR3A, WGM31);
    clear(TCCR3A, WGM30);

    /* spin motor */
    set(DDRB, 5); // set B5 as output, PWM
    set(DDRC, 6);// set C6 as output, PWM 
    set(DDRF, 7); //DIR1 motor 1
    set(DDRB, 6); //DIR2 motor 1
    set(DDRD, 0); //DIR1 motor 2
    set(DDRD, 2); //DIR2 motor 2

    clear(DDRF, 0); //ADC pot 1 head (lol)
    clear(DDRF, 1); //ADC pot 2 head
    clear(DDRF, 4); //ADC pot 1 body
    clear(DDRF, 5); //ADC pot 2 body 

    set(TCCR1A, COM1A1); // clear at OCR1A, set at rollover motor 1
    clear(TCCR1A, COM1A0); 
    set(TCCR3A, COM3A1); // clear at OCR3A, set at rollover motor 2
    clear(TCCR3A, COM3A0);

    setupADC();//calls ADC function

    m_usb_init();
    while(!m_usb_isconnected()); // wait for a connection
         for (;;) { 
           
            /* reading ADC values */
            readADC(0); // F0
            while(bit_is_clear(ADCSRA,ADIF));// does nothing unless flag is set


            oldADC1 = ADC -20; //controller postition head
            m_usb_tx_int(oldADC1);
            m_usb_tx_string("  ");
            set(ADCSRA,ADIF);

            readADC(1); // F1
            while(bit_is_clear(ADCSRA,ADIF));

            obsADC1 = ADC + 85; //follower position head plus buffer
            m_usb_tx_int(obsADC1);
            m_usb_tx_string("  ");
            set(ADCSRA,ADIF); // resets flag
      
            readADC(2); //F4
            while(bit_is_clear(ADCSRA, ADIF));

            oldADC2 = ADC; //controller position body
            m_usb_tx_int(oldADC2);
            m_usb_tx_string("  ");
            set(ADCSRA, ADIF);

            readADC(3); //F5
            while(bit_is_clear(ADCSRA, ADIF));

            obsADC2 = ADC - 22; //follower position body plus buffer
            m_usb_tx_int(obsADC2);
            m_usb_tx_string("  ");
            set(ADCSRA, ADIF);

            error1 = obsADC1 - oldADC1; //calculates control error
            m_usb_tx_int(error1);
            m_usb_tx_string("  ");

            error2 = obsADC2 - oldADC2;
            m_usb_tx_int(error2);
            m_usb_tx_string("\n");


            /* motor 1 control */
            if (error1 < -25){
                clear(PORTF, 7);
                set(PORTB, 6);
            }
            else if (error1 > 25){
                set(PORTF, 7);
                clear(PORTB, 6);
            }
            else {
                clear(PORTF, 7);
                clear(PORTB, 6);
            }
            
            /* motor 2 control */
            if (error2 < -30){
                set(PORTD, 0);
                clear(PORTD, 2);
            }
            else if (error2 > 30){
                clear(PORTD, 0);
                set(PORTD, 2);
            }
            else {
                clear(PORTD, 0);
                clear(PORTD, 2);
            }

            OCR1A = 0.9 * ICR1; //setting duty cycle, speed of motor as apporaches angle
            OCR3A = 0.8 * ICR3;
        
        }
    return 0;
}

void setupADC(void){ //sets up ADC

    clear(ADMUX, REFS1); // setting voltage reference Vcc
    set(ADMUX, REFS0);
    /*set ADC clockdivide 128*/
    set(ADCSRA, ADPS2);
    set(ADCSRA, ADPS1);
    set(ADCSRA, ADPS0);
    /*enables free running mode*/
    clear(ADCSRA, ADATE);
    /*enables ADC subsystem*/
    set(ADCSRA, ADEN);
    /*begins conversion*/


}
void readADC(int n){ //reads ADC based on port specified

    if (n == 0){
        set (DIDR0, ADC0D); //disable pin
        clear(ADCSRB, MUX5);
        clear(ADMUX, MUX2);
        clear(ADMUX, MUX1);
        clear(ADMUX, MUX0);

    }
    if (n == 1){
        set (DIDR0, ADC1D); //disable pin
        clear(ADCSRB, MUX5);
        clear(ADMUX, MUX2);
        clear(ADMUX, MUX1);
        set(ADMUX, MUX0);

    }
    if (n == 2){
        set (DIDR0, ADC4D); //disable pin
        clear(ADCSRB, MUX5);
        set(ADMUX, MUX2);
        clear(ADMUX, MUX1);
        clear(ADMUX, MUX0);

    }
    if (n == 3){
        set (DIDR0, ADC5D); //disable pin
        clear(ADCSRB, MUX5);
        set(ADMUX, MUX2);
        clear(ADMUX, MUX1);
        set(ADMUX, MUX0);

    }
    set(ADCSRA, ADSC);
}
