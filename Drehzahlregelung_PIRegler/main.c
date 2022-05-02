/*
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include "USART_SerielleSchnittstelle.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>


char send_string[] = "String";
char recve_string[] = "EmpfangenString";
char string[10];


int16_t rpm = 0;

uint16_t impulse = 0;
int16_t w_sollwert = 2000;
uint8_t regler_state = 0;
uint16_t counter = 0;


float e = 0;
uint16_t Kr = 4;
uint16_t Kp = 22;
uint16_t Tn = 1;
float T = 0.1;
float I_Anteil = 0;
float _I_Anteil = 0;
float y = 0;


void clearString();

///Impulse zählen
ISR(INT0_vect)
{
    uint8_t sreg = SREG;
    cli();
    impulse++;
    SREG = sreg;
}

///100ms Abtastrate - Regler
ISR(TIMER0_OVF_vect)
{
    uint8_t sreg = SREG;
    cli();
    if(counter >= 6)
    {

        counter = 0;

        if(regler_state == 1)
        {
            rpm = impulse * 20;  // Umdrehungen pro 100ms = impulse/30 -> (60s * Impulse/30) / 0.1
            itoa(rpm, send_string, 10);
            _puts(send_string);
            impulse = 0;

            ///P-Regler
            e = w_sollwert - rpm;   //Regeldifferenz
            y = Kp * e;             //Verstärken

            if(y > 1023)
            {
                OCR1A = 1023;
            }
            else
            if(y < 0)
            {
                OCR1A = 0;
            }
            else
            {
                OCR1A = y;
            }
        }
        else
        if(regler_state == 2)
        {
            rpm = impulse * 20;  // Umdrehungen pro 100ms = impulse/30 -> (60s * Impulse/30) / 0.1
            itoa(rpm, send_string, 10);
            _puts(send_string);
            impulse = 0;

            ///PI-Regler


            e = w_sollwert - rpm;               //Regeldifferenz
            _I_Anteil = I_Anteil;               //Alten I-Anteil speichern, um unnötigen Wachstum zu verhindern
            I_Anteil += Kr*T/Tn*e;              //Summe der I-Anteile bilden
            y = Kr*e + I_Anteil;                //P & I Anteil kombinieren

            if(y<1023 && y >=0)
            {
                OCR1A = y;

            }
            else
            if(y > 0)
            {
                OCR1A = 1023;
                I_Anteil = _I_Anteil;           //I Anteil nicht unnötig wachsen lassen
            }
            else
            {
                OCR1A = 0;
            }
        }
        else
        if(regler_state == 3)
        {
            rpm = impulse * 20;  // Umdrehungen pro 100ms = impulse/30 -> (60s * Impulse/30) / 0.1
            itoa(rpm, send_string, 10);
            _puts(send_string);
            impulse = 0;
            OCR1A = 1023;
        }
    }


    counter++;
    SREG = sreg;
}

ISR(USART_RX_vect)
{
    uint8_t sreg = SREG;
    cli();

    _gets(recve_string);
    if(strcmp(recve_string, "stop") == 0)       //Regler arbeitet nicht
    {
        regler_state = 0;
        EIMSK &= ~(1<<INT0);
        OCR1A = 0;
        impulse = 0;
    }
    else
    if(strcmp(recve_string, "pstart") == 0)     //P-Regler arbeitet
    {
        regler_state = 1;
        EIMSK |= (1<<INT0);
    }
    else
    if(strcmp(recve_string, "pistart") == 0)    //PI-Regler arbeitet
    {
        regler_state = 2;
        EIMSK |= (1<<INT0);
    }
    else
    if(strcmp(recve_string, "sastart") == 0)         //Sprungantwort messen
    {
        regler_state = 3;
        EIMSK |= (1<<INT0);
    }
    else
    if(strcmp(recve_string, "getvalues") == 0)
    {
        char infoString[20]; ///reglerstate,kp,kr,tn,sollwert
        sprintf(infoString, "%u%u#%u*%u!%u", regler_state, Kp, Kr, Tn, w_sollwert);
        _puts(infoString);
    }
    else
    {
        if(recve_string[0] == 'w')  //Stellgröße
        {
            uint8_t i = 1;
            while(recve_string[i] != 'e')
            {
                string[i-1] = recve_string[i];
                i++;
            }
            w_sollwert = atoi(string);
            clearString();
        }
        else
        if(recve_string[0] == 'p')  //Verstärkung P-Regler
        {
            uint8_t i = 1;
            while(recve_string[i] != 'e')
            {
                string[i-1] = recve_string[i];
                i++;
            }
            Kp = atoi(string);
            clearString();
        }
        else
        if(recve_string[0] == 'k')  //Verstärkung PI-Regler
        {
            uint8_t i = 1;
            while(recve_string[i] != 'e')
            {
                string[i-1] = recve_string[i];
                i++;
            }
            Kr = atoi(string);
            clearString();
        }
        else
        if(recve_string[0] == 't')  //Tn PI-Regler
        {
            uint8_t i = 1;
            while(recve_string[i] != 'e')
            {
                string[i-1] = recve_string[i];
                i++;
            }
            Tn = atoi(string);
            clearString();
        }
        else
        if(recve_string[0] == 's')
        {
            regler_state = 3;
        }
    }

    SREG = sreg;
}


int main(void)
{
    ///Setup I/O
    DDRB |= (1<<PB1);   //PB1 als Ausgang für PWM
    DDRD = 0b00000000;

    ///Setup Externe Interrupts
    sei();                  //Globale Freischaltung der Interrupts
    EIMSK &= ~(1<<INT0);     //Freischaltung des Interrupts INT0 durch serielle Schnittstelle
    EICRA |= (1<<ISC00);    //Interrupt durch
    EICRA |= (1<<ISC01);    //steigende Flanke
    PCICR |= (1<<PCIE0);
    PCMSK0 |= (1<<PCINT0);

    ///Setup Timer 0 - 100ms Abtastrate
    TCCR0B = 0b00000101;    //Systemtakt / 1024
    TCCR0A = 0b00000000;    //Normaler Timer
    TIMSK0 |= (1<<TOIE0);   //Überlauf-Interrupt freigeben

    ///Setup Timer 1 - PWM
    TCCR1B = 0b00000101;    //Systemtakt / 1024
    TCCR1B |= (1<<WGM12);   //FastPWM
    TCCR1A = 0b10000011;    //10-Bit PWM, nicht invertiert
    OCR1A = 0;
    TCNT1 = 0;

    ///Setup Serielle Schnittstelle
    init_usart();


    while(1)
    ;

    return 0;
}

void clearString()
{
    memset(string,0, 10);
}
