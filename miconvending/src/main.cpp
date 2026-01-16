#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include "i2c.h"
#include "LCD_I2C.h"

// ================= PIN =================
#define COIN_PIN   PD2      
#define DROP1_PIN  PD3
#define DROP2_PIN  PD4
#define LED_PIN    PB5

// pin motor
#define M1_IN1 PD5
#define M1_IN2 PD6
#define M2_IN1 PD7
#define M2_IN2 PB0   

#define M3_IN1 PB1   
#define M3_IN2 PB2   
#define M4_IN1 PB3   
#define M4_IN2 PB4   

// ================= I2C =================
#define LCD_ADDR     0x27
#define BUTTON_ADDR  0x20

// ================= STATE =================
#define STATE_IDLE     0
#define STATE_SELECT   1
#define STATE_RUNNING  2
#define STATE_FINISH   3

volatile uint8_t coinFlag = 0;
uint8_t state = STATE_IDLE;
uint8_t selectedMotor = 0;

// ================= SETUP =================
void setupPins(void){
    DDRD &= ~((1<<COIN_PIN)|(1<<DROP1_PIN)|(1<<DROP2_PIN));
    PORTD |= (1<<COIN_PIN); 

    DDRD |= (1<<M1_IN1)|(1<<M1_IN2)|(1<<M2_IN1);
    DDRB |= (1<<M2_IN2)|(1<<M3_IN1)|(1<<M3_IN2)|(1<<M4_IN1)|(1<<M4_IN2)|(1<<LED_PIN);

    PORTD &= ~((1<<M1_IN1)|(1<<M1_IN2)|(1<<M2_IN1));
    PORTB &= ~((1<<M2_IN2)|(1<<M3_IN1)|(1<<M3_IN2)|(1<<M4_IN1)|(1<<M4_IN2)|(1<<LED_PIN));
}

// ================= MOTOR =================
void stopAllMotors(void){
    PORTD &= ~((1<<M1_IN1)|(1<<M1_IN2)|(1<<M2_IN1));
    PORTB &= ~((1<<M2_IN2)|(1<<M3_IN1)|(1<<M3_IN2)|(1<<M4_IN1)|(1<<M4_IN2));
}

void startMotor(uint8_t m){
    stopAllMotors();
    switch(m){
        case 1: PORTD |= (1<<M1_IN1); break;
        case 2: PORTD |= (1<<M2_IN1); break;
        case 3: PORTB |= (1<<M3_IN1); break;
        case 4: PORTB |= (1<<M4_IN1); break;
    }
}

// ================= BUTTON =================
uint8_t readButtons(void){
    i2c_start(BUTTON_ADDR<<1 | 1);
    uint8_t v = i2c_read_nack();
    i2c_stop();
    return v;
}

// ================= COIN INTERRUPT =================
ISR(PCINT2_vect){
    if(state == STATE_IDLE){
        if(!(PIND & (1<<COIN_PIN))){
            coinFlag = 1;
        }
    }
}

void setupCoinInterrupt(void){
    PCICR  |= (1<<PCIE2);
    PCMSK2 |= (1<<PCINT18);
    sei();
}

// ================= MAIN =================
int main(void){
    setupPins();
    i2c_init();
    LCD_Init(LCD_ADDR);
    setupCoinInterrupt();

    LCD_Clear();
    LCD_Print("Hello!");
    LCD_Goto(0,1);
    LCD_Print("Insert your coin");

    wdt_enable(WDTO_2S);

    uint8_t lastDrop1 = (PIND>>DROP1_PIN)&1;
    uint8_t lastDrop2 = (PIND>>DROP2_PIN)&1;

    while(1){
        wdt_reset();

        uint8_t drop1 = (PIND>>DROP1_PIN)&1;
        uint8_t drop2 = (PIND>>DROP2_PIN)&1;

        // -------- IDLE --------
        if(state == STATE_IDLE){
            if(coinFlag){
                coinFlag = 0;          
                selectedMotor = 0;

                state = STATE_SELECT;

                LCD_Clear();
                LCD_Print("Select product");
            }
        }

        // -------- SELECT --------
        else if(state == STATE_SELECT){
            uint8_t b = readButtons();

            if(!(b & (1<<0))) selectedMotor = 1;
            else if(!(b & (1<<1))) selectedMotor = 2;
            else if(!(b & (1<<2))) selectedMotor = 3;
            else if(!(b & (1<<3))) selectedMotor = 4;

            if(selectedMotor){
                state = STATE_RUNNING;

                LCD_Clear();
                LCD_Print("Please wait :)");

                PORTB |= (1<<LED_PIN);
                startMotor(selectedMotor);

            
                lastDrop1 = (PIND>>DROP1_PIN)&1;
                lastDrop2 = (PIND>>DROP2_PIN)&1;
            }
        }

        // -------- RUNNING --------
        else if(state == STATE_RUNNING){
            if((lastDrop1==0 && drop1==1) || (lastDrop2==0 && drop2==1)){
                _delay_ms(20);
                if(((PIND>>DROP1_PIN)&1) || ((PIND>>DROP2_PIN)&1)){
                    state = STATE_FINISH;
                }
            }
        }

        // -------- FINISH --------
        else if(state == STATE_FINISH){
            stopAllMotors();
            PORTB &= ~(1<<LED_PIN);

            LCD_Clear();
            LCD_Print("----Thankyou----");

            for(uint16_t i=0;i<300;i++){   
                _delay_ms(10);
                wdt_reset();
            }

            LCD_Clear();
            LCD_Print("Hello!");
            LCD_Goto(0,1);
            LCD_Print("Insert your coin");

            selectedMotor = 0;
            state = STATE_IDLE;
        }

        lastDrop1 = drop1;
        lastDrop2 = drop2;

        _delay_ms(5);
    }
}
