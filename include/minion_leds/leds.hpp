#ifndef __MINION__LEDS_LEDS_HPP__
#define __MINION__LEDS_LEDS_HPP__

#include <iostream>

#include <stdint.h>
#include <wiringPi.h>

#define ON 1    
#define OFF 0 

class Leds {
    public:
        Leds(uint32_t led_pin_R, uint32_t led_pin_G, uint32_t led_pin_B);
        ~Leds();

        void setColorLed(int v_iRed, int v_iGreen, int v_iBlue);

    private:
        uint32_t _led_pin_R;           //LED_R is connected to  wiringPi port 3 of Raspberry pi
        uint32_t _led_pin_G;           //LED_G is connected to  wiringPi port 2 of Raspberry pi
        uint32_t _led_pin_B ;           //LED_B is connected to  wiringPi port 5 of Raspberry pi

};

#endif