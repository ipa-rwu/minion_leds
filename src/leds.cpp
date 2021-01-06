#include "minion_leds/leds.hpp"
#include <iostream>

Leds::Leds(uint32_t led_pin_R, uint32_t led_pin_G, uint32_t led_pin_B) :  
_led_pin_R(led_pin_R), _led_pin_G(led_pin_G), _led_pin_B(led_pin_B) 
{
    wiringPiSetup();
    //Initialize the RGB IO as the output mode
	pinMode(this->_led_pin_R, OUTPUT);
	pinMode(this->_led_pin_G, OUTPUT);
	pinMode(this->_led_pin_B, OUTPUT);
    }

Leds::~Leds() {
}

void Leds::setColorLed(int v_iRed, int v_iGreen, int v_iBlue)
{
  v_iRed == ON ? digitalWrite(this->_led_pin_R, HIGH): digitalWrite(this->_led_pin_R, LOW);
 
  v_iGreen == ON ? digitalWrite(this->_led_pin_G, HIGH) : digitalWrite(this->_led_pin_G, LOW);

  v_iBlue == ON ? digitalWrite(this->_led_pin_B, HIGH) : digitalWrite(this->_led_pin_B, LOW);
}

