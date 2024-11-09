#pragma once


#include <ESP8266WiFi.h>          // https://github.com/esp8266/Arduino
#include "config.h"

enum CMD {
  CMD_NOT_DEFINED,
  CMD_STATE_CHANGED,
};


class OnOffBrightnessBulb {
  public:
    OnOffBrightnessBulb(void);
    
    void      init(void);
    void      loop(void);
    
    bool      getState(void);
    bool      setState(bool p_state);
    
    uint8_t   getBrightness(void);
    bool      setBrightness(uint8_t p_brightness);
    

  private:
    bool      m_state;    
    uint8_t   m_brightness;

};
#endif
