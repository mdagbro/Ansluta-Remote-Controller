#include "ha_mqtt_ansluta_light_with_discovery.h"

///////////////////////////////////////////////////////////////////////////
//   CONSTRUCTOR, INIT() AND LOOP()
///////////////////////////////////////////////////////////////////////////
OnOffBrightnessBulb::OnOffBrightnessBulb(void) {

}

void OnOffBrightnessBulb::init(void) {
  m_state = false;
  m_brightness = 127;
}

///////////////////////////////////////////////////////////////////////////
//   STATE
///////////////////////////////////////////////////////////////////////////
bool OnOffBrightnessBulb::getState(void) {
  return m_state;
}

bool OnOffBrightnessBulb::setState(bool p_state) {
  // checks if the given state is different from the actual state
  if (p_state == m_state)
    return false;
  
  m_state = p_state;

  return true;
}

///////////////////////////////////////////////////////////////////////////
//   BRIGHTNESS
///////////////////////////////////////////////////////////////////////////
uint8_t OnOffBrightnessBulb::getBrightness(void) {
  return m_brightness;
}

bool OnOffBrightnessBulb::setBrightness(uint8_t p_brightness) {
  // checks if the value is smaller, bigger or equal to the actual brightness value
  if (p_brightness < 0 || p_brightness > 255 || p_brightness == m_brightness)
    return false;

  // saves the new brightness value
  m_brightness = p_brightness;
  return true; 
}

