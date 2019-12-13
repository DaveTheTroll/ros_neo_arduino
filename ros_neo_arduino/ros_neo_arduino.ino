// ROS NeoPixel Node

#include <ros.h>
#include <FastLED.h>
#include "ros_neo_arduino/NeoMode.h"
#include "ros_neo_arduino/NeoLeds.h"

#define LED_PIN     5
#define COLOR_ORDER GRB
#define CHIPSET     WS2811
#define NUM_LEDS    60
#define BRIGHTNESS  200
CRGB leds[NUM_LEDS];

ros::NodeHandle  nh;
long lastUpdate;

uint8_t mode;
uint32_t mode_param;

char buf[32];

const int corner_offset = 8;
CRGB off = CRGB(0,0,0);

void OnNeoMode(const ros_neo_arduino::NeoMode& msg)
{
  mode = msg.mode;
  mode_param = msg.param;
  
  switch(mode)
  {
    case ros_neo_arduino::NeoMode::CLEAR:
      nh.loginfo("Neo Mode: CLEAR");
      {
        for(int i=0; i<NUM_LEDS; i++)
        {
          leds[i] = off;
        }
        FastLED.show();
      }
    break;
    case ros_neo_arduino::NeoMode::STATIC:
      nh.loginfo("Neo Mode: STATIC");
      break;
    case ros_neo_arduino::NeoMode::HAZARD:
      nh.loginfo("Neo Mode: HAZARD");
      break;
    case ros_neo_arduino::NeoMode::CHASE:
      nh.loginfo("Neo Mode: CHASE");
      break;
  }
}

void OnNeoLeds(const ros_neo_arduino::NeoLeds& msg)
{
  nh.loginfo("Neo LEDs");
  mode = ros_neo_arduino::NeoMode::STATIC;

  if (msg.index < NUM_LEDS)
  {
    leds[msg.index] = CRGB(msg.r, msg.g, msg.b);
    FastLED.show();
    sprintf(buf, "L:%d (%d,%d,%d)", (int)msg.index, (int)msg.r, (int)msg.g, (int)msg.b);
    nh.loginfo(buf);
  }
  else
  {
    nh.loginfo("Neo LEDs: Out of range");
  }
}

ros::Subscriber<ros_neo_arduino::NeoMode> subMods("neo_mode", &OnNeoMode );
ros::Subscriber<ros_neo_arduino::NeoLeds> subLeds("neo_leds", &OnNeoLeds );

void setup()
{
  mode = ros_neo_arduino::NeoMode::HAZARD;
  lastUpdate = millis();
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness( BRIGHTNESS );
  
  nh.initNode();
  nh.subscribe(subMods);
  nh.subscribe(subLeds);
}

void loop()
{
  nh.spinOnce();
  checkMode();
  delay(1);
}

void checkMode()
{
  switch(mode)
  {
    case ros_neo_arduino::NeoMode::HAZARD:
      {
        static bool on = false;
        long now = millis();
        if (now - lastUpdate > 400)
        {
          lastUpdate = now;
          on = !on;
          CRGB color = on ? CRGB(255, 64, 0) : CRGB(0,0,0);
          for(int i = 0; i < 4; i++)
            for (int j = -2; j < 2; j++)
            {
              int k = ((NUM_LEDS * i / 4) + corner_offset + j + NUM_LEDS) % NUM_LEDS;
              leds[k] = color;
            }
          FastLED.show();
        }
      }
      break;
    case ros_neo_arduino::NeoMode::CHASE:
      {
        static int pos = 0;
        long now = millis();
        if (now - lastUpdate > 20)
        {
          lastUpdate = now;
          CRGB color = CRGB((mode_param >> 16) & 0xFF, (mode_param >> 8) & 0xFF, mode_param & 0xFF);
          leds[pos] = color;
          leds[(pos -3 + NUM_LEDS) % NUM_LEDS] = off;
          pos = (pos+1) % NUM_LEDS;
          FastLED.show();
        }
      }
      break;
  }
}
