// ROS NeoPixel Node

#include <ros.h>
#include <FastLED.h>
#include "ros_neo_arduino/NeoMode.h"

#define LED_PIN     5
#define COLOR_ORDER GRB
#define CHIPSET     WS2811
#define NUM_LEDS    60
#define BRIGHTNESS  200
CRGB leds[NUM_LEDS];

ros::NodeHandle  nh;

enum
{
  Static,
  Clear,
  Hazard
} mode;
long lastModeChange;

void OnNeoMode( const ros_neo_arduino::NeoMode& msg)
{
  nh.loginfo("Neo Mode");
}

ros::Subscriber<ros_neo_arduino::NeoMode> sub("neo_mode", &OnNeoMode );

void setup()
{
  mode = Hazard;
  lastModeChange = millis();
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness( BRIGHTNESS );
  
  nh.initNode();
  nh.subscribe(sub);
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
    case Hazard:
      {
        static bool on = false;
        long now = millis();
        if (now - lastModeChange > 400)
        {
          lastModeChange = now;
          on = !on;
          CRGB color = on ? CRGB(0, 255, 0) : CRGB(0,0,0);
          for(int i = 0; i < 4; i++)
            for (int j = -2; j < 2; j++)
            {
              int k = ((NUM_LEDS * i / 4) + j + NUM_LEDS) % NUM_LEDS;
              leds[k] = color;
            }
          FastLED.show();
        }
      }
      break;
  }
}
