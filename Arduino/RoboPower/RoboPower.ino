/* 
 * rosserial ADC Example
 * 
 * This is a poor man's Oscilloscope.  It does not have the sampling 
 * rate or accuracy of a commerical scope, but it is great to get
 * an analog value into ROS in a pinch.
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#define USE_USBCON

#include "U8glib.h"
#include <ros.h>
#include <rosserial_arduino/Adc.h>
#include <std_msgs/String.h>
#include <sensor_msgs/BatteryState.h>

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);  // I2C / TWI 

unsigned int test;

float  batteryArray[4]={3.628, 3.629, 3.515, 4};

ros::NodeHandle nh;

rosserial_arduino::Adc adc_msg;
ros::Publisher k("adc", &adc_msg);

sensor_msgs::BatteryState battery_msg;
ros::Publisher p("BatteryState", &battery_msg);

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "hello world!";

void draw(void) {
  // graphic commands to redraw the complete screen should be placed here  
  u8g.setFont(u8g_font_unifont);
  //u8g.setFont(u8g_font_osb21);


  u8g.setPrintPos(0, 10); 
  u8g.print("AN0: ");
  u8g.setPrintPos(u8g.getStrWidth("AN0:"), 10); 
  u8g.print(averageAnalog(0));

  u8g.setPrintPos(64, 10); 
  u8g.print("AN1:");
  u8g.setPrintPos(u8g.getStrWidth("AN1:")+64, 10); 
  u8g.print(averageAnalog(1));

  u8g.setPrintPos(0, 20); 
  u8g.print("AN2:");
  u8g.setPrintPos(u8g.getStrWidth("AN2:"), 20); 
  u8g.print(averageAnalog(2));

  u8g.setPrintPos(64, 20); 
  u8g.print("AN3:");
  u8g.setPrintPos(u8g.getStrWidth("AN3:")+64, 20); 
  u8g.print(averageAnalog(3));







  u8g.setPrintPos(0, 30); 
  u8g.print("AN4: ");
  u8g.setPrintPos(u8g.getStrWidth("AN4:"), 30); 
  u8g.print(averageAnalog(4));

  u8g.setPrintPos(64, 30); 
  u8g.print("AN5:");
  u8g.setPrintPos(u8g.getStrWidth("AN5:")+64, 30); 
  u8g.print(averageAnalog(5));

  u8g.setPrintPos(0, 40); 
  u8g.print("AN6:");
  u8g.setPrintPos(u8g.getStrWidth("AN6:"), 40); 
  u8g.print(averageAnalog(6));

  u8g.setPrintPos(64, 40); 
  u8g.print("AN7:");
  u8g.setPrintPos(u8g.getStrWidth("AN7:")+64, 40); 
  u8g.print(averageAnalog(7));








  
  
  // u8g.drawStr( 0, 22, "Hello World!");
  // u8g.drawStr( 0, 44, "Hello World!");
}

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
 test= 0;
  nh.advertise(p);
  nh.advertise(k);
  nh.advertise(chatter);

  // LCD setup
  // flip screen, if required
  // u8g.setRot180();
  
  // set SPI backup if required
  //u8g.setHardwareBackup(u8g_backup_avr_spi);

  // assign default color value
  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
    u8g.setColorIndex(255);     // white
  }
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
    u8g.setColorIndex(3);         // max intensity
  }
  else if ( u8g.getMode() == U8G_MODE_BW ) {
    u8g.setColorIndex(1);         // pixel on
  }
  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255,255,255);
  }
  
  pinMode(8, OUTPUT);
  // END LCD SETUP
}

//We average the analog reading to elminate some of the noise
int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<4; i++) v+= analogRead(pin);
  return v/4;
}

long adc_timer;

void loop()
{
    // picture loop
  u8g.firstPage();  
  do {
    draw();
  } while( u8g.nextPage() );
  
  battery_msg.voltage = 23.221;         //  Voltage in Volts (Mandatory)
  battery_msg.current = -2.900;          //  Current in Ampere - Negative when discharging (A)  (If unmeasured NaN)
  battery_msg.charge = 0.241;           //  Current charge in Ampere - Negative when discharging (A)  (If unmeasured NaN)
  battery_msg.capacity = 3.600;          //  Capacity in Ah (last full capacity)  (If unmeasured NaN)
  battery_msg.design_capacity= 5.000;   //  Capacity in Ah (design capacity)  (If unmeasured NaN)
  battery_msg.percentage= 0.93;         //  Charge percentage on 0 to 1 range  (If unmeasured NaN)
  battery_msg.power_supply_status= battery_msg.POWER_SUPPLY_STATUS_DISCHARGING; //  The charging status as reported. Values defined above
  battery_msg.power_supply_health= battery_msg.POWER_SUPPLY_HEALTH_GOOD;        //  The battery health metric. Values defined above
  battery_msg.power_supply_technology= battery_msg.POWER_SUPPLY_TECHNOLOGY_LION;  // The battery chemistry. Values defined above
  battery_msg.present=true;                 //  True if the battery is present
  battery_msg.cell_voltage_length= 4;       // Number of cell in pack
  battery_msg.st_cell_voltage= 300;
  battery_msg.cell_voltage = batteryArray;  //  An array of individual cell voltages for each cell in the pack, If individual voltages unknown but number of cells known set each to NaN
  battery_msg.location="robot";             //  The location into which the battery is inserted. (slot number or plug)
  battery_msg.serial_number="123456790";    //  The best approximation of the battery serial number


  str_msg.data = hello;

  
  adc_msg.adc0 = averageAnalog(0);
  adc_msg.adc1 = averageAnalog(1);
  adc_msg.adc2 = averageAnalog(2);
  adc_msg.adc3 = averageAnalog(3);
  adc_msg.adc4 = averageAnalog(4);
  adc_msg.adc5 = averageAnalog(5);
      
  p.publish(&battery_msg);    // Publish BatteryState message
  nh.spinOnce();

  k.publish(&adc_msg);        // Publish Adc related message
  nh.spinOnce();
  
  chatter.publish(&str_msg);  // Publish an "Hello World" debug message
  nh.spinOnce();

  
}


/*
      enum { POWER_SUPPLY_STATUS_UNKNOWN =  0 };
      enum { POWER_SUPPLY_STATUS_CHARGING =  1 };
      enum { POWER_SUPPLY_STATUS_DISCHARGING =  2 };
      enum { POWER_SUPPLY_STATUS_NOT_CHARGING =  3 };
      enum { POWER_SUPPLY_STATUS_FULL =  4 };
      enum { POWER_SUPPLY_HEALTH_UNKNOWN =  0 };
      enum { POWER_SUPPLY_HEALTH_GOOD =  1 };
      enum { POWER_SUPPLY_HEALTH_OVERHEAT =  2 };
      enum { POWER_SUPPLY_HEALTH_DEAD =  3 };
      enum { POWER_SUPPLY_HEALTH_OVERVOLTAGE =  4 };
      enum { POWER_SUPPLY_HEALTH_UNSPEC_FAILURE =  5 };
      enum { POWER_SUPPLY_HEALTH_COLD =  6 };
      enum { POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE =  7 };
      enum { POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE =  8 };
      enum { POWER_SUPPLY_TECHNOLOGY_UNKNOWN =  0 };
      enum { POWER_SUPPLY_TECHNOLOGY_NIMH =  1 };
      enum { POWER_SUPPLY_TECHNOLOGY_LION =  2 };
      enum { POWER_SUPPLY_TECHNOLOGY_LIPO =  3 };
      enum { POWER_SUPPLY_TECHNOLOGY_LIFE =  4 };
      enum { POWER_SUPPLY_TECHNOLOGY_NICD =  5 };
      enum { POWER_SUPPLY_TECHNOLOGY_LIMN =  6 };
 */
