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

// 128x64 oled LCD config
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);  // I2C / TWI 

unsigned int test;
float  CellVoltage[4]={3.628, 3.629, 3.515, 4};
long adc_timer;
int vMotor1, vMotor2,vBattery, vMainSupply, v24vFilter1, v24vFilter2, v5vSupply, vMainboardSupply;
ros::NodeHandle nh;

rosserial_arduino::Adc adc_msg;
ros::Publisher k("adc", &adc_msg);

sensor_msgs::BatteryState battery_msg;
ros::Publisher p("BatteryState", &battery_msg);

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "hello world!";


#define VCC_MILLIVOLT                            3300l    // 3.3V as Vref
#define RES_PARTITORE_INPUT_150K0               15000l  //  150kohm
#define RES_PARTITORE_INPUT_47K0                 4700l  //  150kohm
#define RES_PARTITORE_MASSA                      1200l  //  12kohm

void setup()
{ 
  // Variable's setup
  test= 0;

  // ROS interface setup
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(p);
  nh.advertise(k);
  nh.advertise(chatter);

  // LCD setup
  // flip screen, if required
  //u8g.setRot180();

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

  // Arduino DUE 12bit Analog input mode
  analogReadResolution(12);
}





void loop()
{
    // picture loop
  u8g.firstPage();  
  do {
    draw();
  } while( u8g.nextPage() );

  AalogConversion();
  
  battery_msg.voltage = vBattery;         //  Voltage in Volts (Mandatory)
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
  battery_msg.cell_voltage = CellVoltage;  //  An array of individual cell voltages for each cell in the pack, If individual voltages unknown but number of cells known set each to NaN
  battery_msg.location="robot";             //  The location into which the battery is inserted. (slot number or plug)
  battery_msg.serial_number="123456790";    //  The best approximation of the battery serial number

  str_msg.data = hello;

  
  adc_msg.adc0 = vMotor1; //averageAnalog(A0);
  adc_msg.adc1 = vMotor2; //averageAnalog(A1);
  adc_msg.adc2 = v24vFilter1; //averageAnalog(A8);
  adc_msg.adc3 = v24vFilter2; //averageAnalog(A9);
  adc_msg.adc4 = v5vSupply; //averageAnalog(A10);
  adc_msg.adc5 = vMainboardSupply; //averageAnalog(A11);
      
  p.publish(&battery_msg);    // Publish BatteryState message
  nh.spinOnce();

  k.publish(&adc_msg);        // Publish Adc related message
  nh.spinOnce();
  
  chatter.publish(&str_msg);  // Publish an "Hello World" debug message
  nh.spinOnce();
  
}

void AalogConversion(void){
  long mV;
  //int vMotor1, vMotor2,vBattery, vMainSupply, v24vFilter1, v24vFilter2, v5vSupply, vMainboardSupply;
  mV = ((long)averageAnalog(A0)* VCC_MILLIVOLT) / 4095;
  vMotor1  = (signed short)(((mV * (RES_PARTITORE_INPUT_150K0 + RES_PARTITORE_MASSA)) / RES_PARTITORE_MASSA));
  
  mV = ((long)averageAnalog(A1)* VCC_MILLIVOLT) / 4095;
  vMotor2 = (signed short)(((mV * (RES_PARTITORE_INPUT_150K0 + RES_PARTITORE_MASSA)) / RES_PARTITORE_MASSA));
  //adc_msg.mvhbridge1 = adc_msg.adc1;
  
  mV = ((long)averageAnalog(A2)* VCC_MILLIVOLT) / 4095;
  vBattery = (signed short)(((mV * (RES_PARTITORE_INPUT_150K0 + RES_PARTITORE_MASSA)) / RES_PARTITORE_MASSA));

  mV = ((long)averageAnalog(A3)* VCC_MILLIVOLT) / 4095;
  vMainSupply = (signed short)(((mV * (RES_PARTITORE_INPUT_150K0 + RES_PARTITORE_MASSA)) / RES_PARTITORE_MASSA));
  
  mV = ((long)averageAnalog(A4)* VCC_MILLIVOLT) / 4095;
  v24vFilter1 = (signed short)(((mV * (RES_PARTITORE_INPUT_150K0 + RES_PARTITORE_MASSA)) / RES_PARTITORE_MASSA));
    
  mV = ((long)averageAnalog(A5)* VCC_MILLIVOLT) / 4095;
  v24vFilter2 = (signed short)(((mV * (RES_PARTITORE_INPUT_150K0 + RES_PARTITORE_MASSA)) / RES_PARTITORE_MASSA));
    
  mV = ((long)averageAnalog(A6)* VCC_MILLIVOLT) / 4095;
  v5vSupply = (signed short)(((mV * (RES_PARTITORE_INPUT_47K0 + RES_PARTITORE_MASSA)) / RES_PARTITORE_MASSA));
    
  mV = ((long)averageAnalog(A7)* VCC_MILLIVOLT) / 4095;
  vMainboardSupply = (signed short)(((mV * (RES_PARTITORE_INPUT_47K0 + RES_PARTITORE_MASSA)) / RES_PARTITORE_MASSA));
}

void draw(void) {
  // graphic commands to redraw the complete screen should be placed here  
  //  u8g.setFont(u8g_font_unifont);  // 10 Pixel Height
  u8g.setFont(u8g_font_8x13);  // 9 Pixel Height

  u8g.setPrintPos(0, 10); 
  u8g.print("Motor1: ");
  u8g.setPrintPos(u8g.getStrWidth("Motor1: "), 10); 
  u8g.print(vMotor1);
  u8g.print("mV");

  u8g.setPrintPos(0, 20); 
  u8g.print("Motor2: ");
  u8g.setPrintPos(u8g.getStrWidth("Motor2: "), 20); 
  u8g.print(vMotor2);
  u8g.print("mV");

  u8g.setPrintPos(0, 30); 
  u8g.print("Batt. : ");
  u8g.setPrintPos(u8g.getStrWidth("Batt. : "), 30); 
  u8g.print(vBattery);
  u8g.print("mV");

  u8g.setPrintPos(0, 40); 
  u8g.print("Supply: ");
  u8g.setPrintPos(u8g.getStrWidth("Supply: "), 40); 
  u8g.print(vMainSupply);
  u8g.print("mV");

  u8g.setPrintPos(0, 50); 
  u8g.print("5V    : ");
  u8g.setPrintPos(u8g.getStrWidth("5V    : "), 50); 
  u8g.print(v5vSupply);
  u8g.print("mV");

  u8g.setPrintPos(0, 60); 
  u8g.print("CPU   : ");
  u8g.setPrintPos(u8g.getStrWidth("CPU   : "), 60); 
  u8g.print(vMainboardSupply);
  u8g.print("mV");
  
  /*u8g.setPrintPos(0, 15); 
  u8g.print("AN8: ");
  u8g.setPrintPos(u8g.getStrWidth("AN8:  "), 15); 
  u8g.print(averageAnalog(A8));

  u8g.setPrintPos(0, 30); 
  u8g.print("AN9:");
  u8g.setPrintPos(u8g.getStrWidth("AN9:  "), 30); 
  u8g.print(averageAnalog(A9));

  u8g.setPrintPos(0, 45); 
  u8g.print("AN10:");
  u8g.setPrintPos(u8g.getStrWidth("AN10: "), 45); 
  u8g.print(averageAnalog(A10));

  u8g.setPrintPos(0, 60); 
  u8g.print("AN11:");
  u8g.setPrintPos(u8g.getStrWidth("AN11: "), 60); 
  u8g.print(averageAnalog(A11));*/
}

//We average the analog reading to elminate some of the noise
int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<4; i++) v+= analogRead(pin);
  return v/4;
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
