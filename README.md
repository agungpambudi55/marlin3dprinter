# Risearch Documentation
## Contributor 
Agung Pambudi - agungpambudi55 <agung.pambudi5595@gmail.com>

## The Four Week of November 2020
RepRapDiscount FULL GRAPHIC Smart Controller (https://reprap.org/wiki/RepRapDiscount_Full_Graphic_Smart_Controller) Configuration.h on line 2000
```
// LCD12864 Simulator
#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER
```

Setup LCD on line 1440 (Marlin-2.0.x\Marlin\Configuration_adv.h)
```
#define LCD_SERIAL_PORT 3
#define LCD_BAUDRATE 115200
```

Machine name on line 146 (Marlin-2.0.x\Marlin\Configuration.h)
```
#define CUSTOM_MACHINE_NAME "WIDYA 3DCP"
```

Machine name on line 75 (Marlin-2.0.x\Marlin\src\inc\Version.h)
```
#ifndef MACHINE_NAME
  #define MACHINE_NAME "WIDYA 3DCP"
#endif
```

Create new serial port for communication with the host
### Board mega2560
Configuration.h (Marlin-2.0.x\Marlin\Configuration.h) on line 115
```
#define SERIAL_PORT_3 2
```

HAL.h (Marlin-2.0.x\Marlin\src\HAL\AVR\HAL.h) on line 106
```
#ifdef SERIAL_PORT_3
  #if !WITHIN(SERIAL_PORT_3, -1, 3)
    #error "SERIAL_PORT_3 must be from -1 to 3. Please update your configuration."
  #endif
  #define MYSERIAL2 customizedSerial3
#endif
```

MarlinSerial.cpp (Marlin-2.0.x\Marlin\src\HAL\AVR\MarlinSerial.cpp) on line 749
```
#ifdef SERIAL_PORT_3
  // Hookup ISR handlers
  ISR(SERIAL_REGNAME(USART, SERIAL_PORT_3, _RX_vect)) {
    MarlinSerial<MarlinSerialCfg<SERIAL_PORT_3>>::store_rxd_char();
  }

  ISR(SERIAL_REGNAME(USART, SERIAL_PORT_3, _UDRE_vect)) {
    MarlinSerial<MarlinSerialCfg<SERIAL_PORT_3>>::_tx_udr_empty_irq();
  }

  // Preinstantiate
  template class MarlinSerial<MarlinSerialCfg<SERIAL_PORT_3>>;

  // Instantiate
  MarlinSerial<MarlinSerialCfg<SERIAL_PORT_3>> customizedSerial3;
#endif
```

MarlinSerial.h (Marlin-2.0.x\Marlin\src\HAL\AVR\MarlinSerial.h) on line 202
```
#ifdef SERIAL_PORT_3
  extern MarlinSerial<MarlinSerialCfg<SERIAL_PORT_3>> customizedSerial3;
#endif
```

MarlinCore.cpp (Marlin-2.0.x\Marlin\src\MarlinCore.cpp) on setup function
```
MYSERIAL2.begin(57600);
```

![Gambar][gambar-3-url]

### Board LPC1769
HAL.h (Marlin-2.0.x\Marlin\src\HAL\LPC1768\HAL.h) on line 89
```
#ifdef SERIAL_PORT_3
  #if SERIAL_PORT_3 == -1
    #define MYSERIAL2 UsbSerial
  #elif WITHIN(SERIAL_PORT_3, 0, 3)
    #define MYSERIAL2 MSERIAL(SERIAL_PORT_3)
  #else
    #error "SERIAL_PORT_3 must be from -1 to 3. Please update your configuration."
  #endif
#endif
```

## The Third Week of November 2020
### Secondary Serial
Edit file Configuration.h on line 112 (Marlin-2.0.X/Marlin/Configuration.h)
```
#define SERIAL_PORT_2 <serial-port>
```

Add this to setup function in MarlinCore.cpp file (baudrate <= 57600)
```
MYSERIAL1.begin(<baudrate>);
```

Test the print serial to loop function in MarlinCore.cpp file
```
MYSERIAL1.println("Bismillahirrahmanirrahim");

if (MYSERIAL1.available() > 0) {
  char inChar = (char)MYSERIAL1.read();
  MYSERIAL1.println(inChar);
}
```

### Create Communication Read and Write to Other Boards
```
unsigned char dataRX[4], dataRX_sum = 4;
int headerFind = 0, indexData = 0;

while(MYSERIAL1.available()){
  char inChar = (char)MYSERIAL1.read();

  if(headerFind == 0 && inChar == 'F'){ 
    headerFind = 1;
    MYSERIAL1.print(inChar);
    MYSERIAL1.println(" < first header"); 
  }else if(headerFind == 1 && inChar == 'F'){
    headerFind = 2; 
    MYSERIAL1.print(inChar);
    MYSERIAL1.println(" < second header");
  }else if(headerFind == 2){
    dataRX[indexData] = (int)inChar;
    indexData++;
    if(indexData >= dataRX_sum){ headerFind = indexData = 0; }
  }else { headerFind = indexData = 0; }
}
```

![Gambar][gambar-1-url]

Source code for Marlin
```
//### mysourcecode
unsigned int dataSensor1, dataSensor2, dataSensor3, dataSensor4, dataSensor5;
unsigned char dataRX[10], dataRXsum = 10;
int headerFind = 0, indexData = 0;

void processDataSensor(){
  dataRX[0] = dataRX[0] & 0x00ff;
  dataRX[1] = dataRX[1] << 8;
  dataRX[2] = dataRX[2] & 0x00ff;
  dataRX[3] = dataRX[3] << 8;
  dataRX[4] = dataRX[4] & 0x00ff;
  dataRX[5] = dataRX[5] << 8;
  dataRX[6] = dataRX[6] & 0x00ff;
  dataRX[7] = dataRX[7] << 8;
  dataRX[8] = dataRX[8] & 0x00ff;
  dataRX[9] = dataRX[9] << 8;

  dataSensor1 = dataRX[0] | dataRX[1];
  dataSensor2 = dataRX[2] | dataRX[3];
  dataSensor3 = dataRX[4] | dataRX[5];
  dataSensor4 = dataRX[6] | dataRX[7];
  dataSensor5 = dataRX[8] | dataRX[9];

  SERIAL_ECHO("dataSensor1 ");
  SERIAL_ECHOLN(dataSensor1);

  SERIAL_ECHO("dataSensor2 ");
  SERIAL_ECHOLN(dataSensor2);

  SERIAL_ECHO("dataSensor3 ");
  SERIAL_ECHOLN(dataSensor3);

  SERIAL_ECHO("dataSensor4 ");
  SERIAL_ECHOLN(dataSensor4);

  SERIAL_ECHO("dataSensor5 ");
  SERIAL_ECHOLN(dataSensor5);
}

void receiveDataSensor() {
  while(MYSERIAL1.available()){
    char inChar = (char)MYSERIAL1.read();
    SERIAL_ECHOLN(inChar);
    MYSERIAL1.println(inChar);
  
    if(headerFind == 0 && inChar == 'F'){ headerFind = 1; }
    else if(headerFind == 1 && inChar == 'F'){ headerFind = 2; }
    else if(headerFind == 2){      
      dataRX[indexData] = (int)inChar;
      indexData++;
      if(indexData >= dataRXsum){ headerFind = indexData = 0; }
    }else { headerFind = indexData = 0; }
  }
}
//### mysourcecode
```

Source code format tranfer data for sensor extruder
```
unsigned char dataTX[10];
unsigned int dataTXSum = 10;

void transmitDataSensor(int dataSensor1, int dataSensor2, int dataSensor3, int dataSensor4, int dataSensor5) {
  dataTX[0] = dataSensor1 & 0x00ff;
  dataTX[1] = dataSensor1 >> 8;
  dataTX[2] = dataSensor2 & 0x00ff;
  dataTX[3] = dataSensor2 >> 8;
  dataTX[4] = dataSensor3 & 0x00ff;
  dataTX[5] = dataSensor3 >> 8;
  dataTX[6] = dataSensor4 & 0x00ff;
  dataTX[7] = dataSensor4 >> 8;
  dataTX[8] = dataSensor5 & 0x00ff;
  dataTX[9] = dataSensor5 >> 8;

  Serial.print("FF");
  Serial.write((byte*)dataTX, dataTXSum);
}

void setup() {
  Serial.begin(57600);
}

void loop() {
  transmitDataSensor(51,61,95,96,15);
  delay(100);
}
```
![Gambar][gambar-2-url]

## The Second Week of November 2020
### Directory File
```
Marlin-2.0.X
|--Marlin
|  |--lib
|  |  |--readme.txt
|  |--src
|  |  |--core
|  |  |  |--boards.h (name of boards)
|  |  |  |--...
|  |  |--gcode
|  |  |--inc
|  |  |--libs
|  |  |--pins
|  |  |--feature
|  |  |--HAL
|  |  |--lcd
|  |  |--module
|  |  |--sd
|  |  |--MarlinCore.cpp (source main / core program file)
|  |  |--MarlinCore.h (header main / core program file)
|  |--Configuration_adv.cpp (advanced settings file)
|  |--Configuration.h (general settings file)
|  |--...
|--platformio.ini (platformio configuration file)
```

### Build Setup
Edit file Configuration.h on line 129 (Marlin-2.0.X/Marlin/Configuration.h) and choose the name from boards.h (Marlin-2.0.X/Marlin/src/core/boards.h) that matches your setup
```
#ifndef MOTHERBOARD
  #define MOTHERBOARD <name-board>
#endif
```

Edit file platformio.ini on line 21 (Marlin-2.0.X/platformio.ini)
```
default_envs = <name-environment>
```

### Test Print
MarlinCore.cpp
```
void loop() {
  idle();
  SERIAL_ECHOLN("Bismillahirrahmanirrahim");
  delay(1000);
}
```

### BBT TFT70
Setting Connection Baudrate 250000

### Link
https://marlinfw.org
https://github.com/bigtreetech/BIGTREETECH-SKR-V1.3
https://3dwork.io/en/complete-guide-skr-v1-4-and-tmc2209
https://github.com/bigtreetech/BIGTREETECH-TouchScreenHardware
https://github.com/bigtreetech/BIGTREETECH-TouchScreenFirmware

<!-- MARKDOWN LINKS -->
[gambar-1-url]: https://gitlab.com/widyarobotics/3dcp/research-marlin/-/raw/master/screenshoot/parsing%20read%20write%20ser0%20ser1.png
[gambar-2-url]: https://gitlab.com/widyarobotics/3dcp/research-marlin/-/raw/master/screenshoot/read%20write%20parsing%20packet.png
[gambar-3-url]: https://gitlab.com/widyarobotics/3dcp/research-marlin/-/raw/master/screenshoot/new%20serial%20comm%20port.png
