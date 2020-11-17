# Risearch Documentation
## Contributor 
agungpambudi55 <agung.pambudi5595@gmail.com>

## The Third Week of November 2020
### Secondary Serial
Edit file Configuration.h on line 112 (MARLIN-2.0.X/Marlin/Configuration.h)
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
![Gambar][gambar-url]

## The Second Week of November 2020
### Directory File
```
MARLIN-2.0.X
|--Marlin
|  |--lib
|  |  |--readme.txt
|  |--src
|  |  |--core
|  |  |  |--boards.h
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
Edit file Configuration.h on line 129 (MARLIN-2.0.X/Marlin/Configuration.h) and choose the name from boards.h (MARLIN-2.0.X/Marlin/src/core/boards.h) that matches your setup
```
#ifndef MOTHERBOARD
  #define MOTHERBOARD <name-board>
#endif
```

Edit file platformio.ini on line 21 (MARLIN-2.0.X/platformio.ini)
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

<!-- MARKDOWN LINKS -->
[gambar-url]: https://gitlab.com/widyarobotics/3dcp/research-marlin/-/raw/master/screenshoot/parsing%20read%20write%20ser0%20ser1.png
