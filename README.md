# Risearch Documentation
## Contributor
Agung Pambudi - agungpambudi55 *agung.pambudi5595@gmail.com*

## The Second Week of December 2020
### Resume G-Code
- https://github.com/MarlinFirmware/Marlin/issues/10549

* For the technically-minded, G-code line endings are Unix Line Endings (\n), but will accept Windows Line Endings (\r\n), so you should not need to worry about converting between the two, but it is best practice to use Unix Line Endings where possible.

* Example of a communication error with resend request
`>>>` are lines sent from the host to the RepRap machine, `<<<` are lines sent from the RepRap machine to the host.
```
>>> N66555 G1 X131.338 Y133.349 E0.0091*91
<<< ok
>>> N66556 G1 X131.574 Y133.428 E0.0046*92
<<< Error:checksum mismatch, Last Line: 66555
<<< Resend: 66556
<<< ok
>>> N66556 G1 X131.574 Y133.428 E0.0046*92
<<< ok
```

### Board LPC1789 for Marlin
* Add this declaration header function in file queue.h on private of class GCodeQueue
```
static void get_i2c_commands();
```

* Add func get_i2c_commands() in file queue.cpp on func get_available_commands()
```
void GCodeQueue::get_available_commands() {
  get_serial_commands();
  //### mysourcecode
  get_i2c_commands();
  //### mysourcecode
  TERN_(SDSUPPORT, get_sdcard_commands());
}
```

* Add this func get_i2c_commands in file queue.cpp after func get_serial_commands()
```
void GCodeQueue::get_i2c_commands() {
  static char serial_line_buffer[NUM_SERIAL][MAX_CMD_SIZE];
  static uint8_t serial_input_state[NUM_SERIAL] = { PS_NORMAL };

  i2c.address(0x63);
  i2c.request(32);

  while(1<Wire.available() && length < BUFSIZE) {
    int i = 0;

    const int c = Wire.read();
    if (c < 0) continue;

    const char serial_char = c;

    if (ISEOL(serial_char)) {

      // Reset our state, continue if the line was empty
      if (process_line_done(serial_input_state[i], serial_line_buffer[i], serial_count[i]))
        continue;

      char* command = serial_line_buffer[i];

      while (*command == ' ') command++;                   // Skip leading spaces
      char *npos = (*command == 'N') ? command : nullptr;  // Require the N parameter to start the line

      if (npos) {

        bool M110 = strstr_P(command, PSTR("M110")) != nullptr;

        if (M110) {
          char* n2pos = strchr(command + 4, 'N');
          if (n2pos) npos = n2pos;
        }

        const long gcode_N = strtol(npos + 1, nullptr, 10);

        if (gcode_N != last_N[i] + 1 && !M110)
          return gcode_line_error(PSTR(STR_ERR_LINE_NO), i);

        char *apos = strrchr(command, '*');
        if (apos) {
          uint8_t checksum = 0, count = uint8_t(apos - command);
          while (count) checksum ^= command[--count];
          if (strtol(apos + 1, nullptr, 10) != checksum)
            return gcode_line_error(PSTR(STR_ERR_CHECKSUM_MISMATCH), i);
        }
        else
          return gcode_line_error(PSTR(STR_ERR_NO_CHECKSUM), i);

        last_N[i] = gcode_N;
      }
      #if ENABLED(SDSUPPORT)
        // Pronterface "M29" and "M29 " has no line number
        else if (card.flag.saving && !is_M29(command))
          return gcode_line_error(PSTR(STR_ERR_NO_CHECKSUM), i);
      #endif

      if (IsStopped()) {
        char* gpos = strchr(command, 'G');
        if (gpos) {
          switch (strtol(gpos + 1, nullptr, 10)) {
            case 0: case 1:
            #if ENABLED(ARC_SUPPORT)
              case 2: case 3:
            #endif
            #if ENABLED(BEZIER_CURVE_SUPPORT)
              case 5:
            #endif
              PORT_REDIRECT(i);                      // Reply to the serial port that sent the command
              SERIAL_ECHOLNPGM(STR_ERR_STOPPED);
              LCD_MESSAGEPGM(MSG_STOPPED);
              break;
          }
        }
      }

      #if DISABLED(EMERGENCY_PARSER)
        // Process critical commands early
        if (strcmp_P(command, PSTR("M108")) == 0) {
          wait_for_heatup = false;
          TERN_(HAS_LCD_MENU, wait_for_user = false);
        }
        if (strcmp_P(command, PSTR("M112")) == 0) kill(M112_KILL_STR, nullptr, true);
        if (strcmp_P(command, PSTR("M410")) == 0) quickstop_stepper();
      #endif

      #if defined(NO_TIMEOUTS) && NO_TIMEOUTS > 0
        last_command_time = ms;
      #endif

      // Add the command to the queue
      _enqueue(serial_line_buffer[i], true
        #if HAS_MULTI_SERIAL
          , i
        #endif
      );
    }
    else
      process_stream_char(serial_char, serial_input_state[i], serial_line_buffer[i], serial_count[i]);
  }
}
```

### Source code on other board
```
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin(0x63);             // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
}

void loop() {
  delay(100);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent(const uint8_t bytes) {
  Wire.write("M114\n"); // respond with message of 6 bytesx
}
```

## The First Week of December 2020
**Create communication I2C / TWI BUS**
Enabling this will allow you to send and receive I2C data from slave devices on the bus.

* Configuration_adv.h on line 2861
```
#define EXPERIMENTAL_I2CBUS
```

1. Example #1
This macro send the string "Marlin" to the slave device with address 0x63 (99). It uses multiple M260 commands with one B<base 10> arg.
```
M260 A99  ; Target slave address
M260 B77  ; M
M260 B97  ; a
M260 B114 ; r
M260 B108 ; l
M260 B105 ; i
M260 B110 ; n
M260 S1   ; Send the current buffer
```

2. Example #2
Request 6 bytes from slave device with address 0x63 (99).
```
M261 A99 B5
```

3. Example #3
Example serial output of a M261 request, echo:i2c-reply: from:99 bytes:5 data:hello


* Set a value from 8 to 127 to act as a slave
```
#define I2C_SLAVE_ADDRESS  8
```

* Example G-Code for set Feedrate
M221 - Set Flow Percentage: "M221 S<percent>"


## The Four Week of November 2020
- RepRapDiscount FULL GRAPHIC Smart Controller
Link : https://reprap.org/wiki/RepRapDiscount_Full_Graphic_Smart_Controller 
Uncomment this define, Configuration.h on line 2000
```
// LCD12864 Simulator
#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER
```

- Setup LCD on line 1440 (Marlin-2.0.x\Marlin\Configuration_adv.h)
```
#define LCD_SERIAL_PORT 3
#define LCD_BAUDRATE 115200
```

- Machine name, uncomment and write name between "" on line 146 (Marlin-2.0.x\Marlin\Configuration.h)
```
#define CUSTOM_MACHINE_NAME "WIDYA 3DCP"
```

- Machine name on line 75 (Marlin-2.0.x\Marlin\src\inc\Version.h)
```
#ifndef MACHINE_NAME
  #define MACHINE_NAME "WIDYA 3DCP"
#endif
```

### Create new serial port for communication with the host
**1.  Board mega2560**
- Configuration.h (Marlin-2.0.x\Marlin\Configuration.h) on line 115
```
#define SERIAL_PORT_3 2
```

- HAL.h (Marlin-2.0.x\Marlin\src\HAL\AVR\HAL.h) on line 106
```
#ifdef SERIAL_PORT_3
  #if !WITHIN(SERIAL_PORT_3, -1, 3)
    #error "SERIAL_PORT_3 must be from -1 to 3. Please update your configuration."
  #endif
  #define MYSERIAL2 customizedSerial3
#endif
```

- MarlinSerial.cpp (Marlin-2.0.x\Marlin\src\HAL\AVR\MarlinSerial.cpp) on line 749
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

- MarlinSerial.h (Marlin-2.0.x\Marlin\src\HAL\AVR\MarlinSerial.h) on line 202
```
#ifdef SERIAL_PORT_3
  extern MarlinSerial<MarlinSerialCfg<SERIAL_PORT_3>> customizedSerial3;
#endif
```

- MarlinCore.cpp (Marlin-2.0.x\Marlin\src\MarlinCore.cpp) on setup function
```
MYSERIAL2.begin(57600);
```

- Experimental Result
![Gambar][gambar-3-url]

**2. Board LPC1769**
- HAL.h (Marlin-2.0.x\Marlin\src\HAL\LPC1768\HAL.h) on line 89
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
**1. Secondary Serial**
- Edit file Configuration.h on line 112 (Marlin-2.0.X/Marlin/Configuration.h)
```
#define SERIAL_PORT_2 <serial-port>
```

- Add this to setup function in MarlinCore.cpp file (baudrate <= 57600)
```
MYSERIAL1.begin(<baudrate>);
```

- Test the print serial to loop function in MarlinCore.cpp file
```
MYSERIAL1.println("Bismillahirrahmanirrahim");

if (MYSERIAL1.available() > 0) {
  char inChar = (char)MYSERIAL1.read();
  MYSERIAL1.println(inChar);
}
```

**2. Create Communication Read and Write to Other Boards**
- Source code
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
- Experimental Result
![Gambar][gambar-1-url]

**3. Create communication read and write to other boards**
- Source code for Marlin
```
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
```

- Source code format transfer data for sensor extruder
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
- Experimental Result
![Gambar][gambar-2-url]

## The Second Week of November 2020
**1. Board**
- BTT SKR V1.4 Turbo
![Gambar][gambar-4-url]
![Gambar][gambar-5-url]

- BTT SKR V1.3
![Gambar][gambar-6-url]

**2. Directory File**
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

**3. Build Setup**
 - Edit file Configuration.h on line 129 (Marlin-2.0.X/Marlin/Configuration.h) and choose the name from boards.h (Marlin-2.0.X/Marlin/src/core/boards.h) that matches your setup
```
#ifndef MOTHERBOARD
  #define MOTHERBOARD <name-board>
#endif

; EXAMPLE 
; LPC1769
; #define MOTHERBOARD BOARD_BTT_SKR_V1_4_TURBO
; mega2560
; #define MOTHERBOARD BOARD_RAMPS_13_EFB
```

* Edit file platformio.ini on line 21 (Marlin-2.0.X/platformio.ini)
```
default_envs = <name-environment>

; EXAMPLE
; default_envs = LPC1769
; default_envs = mega2560
```

**4. Test Print**
MarlinCore.cpp
```
void loop() {
  idle();
  SERIAL_ECHOLN("Bismillahirrahmanirrahim");
  delay(1000);
}
```

**5. BBT TFT70**
Setting connection baudrate default 250000

### Link
* https://marlinfw.org
* https://github.com/bigtreetech/BIGTREETECH-SKR-V1.3
* https://3dwork.io/en/complete-guide-skr-v1-4-and-tmc2209
* https://github.com/bigtreetech/BIGTREETECH-TouchScreenHardware
* https://github.com/bigtreetech/BIGTREETECH-TouchScreenFirmware
* https://reprap.org/wiki/G-code

<!-- MARKDOWN LINKS -->
[gambar-1-url]: https://gitlab.com/widyarobotics/3dcp/research-marlin/-/raw/master/screenshoot/parsing%20read%20write%20ser0%20ser1.png
[gambar-2-url]: https://gitlab.com/widyarobotics/3dcp/research-marlin/-/raw/master/screenshoot/read%20write%20parsing%20packet.png
[gambar-3-url]: https://gitlab.com/widyarobotics/3dcp/research-marlin/-/raw/master/screenshoot/new%20serial%20comm%20port.png
[gambar-4-url]: https://gitlab.com/widyarobotics/3dcp/research-marlin/-/raw/master/screenshoot/BTT%20SKR%20V1.4%20TURBO.jpg
[gambar-5-url]: https://gitlab.com/widyarobotics/3dcp/research-marlin/-/raw/master/screenshoot/BTT%20SKR%20V1.4%20TURBO%20BACK.jpg
[gambar-6-url]: https://gitlab.com/widyarobotics/3dcp/research-marlin/-/raw/master/screenshoot/BTT%20SKR%20V1.3.jpg