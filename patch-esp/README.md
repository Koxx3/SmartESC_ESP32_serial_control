## Preparation of ESP32 board
- Install Visual Code + Platform.io [tuto](https://platformio.org/install/ide?install=vscode)
- Clone this Github project
_______
- Patch Espressif Framework with patch folder, for that you must : 

copy the contents of the folder "patch-esp\espBLE" (2 files) and paste & Remplace to  :
- platformio\packages\framework-arduinoespressif32\libraries\BLE\src

copy the contents of the folder "patch-esp\Uart" (4 files) and paste & Remplace to :
- C:\Users\ \<username>\ \.platformio\packages\framework-arduinoespressif32\cores\esp32

______
- Erase the ESP32
- Flash the ESP32
