# Issues

#### 1. LoRa libraries

After updating the ESP32 libraries (as suggested by the Arduino IDE), the LoRa related programs won't compile anymore.

Example console output when attempting to compile or upload the official "LoRa" example code provided by Heltec:
> La libreria Heltec ESP32 Dev-Boards è stata dichiarata precompilata:
> Utilizzo della libreria precompilata in C:\Users\lnrdm\Documents\Arduino\libraries\Heltec_ESP32_Dev-Boards\src\esp32s3
> C:\Users\lnrdm\Documents\Arduino\libraries\Heltec_ESP32_Dev-Boards\src\driver\sx1262-board.c: In function 'SX126xWaitOnBusy':
> C:\Users\lnrdm\Documents\Arduino\libraries\Heltec_ESP32_Dev-Boards\src\driver\sx1262-board.c:82:17: error: implicit declaration of function 'lora_printf'; did you mean 'log_printf'? [-Wimplicit-function-declaration]
>    82 |                 lora_printf("spi timeout\r\n");
>       |                 ^~~~~~~~~~~
>       |                 log_printf
> C:\Users\lnrdm\Documents\Arduino\libraries\Heltec_ESP32_Dev-Boards\src\driver\sx126x.c: In function 'sx126xSleep':
> C:\Users\lnrdm\Documents\Arduino\libraries\Heltec_ESP32_Dev-Boards\src\driver\sx126x.c:247:5: error: implicit declaration of function 'delay'; did you mean 'Delay'? [-Wimplicit-function-declaration]
>   247 |     delay( 2 );
>       |     ^~~~~
>       |     Delay
> C:\Users\lnrdm\Documents\Arduino\libraries\Heltec_ESP32_Dev-Boards\src\driver\sx126x.c: In function 'SX126xGetPacketStatus':
> C:\Users\lnrdm\Documents\Arduino\libraries\Heltec_ESP32_Dev-Boards\src\driver\sx126x.c:731:13: error: implicit declaration of function 'memset' [-Wimplicit-function-declaration]
>   731 |             memset( pktStatus, 0, sizeof( PacketStatus_t ) );
>       |             ^~~~~~
> C:\Users\lnrdm\Documents\Arduino\libraries\Heltec_ESP32_Dev-Boards\src\driver\sx126x.c:9:1: note: 'memset' is defined in header '<string.h>'; this is probably fixable by adding '#include <string.h>'
>     8 | #include "esp_sleep.h"
>   +++ |+#include <string.h>
>     9 | /*!
> exit status 1
> 
> Compilation error: exit status 1

#### 2. Adaptive sampling

Due to buffer switching, the FFT gets dirty values and the computed peak frequency is wrong, converging to 0 after some time.
The issue is caused probably by concurrent buffer access and incorrect race conditions handling. The exact issue and a posisble
solution have yet to be found.