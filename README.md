Connections

Raspberry Pi Pico                            ENC28J60
SIGNAL NAME    GPIO   PIN Nª
MISO            16      21          <---        SO
MOSI            19      25          --->        SI
CLK             18      24          --->        SCK
ENC_RST         21      27          --->        RESET
ENC_CS          17      22          --->        CS

IRQ             22      29          <---        INT             Unused (pooling in a worker_thread)
3V3(OUT)                36          --->        VCC
GND                     38          --->        GND
