/**
 * PINS
 */

#define LED                     LED_BUILTIN
#define CLOCK_TICK_INPUT        18
#define RW_PIN                  39

#define ADDRESS_LEN             16
#define DATA_LEN                8


typedef enum {
    READ = LOW,
    WRITE = HIGH
} mem_access_t;


//                                             LSB                                                        MSB
static const byte address_pins[ADDRESS_LEN] = {22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52};
static const byte data_pins[DATA_LEN]       = {23, 25, 27, 29, 31, 33, 35, 37};


/**
 *      0000 - 3000     RAM         (0100 - 01FF    STACK)
 *      6000 - 600F     I/O
 *      8000 - FFFF     ROM         (EFFC - EFFD    RESET VECTOR)
 */
#define TOT_MEM             0xFFFF
#define RAM_SIZE            0x0300
#define RAM_START_ADDRESS   0x0000
#define ROM_SIZE            0x1000
#define ROM_START_ADDRESS   (TOT_MEM - ROM_SIZE + 0x0001)

static byte RAM[RAM_SIZE];
static byte ROM[ROM_SIZE];
// static byte IO[0x600F - 0x6000];


void setup() {
    set_all_ram(0x00);
    set_all_rom(0xEA);

    // MANUAL CLOCK LED
    digitalWrite(LED, HIGH);
    pinMode(LED, OUTPUT);

    // TOGGLE CLOCK MODE BUTTON INTERRUPT
    pinMode(CLOCK_TICK_INPUT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CLOCK_TICK_INPUT), clock_tick_ISR, RISING);

    // MEMORY READ WRITE MODE INPUT
    pinMode(RW_PIN, INPUT);

    // INIT ADDRESS PINS
    for (int i = 0; i < ADDRESS_LEN; i++) {
        pinMode(address_pins[i], INPUT);
    }

    // INIT DATA PINS
    for (int i = 0; i < DATA_LEN; i++) {
        digitalWrite(LED, HIGH);
        pinMode(data_pins[i], OUTPUT);
    }

    // INIT SERIAL
    Serial.begin(115200);

    while (!Serial) ;     // wait for serial port to connect. Needed for native USB port only

    // Setup the
    ROM[rom_addr(0xFFFC)] = 0x00;
    ROM[rom_addr(0xFFFD)] = 0xFF;

    byte prog[] = { 0xa9, 0x01, 0x8d, 0x00, 0x02, 0xa9, 0x05, 0x8d, 0x01, 0x02, 0xa9, 0x08, 0x8d, 0x02, 0x02 };
    byte prog[] = { 0xA9, 0x01, 0x8D, 0x00, 0x02, 0xA9, 0x05, 0x8D, 0x01, 0x02, 0xA9, 0x08, 0x8D, 0x02, 0x02 };
    // byte prog[] = {0x01, 0xa9, 0x00, 0x8d, 0xa9, 0x02, 0x8d, 0x05, 0x02, 0x01, 0x08, 0xa9, 0x02, 0x8d, 0x00, 0x02};

    for (int i = 0; i < sizeof(prog); i++) {
        ROM[rom_addr(0xFF00 + i)] = prog[i];
    }
}


void loop() {

}


/**
   INTERRUPT HANDLERS
*/
void clock_tick_ISR() {
    uint16_t address = read_address();
    mem_access_t rw = read_rw();
    byte data;

    const char *mem;

    if (address > 0x7FFF) {
        // ROM
        mem = "ROM";

        switch (rw) {
        case READ:
            data = ROM[rom_addr(address)];
            write_data(data);
            break;
        }
    } else if (address < 0x3000) {
        // RAM
        mem = "RAM";

        switch (rw) {
        case READ:
            data = RAM[ram_addr(address)];
            write_data(data);
            break;

        case WRITE:
            data = read_data();
            RAM[ram_addr(address)] = data;
            break;
        }
    } else {
        // I/O and Other
        mem = "I/O";
    }


    static char buff[20];
    sprintf(buff, "%s   %04X   %c   %02X  |  %02X  %02X  %02X", mem, address, rw == READ ? 'R' : 'W', 
            data, RAM[ram_addr(0x0200)], RAM[ram_addr(0x0201)], RAM[ram_addr(0x0202)]);

    Serial.println(buff);
}


/**
 * UTIL FUNCTIONS
 */
static uint16_t read_address() {
    uint16_t res = 0x0000;

    for (int i = 0; i < ADDRESS_LEN; i++) {
        res |= digitalRead(address_pins[i]) << i;
    }

    return res;
}


static byte read_data() {
    byte pin;
    byte res = 0x00;

    for (int i = 0; i < DATA_LEN; i++) {
        pin = data_pins[i];
        pinMode(pin, INPUT);
        res |= digitalRead(pin) << i;
    }

    return res;
}


static void write_data(byte data) {
    byte pin;

    for (int i = 0, offset = (DATA_LEN - 1); i < DATA_LEN; i++, offset--) {
        pin = data_pins[i];
        digitalWrite(pin, data & (0x01 << i));
        pinMode(pin, OUTPUT);
    }
}


static mem_access_t read_rw() {
    return digitalRead(RW_PIN) ? READ : WRITE;
}


// Return the remapped RAM address
inline static uint16_t ram_addr(uint16_t abs_adr) {
    return abs_adr - RAM_START_ADDRESS;
}


// Return the remapped ROM address
inline static uint16_t rom_addr(uint16_t abs_adr) {
    return abs_adr - ROM_START_ADDRESS;
}


inline static void set_all_ram(byte data) {
    for (int i = 0; i < (sizeof(RAM) / sizeof(RAM[0])); i++) {
        RAM[i] = data;
    }
}


inline static void set_all_rom(byte data) {
    for (int i = 0; i < (sizeof(ROM) / sizeof(ROM[0])); i++) {
        ROM[i] = data;
    }
}