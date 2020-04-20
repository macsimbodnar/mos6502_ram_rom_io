// #define LED                     LED_BUILTIN  // PIN 13
#define CLOCK_TICK_INPUT        18
#define RW_PIN                  13

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
 *      8000 - FFFF     ROM         (FFFC - FFFD    RESET VECTOR)
 */
#define TOT_MEM             0xFFFF
#define RAM_SIZE            0x1000
#define RAM_START_ADDRESS   0x0000

static byte RAM[RAM_SIZE];
// static byte IO[0x600F - 0x6000];

// #define ROM_ENABLED          // Uncomment this if want to handle also the ROM
#ifdef ROM_ENABLED
#define ROM_SIZE            0x0800
#define ROM_START_ADDRESS   (TOT_MEM - ROM_SIZE + 0x0001)
static byte ROM[ROM_SIZE];
#endif


typedef struct {
    const byte pins[DATA_LEN];
    byte data_direction[DATA_LEN];
} io_port_t;


static io_port_t io_port_A = {
//   LSB                  MSB
    {4, 5, 6, 7, 8, 9, 10, 11},
    {INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT}
};


static io_port_t io_port_B = {
//   LSB                        MSB
    {39, 41, 43, 45, 47, 49, 51, 53},
    {INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT}
};


void setup() {

    // MOS6502 CLOCK INTERRUPT
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
        pinMode(data_pins[i], OUTPUT);
    }

    // INIT SERIAL
    Serial.begin(115200);

    while (!Serial) ;     // wait for serial port to connect. Needed for native USB port only

    // Init memory
    set_all_ram(0x00);

#ifdef ROM_ENABLED
    set_all_rom(0xEA);

    // Setup the reset vector to address 0xFF00 (will be the entry point to the program)
    ROM[rom_addr(0xFFFC)] = 0x00;
    ROM[rom_addr(0xFFFD)] = 0xFF;

    // Load a simple program into memory
    // LDA #$01     ; Load 1 into register A
    // STA $0200    ; Save the value of A into mem address 0x0200 (It is in RAM memory section)
    // LDA #$05     ; Load 5 into register A
    // STA $0201    ; Save the value of A into mem address 0x0201 (It is in RAM memory section)
    // LDA #$08     ; Load 8 into register A
    // STA $0202    ; Save the value of A into mem address 0x0202 (It is in RAM memory section)
    byte prog[] = { 0xA9, 0x01, 0x8D, 0x00, 0x02, 0xA9, 0x05, 0x8D, 0x01, 0x02, 0xA9, 0x08, 0x8D, 0x02, 0x02 };

    for (int i = 0; i < sizeof(prog); i++) {
        ROM[rom_addr(0xFF00 + i)] = prog[i];
    }

#endif
}


void loop() {
}


/**
   INTERRUPT HANDLERS
*/
void clock_tick_ISR() {
    noInterrupts();

    uint16_t address = read_address();
    mem_access_t rw = read_rw();
    byte data;

    const char *mem;

    if (address > 0x7FFF) {
        // ROM
        mem = "ROM";

#ifdef ROM_ENABLED

        // The ROM is handled internally
        switch (rw) {
        case READ:
            data = ROM[rom_addr(address)];
            write_data(data);
            break;
        }

#else
        // The ROM is handled by external chip, the AT28C256
        // Just display data on the data bus
        data = read_data();
#endif

    } else if (address < 0x3000) {      // The ram is from address 0x0000 to the 0x3000
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
    } else if (address > 0x5FFF) {      // The I/O is from address 0x6000 to the address 0x7FFF

        // I/O and Other
        mem = "I/O";

        switch (rw) {
        case READ:
            // data = latch_and_write_data();
            break;

        case WRITE:
            switch (address) {
            case 0x6000:        // PORT B
                data = read_and_latch_io(&io_port_B);
                break;

            case 0x6001:        // PORT A
                data = read_and_latch_io(&io_port_A);

                break;

            case 0x6002:        // Data direction Port B
                data = set_data_direction(&io_port_B);
                break;

            case 0x6003:        // Data direction Port A
                data = set_data_direction(&io_port_A);
                break;
            }

            break;
        }


    } else {                            // Addresses from 0x3000 to the 0x5FFF are unused
        mem = "UDF";
    }

    // Print the status on serial
    static char buff[20];
    sprintf(buff, "%s   %04X   %c   %02X  |  %02X  %02X  %02X", mem, address, rw == READ ? 'R' : 'W',
            data, RAM[ram_addr(0x0200)], RAM[ram_addr(0x0201)], RAM[ram_addr(0x0202)]);

    Serial.println(buff);

    interrupts();
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


static byte read_and_latch_io(io_port_t *port) {
    byte pin;
    byte io_pin;
    byte res = 0x00;

    for (int i = 0; i < DATA_LEN; i++) {
        pin = data_pins[i];
        io_pin = port->pins[i];
        pinMode(pin, INPUT);
        byte val = digitalRead(pin);

        // Write on pin only if the mode is write to avoid weird behavior
        if (port->data_direction[i] == OUTPUT) {
            digitalWrite(io_pin, val);
            pinMode(io_pin, OUTPUT);
        }

        res |= val << i;
    }

    return res;
}


static byte set_data_direction(io_port_t *port) {
    byte pin;
    byte res = 0x00;

    for (int i = 0; i < DATA_LEN; i++) {
        pin = data_pins[i];
        pinMode(pin, INPUT);
        byte val = digitalRead(pin);

        port->data_direction[i] = val ? OUTPUT : INPUT;

        res |= val << i;
    }

    return res;
}


#ifdef false
static byte read_and_latch_data() {
    byte data_pin;
    byte latch_pin;
    byte res = 0x00;

    for (int i = 0; i < DATA_LEN; i++) {
        data_pin = data_pins[i];
        latch_pin = io_port_B_pins[i];

        pinMode(data_pin, INPUT);
        byte status = digitalRead(data_pin);
        digitalWrite(latch_pin, status);
        pinMode(latch_pin, OUTPUT);

        res |= status << i;
    }

    return res;
}

static byte latch_and_write_data() {
    byte data_pin;
    byte latch_pin;
    byte res = 0x00;

    for (int i = 0; i < DATA_LEN; i++) {
        data_pin = data_pins[i];
        latch_pin = io_port_B_pins[i];

        pinMode(latch_pin, INPUT);
        byte status = digitalRead(latch_pin);
        digitalWrite(data_pin, status);
        pinMode(data_pin, OUTPUT);

        res |= status << i;
    }

    return res;
}
#endif


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


inline static void set_all_ram(byte data) {
    for (int i = 0; i < (sizeof(RAM) / sizeof(RAM[0])); i++) {
        RAM[i] = data;
    }
}


#ifdef ROM_ENABLED
// Return the remapped ROM address
inline static uint16_t rom_addr(uint16_t abs_adr) {
    return abs_adr - ROM_START_ADDRESS;
}


inline static void set_all_rom(byte data) {
    for (int i = 0; i < (sizeof(ROM) / sizeof(ROM[0])); i++) {
        ROM[i] = data;
    }
}
#endif