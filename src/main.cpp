#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

// U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0); // Try this SH1106 driver

#define SDA_PIN 8
#define SCL_PIN 9

#define I2C_ADDRESS 0x3C // AHT10 I2C address

#define LCD_BUFFER_SIZE 1024
uint8_t lcdBuffer[LCD_BUFFER_SIZE];
volatile uint16_t lcdBufferIndex = 0;
volatile bool i2cCommandReceived = false;

// i2c functions
void receiveEvent(int howMany);
void processLcdBuffer();
void outputI2CCommands();
void requestEvent();
void processCommand(uint8_t command);

#define NEOPIXEL_PIN 13

Adafruit_NeoPixel RGB = Adafruit_NeoPixel(3, NEOPIXEL_PIN, NEO_RGB + NEO_KHZ800);

#define LCD_SCK 4  // SCK
#define LCD_MISO 5 // (not needed for most LCDs)
#define LCD_MOSI 6 // MOSI
#define LCD_CS 7   // SS (CS)
#define LCD_A0 1   // or any free GPIO for DC
#define LCD_RST 2  // or any free GPIO for reset

U8G2_UC1701_MINI12864_F_4W_HW_SPI u8g2(U8G2_R0, LCD_CS, LCD_A0, LCD_RST);
// U8G2_ST7567_ENH_DG128064I_F_4W_SW_SPI u8g2(U8G2_R2, LCD_SCK, LCD_MOSI, LCD_CS, LCD_A0, LCD_RST);
// U8G2_ST7567_128X64_F_4W_SW_SPI u8g2(U8G2_R0, SCK, MOSI, CS, DC, RESET);
// U8G2_UC1701_MINI12864_F_4W_SW_SPI u8g2(U8G2_R0, LCD_SCK, LCD_MOSI, LCD_CS, LCD_A0, LCD_RST);
// bool firstCommand = true; // Track if this is the first command


volatile bool displayOff = false;
volatile bool displayOn = false;
volatile bool displayEnabled = true;

void setup()
{
    Serial.begin(115200); // Serial for debugging
    SPI.begin(LCD_SCK, LCD_MISO, LCD_MOSI);
    u8g2.begin();
    u8g2.setContrast(255);
    u8g2.clearBuffer();

    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 15, "SSD1306 OLED");
    u8g2.drawStr(0, 30, "Waiting for I2C");
    u8g2.sendBuffer();
    RGB.begin();
    RGB.setPixelColor(0, RGB.Color(255, 0, 255));
    RGB.setPixelColor(2, RGB.Color(255, 0, 255));
    RGB.setPixelColor(1, RGB.Color(255, 0, 255));
    RGB.show();
    // Wire.setClock(400000); // 400kHz instead of default 100kHz
    Wire.begin(I2C_ADDRESS, SDA_PIN, SCL_PIN, 3400000);
    Wire.onReceive(receiveEvent);
    // Wire.onRequest(requestEvent);
    Serial.println("I2C OLED Display Initialized");
}

void loop()
{
    if (displayOff)
    {
        Serial.println("Turning off LEDs");
        RGB.clear();
        RGB.show();
        displayOff = false;
        displayEnabled = false; // Disable display rendering
    }
    
    if (displayOn)
    {
        Serial.println("Turning on LEDs");
        RGB.setPixelColor(0, RGB.Color(255, 0, 255));
        RGB.setPixelColor(1, RGB.Color(255, 0, 255));
        RGB.setPixelColor(2, RGB.Color(255, 0, 255));
        RGB.show();
        displayOn = false;
        displayEnabled = true; // Enable display rendering
    }

    if (i2cCommandReceived)
    {
        // outputI2CCommands();
        processLcdBuffer();
        i2cCommandReceived = false; // Reset flag for next loop
    }
}

void requestEvent()
{
    // Default response is 0xFF for read commands
    // This matches typical OLED display behavior
    Wire.write(0xFF);
}

// Modify receiveEvent to track read commands:
// Suggested improvement for receiveEvent:
void receiveEvent(int howMany)
{
    noInterrupts();

    if (lcdBufferIndex + howMany >= LCD_BUFFER_SIZE)
    {
        lcdBufferIndex = 0;
    }

    while (Wire.available() && lcdBufferIndex < LCD_BUFFER_SIZE)
    {
        uint8_t data = Wire.read();
        if (data == 0xAE)
        {
            if (displayEnabled == true)
            {
            displayOff = true;  // Set flag instead of direct control
            }
        }
        else if (data == 0xAF)
        {
            if (displayEnabled == false)
            {
            displayOn = true;   // Set flag instead of direct control
            }
        }
        else if (data == 0xA6 || data == 0xA7) // Normal or Inverted display
        {
            // Handle display mode changes if needed
        }
        lcdBuffer[lcdBufferIndex++] = data;
    }

    i2cCommandReceived = true;
    interrupts();
}
struct DisplayState
{
    uint8_t frame[1024];
    uint16_t index;
    uint8_t column;
    uint8_t page;
    bool inverted;
    uint8_t last_cmd;
    uint32_t frame_count;
    uint8_t start_column;
    uint8_t start_page;
    uint8_t last_data[1024]; // Store last complete frame
    bool has_last_frame;
    bool frame_started;           // Track if we're mid-frame
    bool reset_pending;           // Track if position reset is pending
    uint8_t display_offset;       // Track vertical display offset
    uint8_t display_buffer[1024]; // Add a persistent display buffer
    uint8_t clock_div;         // Clock divide ratio
    uint8_t osc_freq;         // Oscillator frequency
    uint8_t multiplex_ratio;  // Display multiplex ratio
    bool charge_pump_enabled; // Charge pump state
    uint32_t last_frame_time; // For frame timing
};

bool frameReady = false; // Flag to indicate if a frame is ready to be rendered


void processCommands(DisplayState &state, uint16_t &i)
{
    while (i < lcdBufferIndex)
    {
        // Check if we've hit a data control byte (0x40) - stop processing commands
        if (lcdBuffer[i] == 0x40) {
            break;
        }
        
        // Check if we've hit another command control byte (0x00) - continue with next command sequence
        if (lcdBuffer[i] == 0x00) {
            i++; // Skip the control byte
            continue;
        }
        
        uint8_t cmd = lcdBuffer[i++];
        state.last_cmd = cmd;
        // Serial.printf("Processing command: 0x%02X\n", cmd); // Debug output
        
        switch (cmd)
        {
        case 0x21: // Set Column Address
            if (i + 1 < lcdBufferIndex)
            {
                state.start_column = lcdBuffer[i++];
                i++; // Skip end
                state.column = state.start_column;
                // Serial.printf("Set column range, start: %d\n", state.start_column);
            }
            break;

        case 0x22: // Set Page Address
            if (i + 1 < lcdBufferIndex)
            {
                state.start_page = lcdBuffer[i++];
                i++; // Skip end
                state.page = state.start_page;
                // Serial.printf("Set page range, start: %d\n", state.start_page);
            }
            break;

        case 0x7F: // Reset display position
            state.column = state.start_column;
            state.page = state.start_page;
            // Serial.println("Reset display position");
            break;
        case 0xAE: // Display OFF
            // Serial.println("Display OFF command received");
            displayOff = true;  // Set flag instead of direct control
            break;
        case 0xAF: // Display ON
            // Serial.println("Display ON command received");
            displayOn = true;   // Set flag instead of direct control
            frameReady = true;  // Indicate a frame is ready to be rendered
            break;
                        
        case 0xA6: // Normal display mode
            state.inverted = false;
            // Serial.println("Normal display mode");
            break;

        case 0xA7: // Inverted display mode
            state.inverted = true;
            // Serial.println("Inverted display mode");
            break;

        case 0xD5: // Set Clock Divide Ratio/Oscillator Frequency
            if (i < lcdBufferIndex)
            {
                uint8_t value = lcdBuffer[i++];
                state.clock_div = value & 0x0F; // Lower nibble
                state.osc_freq = value >> 4;    // Upper nibble
                Serial.printf("Set clock div/osc freq: 0x%02X\n", value);
            }
            break;

        case 0xA8: // Set Multiplex Ratio
            if (i < lcdBufferIndex)
            {
                state.multiplex_ratio = lcdBuffer[i++] & 0x3F; // 0-63
                Serial.printf("Set multiplex ratio: %d\n", state.multiplex_ratio);
            }
            break;

        case 0x8D: // Charge Pump Setting
            if (i < lcdBufferIndex)
            {
                uint8_t pump_setting = lcdBuffer[i++];
                state.charge_pump_enabled = (pump_setting == 0x14);
                Serial.printf("Charge pump setting: 0x%02X (enabled: %s)\n", 
                              pump_setting, state.charge_pump_enabled ? "yes" : "no");
            }
            break;

        case 0x81: // Set Contrast
            if (i < lcdBufferIndex)
            {
                uint8_t contrast = lcdBuffer[i++];
                Serial.printf("Set contrast: %d\n", contrast);
            }
            break;
            
        case 0xD3: // Set Display Offset
            if (i < lcdBufferIndex)
            {
                state.display_offset = lcdBuffer[i++] & 0x3F;
                Serial.printf("Set display offset: %d\n", state.display_offset);
            }
            break;

        default:
            // Handle other single-byte commands
            if ((cmd >= 0x10 && cmd <= 0x1F) || (cmd >= 0x00 && cmd <= 0x0F)) {
                // Column address commands - ignore for now
                Serial.printf("Column address command: 0x%02X\n", cmd);
            } else if (cmd >= 0xB0 && cmd <= 0xB7) {
                // Page address commands
                state.page = cmd & 0x07;
                Serial.printf("Set page address: %d\n", state.page);
            } else {
                Serial.printf("Unknown/unhandled command: 0x%02X\n", cmd);
            }
            break;
        }
    }
}

void processCommand(uint8_t command)
{
    switch (command)
    {
    case 0xAE: // Display OFF
        Serial.println("Display OFF command received");
        displayOff = true;  // Set flag instead of direct control
        break;
    case 0xAF: // Display ON
        Serial.println("Display ON command received");
        displayOn = true;   // Set flag instead of direct control
        frameReady = true;  // Indicate a frame is ready to be rendered
        break;
    case 0xA6: // Normal display mode
        Serial.println("Normal display mode");
        break;
    case 0xA7: // Inverted display mode
        Serial.println("Inverted display mode");
        break;
    case 0xD5: // Set Clock Divide Ratio/Oscillator Frequency
        Serial.println("Set Clock Divide Ratio/Oscillator Frequency command received");
        // Handle clock divide ratio and oscillator frequency settings
        break;
    case 0xA8: // Set Multiplex Ratio
        Serial.println("Set Multiplex Ratio command received");
        // Handle multiplex ratio settings
        break;
    case 0x8D: // Charge Pump Setting
        Serial.println("Charge Pump Setting command received");
        // Handle charge pump settings
        break;
    case 0x81: // Set Contrast
        Serial.println("Set Contrast command received");
        // Handle contrast settings
        break;
    case 0xD3: // Set Display Offset
        Serial.println("Set Display Offset command received");
        // Handle display offset settings
        break;
    default:
        Serial.printf("Unknown command: 0x%02X\n", command);
        // Handle unknown commands if necessary
        break;
    }
}

void processDataBlock(DisplayState &state, uint16_t &i)
{
    while (i < lcdBufferIndex && state.index < 1024)
    {
        // Always use start positions for initial placement
        uint16_t pos = (state.page + state.start_page) * 128 +
                       (state.column + state.start_column);

        if (pos < 1024)
        {
            state.frame[pos] = lcdBuffer[i++];
            state.index++;
            state.column++;

            if (state.column >= 128 - state.start_column)
            {
                state.column = 0;
                state.page++;

                if (state.page >= 8 - state.start_page)
                {
                    state.page = 0;
                }
            }
        }
    }
}
void renderFrame(DisplayState &state)
{
    uint8_t *u8g2buf = u8g2.getBufferPtr();

    // Copy our persistent buffer to U8G2's buffer
    memcpy(u8g2buf, state.display_buffer, 1024);

    // Update our display buffer with new frame data
    for (int page = 0; page < 8; page++)
    {
        for (int col = 0; col < 128; col++)
        {
            uint8_t data = state.frame[page * 128 + col];
            if (state.inverted)
                data = ~data;

            state.display_buffer[page * 128 + col] = data;
            u8g2buf[page * 128 + col] = data;
        }
    }

    u8g2.sendBuffer();
}

// function to just output all the i2c commands to serial
void outputI2CCommands()
{
    for (uint16_t i = 0; i < lcdBufferIndex; i++)
    {
        Serial.printf("0x%02X ", lcdBuffer[i]);
    }
    Serial.println();
}


TaskHandle_t renderTask = nullptr;

void renderTaskFunction(void *parameter)
{
    DisplayState* state = (DisplayState*)parameter;
    renderFrame(*state);
    delete state; // Clean up the allocated state
    vTaskDelete(NULL);
    renderTask = nullptr;
}

void processLcdBuffer()
{
    static DisplayState state = {0};

    uint16_t i = 0;
    while (i < lcdBufferIndex)
    {
        uint8_t control = lcdBuffer[i++];

        if (control == 0x40)
        {
            processDataBlock(state, i);
        }
        else if (control == 0x00)
        {
            processCommands(state, i);
        }
    }

    if (state.index == 1024 || frameReady)
    {
        DisplayState* taskState = new DisplayState(state);

        xTaskCreatePinnedToCore(
            renderTaskFunction,
            "RenderTask",
            65536,
            (void*)taskState,  // Pass the state pointer
            100,
            &renderTask,
            !ARDUINO_RUNNING_CORE // Pin to core 1
        );

        state.index = 0;
        frameReady = false;
    }

    lcdBufferIndex = 0;
}