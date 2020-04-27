#include "FT81x.h"

#define DISPLAY_WIDTH 480
#define DISPLAY_HEIGHT 480

#define READ  0x000000
#define WRITE 0x800000

#define DLSTART()                    0xFFFFFF00
#define SWAP()                       0xFFFFFF01
#define CLEAR(c, s, t)               ((0x26 << 24) | ((c) << 2) | ((s) << 1) | (t))
#define BEGIN(p)                     ((0x1F << 24) | (p))
#define END()                        (0x21 << 24)
#define END_DL()                     0x00
#define CLEAR_COLOR_RGB(r, g, b)     ((0x02 << 24) | ((r) << 16) | ((g) << 8) | (b))
#define CLEAR_COLOR(rgb)             ((0x02 << 24) | ((rgb) & 0xFFFFFF))
#define COLOR_RGB(r, g, b)           ((0x04 << 24) | ((r) << 16) | ((g) << 8) | (b))
#define COLOR(rgb)                   ((0x04 << 24) | ((rgb) & 0xFFFFFF))
#define POINT_SIZE(s)                ((0x0D << 24) | ((s) & 0xFFF))
#define LINE_WIDTH(w)                ((0x0E << 24) | ((w) & 0xFFF))
#define VERTEX2II(x, y, h, c)        ((1 << 31) | (((x) & 0xFF) << 21) | (((y) & 0xFF) << 12) | ((h) << 7) | (c))
#define VERTEX2F(x, y)               ((1 << 30) | (((x) & 0xFFFF) << 15) | ((y) & 0xFFFF))

#define POINTS 2
#define RECTS  9

static uint32_t dli = 0;

void FT81x::init() {
    pinMode(FT81x_CS1, OUTPUT); 
    digitalWrite(FT81x_CS1, HIGH);

    pinMode(FT81x_CS2, OUTPUT); 
    digitalWrite(FT81x_CS2, HIGH);

    pinMode(FT81x_DC, OUTPUT); 
    digitalWrite(FT81x_DC, LOW);

    FT81x::initFT81x();
    FT81x::initDisplay();
}

void FT81x::initFT81x()
{
    // reset
    FT81x::sendCommand(FT81x_CMD_RST_PULSE);
    delay(300);

    // activate
    FT81x::sendCommand(FT81x_CMD_ACTIVE);
    
    // wait for boot-up to complete
    delay(100);
    while (FT81x::read8(FT81x_REG_ID) != 0x7C) {
        __asm__("nop");
    }
    while (FT81x::read8(FT81x_REG_CPURESET) != 0x00) {
        __asm__("nop");
    }

    // pindrive
    /*FT81x::sendCommand(FT81x_CMD_PINDRIVE | 0x01 | (0x09 << 2));
    FT81x::sendCommand(FT81x_CMD_PINDRIVE | 0x01 | (0x0A << 2));
    FT81x::sendCommand(FT81x_CMD_PINDRIVE | 0x01 | (0x0B << 2));
    FT81x::sendCommand(FT81x_CMD_PINDRIVE | 0x01 | (0x0D << 2));
    delay(300);*/

    // configure rgb interface
    FT81x::write16(FT81x_REG_HCYCLE, DISPLAY_WIDTH + 68);
    FT81x::write16(FT81x_REG_HOFFSET, 42);
    FT81x::write16(FT81x_REG_HSYNC0, 4);
    FT81x::write16(FT81x_REG_HSYNC1, 8);
    FT81x::write16(FT81x_REG_HSIZE, DISPLAY_WIDTH);

    FT81x::write16(FT81x_REG_VCYCLE, DISPLAY_HEIGHT + 20);
    FT81x::write16(FT81x_REG_VOFFSET, 12);
    FT81x::write16(FT81x_REG_VSYNC0, 4);
    FT81x::write16(FT81x_REG_VSYNC1, 8);
    FT81x::write16(FT81x_REG_VSIZE, DISPLAY_HEIGHT);

    FT81x::write8(FT81x_REG_SWIZZLE, 0);
    FT81x::write8(FT81x_REG_PCLK_POL, 0);
    FT81x::write8(FT81x_REG_CSPREAD, 1);
    FT81x::write8(FT81x_REG_DITHER, 0);
    FT81x::write8(FT81x_REG_ROTATE, 0);

    // write first display list
    FT81x::clear(0x00FFF8);
    FT81x::swap();

    // enable pixel clock
    FT81x::write8(FT81x_REG_PCLK, 10);

    // reset display (somehow this generates a low pulse)
    FT81x::write16(FT81x_REG_GPIOX, 0x8000 | FT81x::read16(FT81x_REG_GPIOX));
    delay(300);
}

void FT81x::initDisplay()
{
    // display on
    sendCommandToDisplay(ST7701_DISPON);
    delay(100);
    sendCommandToDisplay(ST7701_SLPOUT);
    delay(100);

    // set pixel format
    sendCommandWithParamToDisplay(ST7701_COLMOD, 0x70);

    // normal mode on
    sendCommandToDisplay(ST7701_NORON);

    //sendCommandToDisplay(0x23); // all pixels on

    /*Serial.printf("RDID1: %x\n", queryDisplay(ST7701_RDID1));
    Serial.printf("RDID2: %x\n", queryDisplay(ST7701_RDID2));
    Serial.printf("RDID3: %x\n", queryDisplay(ST7701_RDID3));
    Serial.printf("RDDSDR: %x\n", queryDisplay(ST7701_RDDSDR));
    Serial.printf("RDDPM: %x (must be > 8)\n", queryDisplay(ST7701_RDDPM));*/
}

void FT81x::clear(uint32_t color)
{
    cmd(DLSTART());
    cmd(CLEAR_COLOR(color));
    cmd(CLEAR(1, 1, 1));
    cmd(END_DL());
}

void FT81x::drawCircle(int16_t x, int16_t y, uint8_t size, uint32_t color)
{
    cmd(DLSTART());
    cmd(COLOR(color));
    cmd(POINT_SIZE(size * 16));
    cmd(BEGIN(POINTS));
    cmd(VERTEX2F(x * 16, y * 16));
    cmd(END());
    cmd(END_DL());
}

void FT81x::drawRect(int16_t x, int16_t y, uint16_t width, uint16_t height, uint8_t cornerRadius, uint32_t color)
{
    cmd(DLSTART());
    cmd(COLOR(color));
    cmd(LINE_WIDTH(cornerRadius * 16));
    cmd(BEGIN(RECTS));
    cmd(VERTEX2II(x, y, 0, 0));
    cmd(VERTEX2II((x + width), (y + height), 0, 0));
    cmd(END());
    cmd(END_DL());
}

void FT81x::dl(uint32_t cmd)
{
    uint32_t addr = FT81x_RAM_DL + dli;
    //Serial.printf("write32 %x, %x\n", addr, cmd);
    write32(addr, cmd);
    dli += 4;
}

void FT81x::cmd(uint32_t cmd)
{
    uint16_t cmdWrite = FT81x::read16(FT81x_REG_CMD_WRITE);
    uint32_t addr = FT81x_RAM_CMD + cmdWrite;
    //Serial.printf("write32 %x, %x\n", addr, cmd);
    write32(addr, cmd);
    write16(FT81x_REG_CMD_WRITE, (cmdWrite + 4) % 4096);
}

void FT81x::swap()
{
    cmd(SWAP());
}

void FT81x::sendCommand(uint32_t cmd)
{
    digitalWrite(FT81x_CS1, LOW);
    SPI.beginTransaction(FT81x_SPI_SETTINGS);
    SPI.transfer((cmd >> 16) & 0xFF);
    SPI.transfer((cmd >> 8) & 0xFF);
    SPI.transfer(cmd & 0xFF);
    SPI.endTransaction();
    digitalWrite(FT81x_CS1, HIGH);
}

uint8_t FT81x::read8(uint32_t address)
{
    uint32_t cmd = address;
    digitalWrite(FT81x_CS1, LOW);
    SPI.beginTransaction(FT81x_SPI_SETTINGS);
    SPI.transfer((cmd >> 16) & 0xFF);
    SPI.transfer((cmd >> 8) & 0xFF);
    SPI.transfer(cmd & 0xFF);
    SPI.transfer(0x00); // dummy byte
    uint8_t result = SPI.transfer(0x00);
    SPI.endTransaction();
    digitalWrite(FT81x_CS1, HIGH);
    return result;
}

uint16_t FT81x::read16(uint32_t address)
{
    uint32_t cmd = address | READ;
    digitalWrite(FT81x_CS1, LOW);
    SPI.beginTransaction(FT81x_SPI_SETTINGS);
    SPI.transfer((cmd >> 16) & 0xFF);
    SPI.transfer((cmd >> 8) & 0xFF);
    SPI.transfer(cmd & 0xFF);
    SPI.transfer(0x00); // dummy byte
    uint16_t result = SPI.transfer(0x00);
    result |= (SPI.transfer(0x00) << 8);
    SPI.endTransaction();
    digitalWrite(FT81x_CS1, HIGH);
    return result;
}

uint32_t FT81x::read32(uint32_t address)
{
    uint32_t cmd = address | READ;
    digitalWrite(FT81x_CS1, LOW);
    SPI.beginTransaction(FT81x_SPI_SETTINGS);
    SPI.transfer((cmd >> 16) & 0xFF);
    SPI.transfer((cmd >> 8) & 0xFF);
    SPI.transfer(cmd & 0xFF);
    SPI.transfer(0x00); // dummy byte
    uint16_t result = SPI.transfer(0x00);
    result |= (SPI.transfer(0x00) << 8);
    result |= (SPI.transfer(0x00) << 16);
    result |= (SPI.transfer(0x00) << 24);
    SPI.endTransaction();
    digitalWrite(FT81x_CS1, HIGH);
    return result;
}

void FT81x::write8(uint32_t address, uint8_t data)
{
    uint32_t cmd = address | WRITE;
    digitalWrite(FT81x_CS1, LOW);
    SPI.beginTransaction(FT81x_SPI_SETTINGS);
    SPI.transfer((cmd >> 16) & 0xFF);
    SPI.transfer((cmd >> 8) & 0xFF);
    SPI.transfer(cmd & 0xFF);
    SPI.transfer(data);
    SPI.endTransaction();
    digitalWrite(FT81x_CS1, HIGH);
}

void FT81x::write16(uint32_t address, uint16_t data)
{
    uint32_t cmd = address | WRITE;
    digitalWrite(FT81x_CS1, LOW);
    SPI.beginTransaction(FT81x_SPI_SETTINGS);
    SPI.transfer((cmd >> 16) & 0xFF);
    SPI.transfer((cmd >> 8) & 0xFF);
    SPI.transfer(cmd & 0xFF);
    SPI.transfer(data & 0xFF);
    SPI.transfer((data >> 8) & 0xFF);
    SPI.endTransaction();
    digitalWrite(FT81x_CS1, HIGH);
}

void FT81x::write32(uint32_t address, uint32_t data)
{
    uint32_t cmd = address | WRITE;
    digitalWrite(FT81x_CS1, LOW);
    SPI.beginTransaction(FT81x_SPI_SETTINGS);
    SPI.transfer((cmd >> 16) & 0xFF);
    SPI.transfer((cmd >> 8) & 0xFF);
    SPI.transfer(cmd & 0xFF);
    SPI.transfer(data & 0xFF);
    SPI.transfer((data >> 8) & 0xFF);
    SPI.transfer((data >> 16) & 0xFF);
    SPI.transfer((data >> 24) & 0xFF);
    SPI.endTransaction();
    digitalWrite(FT81x_CS1, HIGH);
}

void FT81x::sendCommandToDisplay(uint8_t cmd)
{
    digitalWrite(FT81x_DC, LOW);
    digitalWrite(FT81x_CS2, LOW);
    SPI.beginTransaction(FT81x_SPI_SETTINGS);
    SPI.transfer(cmd);
    SPI.endTransaction();
    digitalWrite(FT81x_CS2, HIGH);
}

void FT81x::sendCommandWithParamToDisplay(uint8_t cmd, uint8_t param)
{
    digitalWrite(FT81x_DC, LOW);
    digitalWrite(FT81x_CS2, LOW);
    SPI.beginTransaction(FT81x_SPI_SETTINGS);
    SPI.transfer(cmd);
    digitalWrite(FT81x_DC, HIGH);
    SPI.transfer(param);
    SPI.endTransaction();
    digitalWrite(FT81x_CS2, HIGH);
    digitalWrite(FT81x_DC, LOW);
}

uint8_t FT81x::queryDisplay(uint8_t cmd)
{
    digitalWrite(FT81x_DC, LOW);
    digitalWrite(FT81x_CS2, LOW);
    SPI.beginTransaction(FT81x_SPI_SETTINGS);
    uint8_t result = SPI.transfer16(cmd << 8) & 0xFF;
    SPI.endTransaction();
    digitalWrite(FT81x_CS2, HIGH);
    return result;
}
