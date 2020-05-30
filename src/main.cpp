/**
 * FT81x on ST7701S Arduino Driver
 * Copyright (C) 2020  Raphael Stäbler
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 **/

#include <Arduino.h>
#include <FT81x.h>
#include <SPI.h>
#include <candy.h>

void waitForKeyPress();
void dumpChipID();

static unsigned int x = 0;

FT81x ft81x;

void setup() {
    Serial.begin(9600);

    SPI.begin();

    waitForKeyPress();

    Serial.println("Enable display");
    ft81x.begin();

    dumpChipID();

    delay(100);

    Serial.printf("REG_ID %x\n", ft81x.read8(FT81x_REG_ID));

    Serial.printf("REG_HCYCLE %i\n", ft81x.read16(FT81x_REG_HCYCLE));
    Serial.printf("REG_HSIZE %i\n", ft81x.read16(FT81x_REG_HSIZE));

    Serial.printf("REG_VCYCLE %i\n", ft81x.read16(FT81x_REG_VCYCLE));
    Serial.printf("REG_VSIZE %i\n", ft81x.read16(FT81x_REG_VSIZE));

    // waitForKeyPress();
}

void loop() {
    ft81x.writeGRAM(0, 8192, candy_map);

    ft81x.beginDisplayList();
    ft81x.clear(FT81x_COLOR_RGB(0, 0, 0));
    ft81x.drawText(160, (x + 28) % 480, 31, FT81x_COLOR_RGB(255, 255, 255), 0, "Hello World\0");
    ft81x.drawText((x + 28) % 480, 200, 31, FT81x_COLOR_RGB(255, 255, 255), 0, "FT81x\0");
    ft81x.drawCircle(x, 223, 20, FT81x_COLOR_RGB(255, 0, 0));
    ft81x.drawBitmap(0, 16, 16, 64, 64, 2);
    ft81x.drawSpinner(240, 240, 0, 0);
    ft81x.swapScreen();

    x = (x + 1) % 480;

    // waitForKeyPress();
}

void waitForKeyPress() {
    Serial.println("\nPress a key to continue\n");
    while (!Serial.available()) {
        __asm__ volatile("nop");
    }
    while (Serial.available()) {
        Serial.read();
    }
}

void dumpChipID() {
    digitalWrite(SS, LOW);
    SPI.beginTransaction(FT81x_SPI_SETTINGS);
    SPI.transfer(0x0C);
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    SPI.transfer(0x00);  // dummy byte
    Serial.println("Chip ID:");
    Serial.printf("0x0C0000: %x (supposed to be 0x8)\n", SPI.transfer(0x00));
    Serial.printf("0x0C0001: %x (supposed to be 0x12)\n", SPI.transfer(0x00));
    Serial.printf("0x0C0002: %x (supposed to be 0x1)\n", SPI.transfer(0x00));
    Serial.printf("0x0C0003: %x (supposed to be 0x0)\n", SPI.transfer(0x00));
    Serial.println("");
    SPI.endTransaction();
    digitalWrite(SS, HIGH);
}
