//-----------------------------------------------------------------------------
// Copyright (C) Proxmark3 contributors. See AUTHORS.md for details.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// See LICENSE.txt for the text of the license.
//-----------------------------------------------------------------------------
#include "pcf7931.h"

#include "proxmark3_arm.h"
#include "cmd.h"
#include "BigBuf.h"
#include "fpgaloader.h"
#include "ticks.h"
#include "dbprint.h"
#include "util.h"
#include "lfsampling.h"
#include "string.h"

#define T0_PCF 8 //period for the pcf7931 in us
#define ALLOC 16

size_t DemodPCF7931_DISABLED(uint8_t **outBlocks, bool ledcontrol) {

    // 2021 iceman, memor
    uint8_t bits[256] = {0x00};
    uint8_t blocks[8][16];

    uint8_t *dest = BigBuf_get_addr();

    int g_GraphTraceLen = BigBuf_max_traceLen();
    if (g_GraphTraceLen > 18000) {
        g_GraphTraceLen = 18000;
    }

    int i = 2, j, lastval, bitidx, half_switch;
    int clock = 64;
    int tolerance = clock / 8;
    int pmc, block_done;
    int lc, warnings = 0;
    size_t num_blocks = 0;
    int lmin = 64, lmax = 192;
    uint8_t dir;

    BigBuf_Clear_keep_EM();
    LFSetupFPGAForADC(LF_DIVISOR_125, true);
    DoAcquisition_default(0, true, ledcontrol);

    /* Find first local max/min */
    if (dest[1] > dest[0]) {
        while (i < g_GraphTraceLen) {
            if (!(dest[i] > dest[i - 1]) && dest[i] > lmax) {
                break;
            }
            i++;
        }
        dir = 0;
    } else {
        while (i < g_GraphTraceLen) {
            if (!(dest[i] < dest[i - 1]) && dest[i] < lmin) {
                break;
            }
            i++;
        }
        dir = 1;
    }

    lastval = i++;
    half_switch = 0;
    pmc = 0;
    block_done = 0;

    for (bitidx = 0; i < g_GraphTraceLen; i++) {

        if ((dest[i - 1] > dest[i] && dir == 1 && dest[i] > lmax) || (dest[i - 1] < dest[i] && dir == 0 && dest[i] < lmin)) {
            lc = i - lastval;
            lastval = i;

            // Switch depending on lc length:
            // Tolerance is 1/8 of clock rate (arbitrary)
            if (ABS(lc - clock / 4) < tolerance) {
                // 16T0
                if ((i - pmc) == lc) { // 16T0 was previous one
                    // It's a PMC
                    i += (128 + 127 + 16 + 32 + 33 + 16) - 1;
                    lastval = i;
                    pmc = 0;
                    block_done = 1;
                } else {
                    pmc = i;
                }
            } else if (ABS(lc - clock / 2) < tolerance) {
                // 32TO
                if ((i - pmc) == lc) { // 16T0 was previous one
                    // It's a PMC !
                    i += (128 + 127 + 16 + 32 + 33) - 1;
                    lastval = i;
                    pmc = 0;
                    block_done = 1;
                } else if (half_switch == 1) {
                    bits[bitidx++] = 0;
                    half_switch = 0;
                } else
                    half_switch++;
            } else if (ABS(lc - clock) < tolerance) {
                // 64TO
                bits[bitidx++] = 1;
            } else {
                // Error
                if (++warnings > 10) {

                    if (g_dbglevel >= DBG_EXTENDED) {
                        Dbprintf("Error: too many detection errors, aborting");
                    }

                    return 0;
                }
            }

            if (block_done == 1) {
                if (bitidx == 128) {
                    for (j = 0; j < 16; ++j) {
                        blocks[num_blocks][j] =
                            128 * bits[j * 8 + 7] +
                            64 * bits[j * 8 + 6] +
                            32 * bits[j * 8 + 5] +
                            16 * bits[j * 8 + 4] +
                            8 * bits[j * 8 + 3] +
                            4 * bits[j * 8 + 2] +
                            2 * bits[j * 8 + 1] +
                            bits[j * 8]
                            ;
                    }
                    num_blocks++;
                }
                bitidx = 0;
                block_done = 0;
                half_switch = 0;
            }

            if (i < g_GraphTraceLen) {
                dir = (dest[i - 1] > dest[i]) ? 0 : 1;
            }
        }

        if (bitidx == 255) {
            bitidx = 0;
        }

        if (num_blocks == 4) {
            break;
        }
    }
    memcpy(outBlocks, blocks, 16 * num_blocks);
    return num_blocks;
}

bool IsBlock0PCF7931(uint8_t *block) {
    // assuming all RFU bits are set to 0
    // if PAC is enabled password is set to 0
    if (block[7] == 0x01) {
        if (!memcmp(block, "\x00\x00\x00\x00\x00\x00\x00", 7) &&
                !memcmp(block + 9, "\x00\x00\x00\x00\x00\x00\x00", 7)) {
            return true;
        }

    } else if (block[7] == 0x00) {
        if (!memcmp(block + 9, "\x00\x00\x00\x00\x00\x00\x00", 7)) {
            return true;
        }
    }
    return false;
}

bool IsBlock1PCF7931(const uint8_t *block) {
    // assuming all RFU bits are set to 0

    uint8_t rb1 = block[14] & 0x80;
    uint8_t rfb = block[14] & 0x7f;
    uint8_t rlb = block[15];

    if (block[10] == 0
            && block[11] == 0
            && block[12] == 0
            && block[13] == 0) {
        // block 1 is sent only if (RLB >= 1 && RFB <= 1) or RB1 enabled
        if (rfb <= rlb
                && rfb <= 9
                && rlb <= 9
                && ((rfb <= 1 && rlb >= 1) || rb1)) {
            return true;
        }
    }

    return false;
}

void ReadPCF7931_DISABLED(bool ledcontrol) {
    int found_blocks = 0; // successfully read blocks
    int max_blocks = 8;   // readable blocks
    uint8_t memory_blocks[8][17]; // PCF content
    uint8_t single_blocks[8][17]; // PFC blocks with unknown position
    int single_blocks_cnt = 0;

    size_t n; // transmitted blocks
    uint8_t tmp_blocks[4][16]; // temporary read buffer

    uint8_t found_0_1 = 0; // flag: blocks 0 and 1 were found
    int errors = 0; // error counter
    int tries = 0; // tries counter

    memset(memory_blocks, 0, 8 * 17 * sizeof(uint8_t));
    memset(single_blocks, 0, 8 * 17 * sizeof(uint8_t));

    int i = 0, j = 0;

    do {
        i = 0;

        memset(tmp_blocks, 0, 4 * 16 * sizeof(uint8_t));
        n = DemodPCF7931((uint8_t **)tmp_blocks, ledcontrol);
        if (!n)
            ++errors;

        // exit if no block is received
        if (errors >= 10 && found_blocks == 0 && single_blocks_cnt == 0) {

            if (g_dbglevel >= DBG_INFO)
                Dbprintf("[!!] Error, no tag or bad tag");

            return;
        }
        // exit if too many errors during reading
        if (tries > 50 && (2 * errors > tries)) {

            if (g_dbglevel >= DBG_INFO) {
                Dbprintf("[!!] Error reading the tag, only partial content");
            }

            goto end;
        }

        // our logic breaks if we don't get at least two blocks
        if (n < 2) {
            // skip if all 0s block or no blocks
            if (n == 0 || !memcmp(tmp_blocks[0], "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00", 16))
                continue;

            // add block to single blocks list
            if (single_blocks_cnt < max_blocks) {
                for (i = 0; i < single_blocks_cnt; ++i) {
                    if (!memcmp(single_blocks[i], tmp_blocks[0], 16)) {
                        j = 1;
                        break;
                    }
                }
                if (j != 1) {
                    memcpy(single_blocks[single_blocks_cnt], tmp_blocks[0], 16);
                    print_result("got single block", single_blocks[single_blocks_cnt], 16);
                    single_blocks_cnt++;
                }
                j = 0;
            }
            ++tries;
            continue;
        }

        if (g_dbglevel >= DBG_EXTENDED)
            Dbprintf("(dbg) got %d blocks (%d/%d found) (%d tries, %d errors)", n, found_blocks, (max_blocks == 0 ? found_blocks : max_blocks), tries, errors);

        for (i = 0; i < n; ++i) {
            print_result("got consecutive blocks", tmp_blocks[i], 16);
        }

        i = 0;
        if (!found_0_1) {
            while (i < n - 1) {
                if (IsBlock0PCF7931(tmp_blocks[i]) && IsBlock1PCF7931(tmp_blocks[i + 1])) {
                    found_0_1 = 1;
                    memcpy(memory_blocks[0], tmp_blocks[i], 16);
                    memcpy(memory_blocks[1], tmp_blocks[i + 1], 16);
                    memory_blocks[0][ALLOC] = memory_blocks[1][ALLOC] = 1;
                    // block 1 tells how many blocks are going to be sent
                    max_blocks = MAX((memory_blocks[1][14] & 0x7f), memory_blocks[1][15]) + 1;
                    found_blocks = 2;

                    Dbprintf("Found blocks 0 and 1. PCF is transmitting %d blocks.", max_blocks);

                    // handle the following blocks
                    for (j = i + 2; j < n; ++j) {
                        memcpy(memory_blocks[found_blocks], tmp_blocks[j], 16);
                        memory_blocks[found_blocks][ALLOC] = 1;
                        ++found_blocks;
                    }
                    break;
                }
                ++i;
            }
        } else {
            // Trying to re-order blocks
            // Look for identical block in memory blocks
            while (i < n - 1) {
                // skip all zeroes blocks
                if (memcmp(tmp_blocks[i], "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00", 16)) {
                    for (j = 1; j < max_blocks - 1; ++j) {
                        if (!memcmp(tmp_blocks[i], memory_blocks[j], 16) && !memory_blocks[j + 1][ALLOC]) {
                            memcpy(memory_blocks[j + 1], tmp_blocks[i + 1], 16);
                            memory_blocks[j + 1][ALLOC] = 1;
                            if (++found_blocks >= max_blocks) goto end;
                        }
                    }
                }
                if (memcmp(tmp_blocks[i + 1], "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00", 16)) {
                    for (j = 0; j < max_blocks; ++j) {
                        if (!memcmp(tmp_blocks[i + 1], memory_blocks[j], 16) && !memory_blocks[(j == 0 ? max_blocks : j) - 1][ALLOC]) {
                            if (j == 0) {
                                memcpy(memory_blocks[max_blocks - 1], tmp_blocks[i], 16);
                                memory_blocks[max_blocks - 1][ALLOC] = 1;
                            } else {
                                memcpy(memory_blocks[j - 1], tmp_blocks[i], 16);
                                memory_blocks[j - 1][ALLOC] = 1;
                            }
                            if (++found_blocks >= max_blocks) goto end;
                        }
                    }
                }
                ++i;
            }
        }
        ++tries;
        if (BUTTON_PRESS()) {
            if (g_dbglevel >= DBG_EXTENDED)
                Dbprintf("Button pressed, stopping.");

            goto end;
        }
    } while (found_blocks < max_blocks);

end:
    Dbprintf("-----------------------------------------");
    Dbprintf("Memory content:");
    Dbprintf("-----------------------------------------");
    for (i = 0; i < max_blocks; ++i) {
        if (memory_blocks[i][ALLOC])
            print_result("Block", memory_blocks[i], 16);
        else
            Dbprintf("<missing block %d>", i);
    }
    Dbprintf("-----------------------------------------");

    if (found_blocks < max_blocks) {
        Dbprintf("-----------------------------------------");
        Dbprintf("Blocks with unknown position:");
        Dbprintf("-----------------------------------------");
        for (i = 0; i < single_blocks_cnt; ++i)
            print_result("Block", single_blocks[i], 16);

        Dbprintf("-----------------------------------------");
    }
    reply_mix(CMD_ACK, 0, 0, 0, 0, 0);
}

static void RealWritePCF7931(uint8_t *pass, uint16_t init_delay, int32_t l, int32_t p, uint8_t address, uint8_t byte, uint8_t data, bool ledcontrol) {
    uint32_t tab[1024] = {0}; // data times frame
    uint32_t u = 0;
    uint8_t parity = 0;
    bool comp = 0;

    //BUILD OF THE DATA FRAME
    //alimentation of the tag (time for initializing)
    AddPatternPCF7931(init_delay, 0, 8192 / 2 * T0_PCF, tab);
    AddPatternPCF7931(8192 / 2 * T0_PCF + 319 * T0_PCF + 70, 3 * T0_PCF, 29 * T0_PCF, tab);
    //password indication bit
    AddBitPCF7931(1, tab, l, p);
    //password (on 56 bits)
    AddBytePCF7931(pass[0], tab, l, p);
    AddBytePCF7931(pass[1], tab, l, p);
    AddBytePCF7931(pass[2], tab, l, p);
    AddBytePCF7931(pass[3], tab, l, p);
    AddBytePCF7931(pass[4], tab, l, p);
    AddBytePCF7931(pass[5], tab, l, p);
    AddBytePCF7931(pass[6], tab, l, p);
    //programming mode (0 or 1)
    AddBitPCF7931(0, tab, l, p);

    //block address on 6 bits
    for (u = 0; u < 6; ++u) {
        if (address & (1 << u)) { // bit 1
            ++parity;
            AddBitPCF7931(1, tab, l, p);
        } else {              // bit 0
            AddBitPCF7931(0, tab, l, p);
        }
    }

    //byte address on 4 bits
    for (u = 0; u < 4; ++u) {
        if (byte & (1 << u)) { // bit 1
            parity++;
            AddBitPCF7931(1, tab, l, p);
        } else                // bit 0
            AddBitPCF7931(0, tab, l, p);
    }

    //data on 8 bits
    for (u = 0; u < 8; u++) {
        if (data & (1 << u)) { // bit 1
            parity++;
            AddBitPCF7931(1, tab, l, p);
        } else                //bit 0
            AddBitPCF7931(0, tab, l, p);
    }

    //parity bit
    if ((parity % 2) == 0)
        AddBitPCF7931(0, tab, l, p); //even parity
    else
        AddBitPCF7931(1, tab, l, p);//odd parity

    //time access memory
    AddPatternPCF7931(5120 + 2680, 0, 0, tab);

    //conversion of the scale time
    for (u = 0; u < 500; ++u)
        tab[u] = (tab[u] * 3) / 2;

    //compensation of the counter reload
    while (!comp) {
        comp = 1;
        for (u = 0; tab[u] != 0; ++u)
            if (tab[u] > 0xFFFF) {
                tab[u] -= 0xFFFF;
                comp = 0;
            }
    }

    SendCmdPCF7931(tab, ledcontrol);
}

/* Write on a byte of a PCF7931 tag
 * @param address : address of the block to write
   @param byte : address of the byte to write
    @param data : data to write
 */
void WritePCF7931(uint8_t pass1, uint8_t pass2, uint8_t pass3, uint8_t pass4, uint8_t pass5, uint8_t pass6, uint8_t pass7, uint16_t init_delay, int32_t l, int32_t p, uint8_t address, uint8_t byte, uint8_t data, bool ledcontrol) {

    if (g_dbglevel >= DBG_INFO) {
        Dbprintf("Initialization delay : %d us", init_delay);
        Dbprintf("Offsets : %d us on the low pulses width, %d us on the low pulses positions", l, p);
    }

    Dbprintf("Password (LSB first on each byte): %02x %02x %02x %02x %02x %02x %02x", pass1, pass2, pass3, pass4, pass5, pass6, pass7);
    Dbprintf("Block address : %02x", address);
    Dbprintf("Byte address : %02x", byte);
    Dbprintf("Data : %02x", data);

    uint8_t password[7] = {pass1, pass2, pass3, pass4, pass5, pass6, pass7};

    RealWritePCF7931(password, init_delay, l, p, address, byte, data, ledcontrol);
}


/* Send a trame to a PCF7931 tags
 * @param tab : array of the data frame
 */

void SendCmdPCF7931(const uint32_t *tab, bool ledcontrol) {
    uint16_t u = 0, tempo = 0;

    if (g_dbglevel >= DBG_INFO) {
        Dbprintf("Sending data frame...");
    }

    FpgaDownloadAndGo(FPGA_BITSTREAM_LF);
    FpgaSendCommand(FPGA_CMD_SET_DIVISOR, LF_DIVISOR_125); //125kHz
    FpgaWriteConfWord(FPGA_MAJOR_MODE_LF_PASSTHRU);

    if (ledcontrol) LED_A_ON();

    // steal this pin from the SSP and use it to control the modulation
    AT91C_BASE_PIOA->PIO_PER = GPIO_SSC_DOUT;
    AT91C_BASE_PIOA->PIO_OER = GPIO_SSC_DOUT;

    //initialization of the timer
    AT91C_BASE_PMC->PMC_PCER |= (0x1 << AT91C_ID_TC0);
    AT91C_BASE_TCB->TCB_BMR = AT91C_TCB_TC0XC0S_NONE | AT91C_TCB_TC1XC1S_TIOA0 | AT91C_TCB_TC2XC2S_NONE;
    AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKDIS;                 // timer disable
    AT91C_BASE_TC0->TC_CMR = AT91C_TC_CLKS_TIMER_DIV3_CLOCK;  // clock at 48/32 MHz
    AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKEN;

    // Assert a sync signal. This sets all timers to 0 on next active clock edge
    AT91C_BASE_TCB->TCB_BCR = 1;

    tempo = AT91C_BASE_TC0->TC_CV;
    for (u = 0; tab[u] != 0; u += 3) {
        // modulate antenna
        HIGH(GPIO_SSC_DOUT);
        while (tempo != tab[u]) {
            tempo = AT91C_BASE_TC0->TC_CV;
        }

        // stop modulating antenna
        LOW(GPIO_SSC_DOUT);
        while (tempo != tab[u + 1]) {
            tempo = AT91C_BASE_TC0->TC_CV;
        }

        // modulate antenna
        HIGH(GPIO_SSC_DOUT);
        while (tempo != tab[u + 2]) {
            tempo = AT91C_BASE_TC0->TC_CV;
        }
    }

    if (ledcontrol) LED_A_OFF();
    FpgaWriteConfWord(FPGA_MAJOR_MODE_OFF);
    SpinDelay(200);

    AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKDIS; // timer disable
}


/* Add a byte for building the data frame of PCF7931 tags
 * @param b : byte to add
 * @param tab : array of the data frame
 * @param l : offset on low pulse width
 * @param p : offset on low pulse positioning
 */
bool AddBytePCF7931(uint8_t byte, uint32_t *tab, int32_t l, int32_t p) {
    uint32_t u;
    for (u = 0; u < 8; ++u) {
        if (byte & (1 << u)) { //bit is 1
            if (AddBitPCF7931(1, tab, l, p) == 1) return true;
        } else { //bit is 0
            if (AddBitPCF7931(0, tab, l, p) == 1) return true;
        }
    }

    return false;
}

/* Add a bits for building the data frame of PCF7931 tags
 * @param b : bit to add
 * @param tab : array of the data frame
 * @param l : offset on low pulse width
 * @param p : offset on low pulse positioning
 */
bool AddBitPCF7931(bool b, uint32_t *tab, int32_t l, int32_t p) {
    uint8_t u = 0;

    //we put the cursor at the last value of the array
    for (u = 0; tab[u] != 0; u += 3) { };

    if (b == 1) {   //add a bit 1
        if (u == 0)
            tab[u] = 34 * T0_PCF + p;
        else
            tab[u] = 34 * T0_PCF + tab[u - 1] + p;

        tab[u + 1] =  6 * T0_PCF + tab[u] + l;
        tab[u + 2] = 88 * T0_PCF + tab[u + 1] - l - p;
        return false;
    } else { //add a bit 0

        if (u == 0)
            tab[u] = 98 * T0_PCF + p;
        else
            tab[u] = 98 * T0_PCF + tab[u - 1] + p;

        tab[u + 1] =  6 * T0_PCF + tab[u] + l;
        tab[u + 2] = 24 * T0_PCF + tab[u + 1] - l - p;
        return false;
    }
    return true;
}

/* Add a custom pattern in the data frame
 * @param a : delay of the first high pulse
 * @param b : delay of the low pulse
 * @param c : delay of the last high pulse
 * @param tab : array of the data frame
 */
bool AddPatternPCF7931(uint32_t a, uint32_t b, uint32_t c, uint32_t *tab) {
    uint32_t u = 0;
    for (u = 0; tab[u] != 0; u += 3) {} //we put the cursor at the last value of the array

    tab[u]   = (u == 0) ? a : a + tab[u - 1];
    tab[u + 1] = b + tab[u];
    tab[u + 2] = c + tab[u + 1];

    return true;
}


/* Below this line are replacement functions
   for the disabled one that you can find above. */

size_t DemodPCF7931(uint8_t **outBlocks, bool ledcontrol) {

    // 2021 iceman, memor
    uint8_t bits[256] = {0x00};
    uint8_t blocks[8][16];
    uint8_t *dest = BigBuf_get_addr();

    int g_GraphTraceLen = BigBuf_max_traceLen();
//    if (g_GraphTraceLen > 18000) {
//        g_GraphTraceLen = 18000;
//    }

    int i, j, lastval, bitidx, half_switch;
    int clock = 64;
    int tolerance = clock / 8;
    int block_done, start_reading_blocks;
    int lc = 0;
    size_t num_blocks = 0;
    int lmin = 64, lmax = 192;

    BigBuf_Clear_keep_EM();
    LFSetupFPGAForADC(LF_DIVISOR_125, true);
    DoAcquisition_default(0, true, ledcontrol);

    // Decode transmission
    bitidx = 0;
    lastval = 0;
    block_done = 0;
    half_switch = 0;
    start_reading_blocks = 0;

    for (i = 2; i < g_GraphTraceLen; i++) {

        if (   ((dest[i - 2] < lmax) && (dest[i - 1] < lmax) && (dest[i] >= lmax))
            || ((dest[i - 2] > lmin) && (dest[i - 1] > lmin) && (dest[i] <= lmin)) ) {

            // Get T0 len
            lc = i - lastval;
            lastval = i;

            // Skip First
            if (lastval == 0) { continue; }

            //
            if (ABS(lc - clock / 4) < tolerance) {
                // === 16T0 ===
                if (start_reading_blocks) {
                    block_done = 1;
                    start_reading_blocks = 0;
                    // Dbprintf("[cb] 16T0 - block done");
                } else {
                    // Dbprintf("[cb] 16T0");
                }
            } else if (ABS(lc - clock / 2) < tolerance) {
                // === 32T0 ===
                if (start_reading_blocks) {
                    if (half_switch == 1) {
                        // BIT 0
                        bits[bitidx++] = 0;
                        half_switch = 0;
                        // Dbprintf("[cb] 32T0 bit = 0");
                    } else {
                        half_switch++;
                        // Dbprintf("[cb] 32T0");
                    }
                }
            } else if (ABS(lc - clock) < tolerance) {
                // === 64T0 ===
                if (start_reading_blocks) {
                    // BIT 1
                    bits[bitidx++] = 1;
                    //Dbprintf("[cb] 64T0 bit = 1");
                }
            } else if (ABS(lc - clock * 2) < tolerance) {
                // === 128T0 ===
                // Dbprintf("[cb] 128T0");
            // 128 + 16 + 32 + 32 = 208
            } else if (ABS(lc - clock * 2 - clock - clock / 4) < tolerance) {
                // === 208T0 ===
                // Dbprintf("[cb] 208T0 - start reading block");
                // Dbprintf("[cb] 208T0 - start reading block at: %d", i);
                start_reading_blocks = 1;
                bitidx = 0;
                block_done = 0;
                half_switch = 0;
            } else {
                // === Unknown ===
                // Dbprintf("[cb] Unknown pulse T0 [%d]", lc);
            }

            if (block_done == 1) {
                // Dbprintf("[cb] bitidx = %d, num_blocks = %d", bitidx, num_blocks);
                if (bitidx == 128) {
                    for (j = 0; j < 16; ++j) {
                        blocks[num_blocks][j] =
                            128 * bits[j * 8 + 7] +
                             64 * bits[j * 8 + 6] +
                             32 * bits[j * 8 + 5] +
                             16 * bits[j * 8 + 4] +
                              8 * bits[j * 8 + 3] +
                              4 * bits[j * 8 + 2] +
                              2 * bits[j * 8 + 1] +
                              1 * bits[j * 8 + 0];
                    }
                    num_blocks++;
                }
                bitidx = 0;
                block_done = 0;
                half_switch = 0;
            }
        }
        if (num_blocks == 4) {
            break;
        }
    }
    memcpy(outBlocks, blocks, 16 * num_blocks);

    return num_blocks;

}

bool CanBeBlock0PCF7931(uint8_t *block) {
    // if PAC=1
    if ((block[7] & 0x01) == 0x01) {
        // password is superseded by 0 if PAC = 1
        if (memcmp(block, "\x00\x00\x00\x00\x00\x00\x00", 7) == 0) {
            return true;
        }
    // if PAC = 0
    } else {
        // password is readable if PAC = 0
        // we assume that the password is different than all zeros
        // the default password of a new pfc7931as apparently is:
        // 0x56 0x55 0x55 0x55 0x55 0x55 0x55
        if (memcmp(block, "\x00\x00\x00\x00\x00\x00\x00", 7) != 0) {
            return true;
        }
    }
    return false;
}

bool CanBeBlock1PCF7931(const uint8_t *block, uint8_t blocks_cnt) {
    // uint8_t RB1 = block[14] & 0x80;     // 10000000b
    uint8_t RFB = block[14] & 0x07;     // 00000111b
    uint8_t RLB = block[15] & 0x07;     // 00000111b

    // count how many blocks the pcf will send
    // this try replicate the modulo count of the pcf
    int transmitted_blocks = 1;
    if (RFB != RLB) {
        uint8_t i = RFB;
        while (i != RLB) {
            transmitted_blocks++;
            i = (i + 1) & 7;
        }
    }
    if (transmitted_blocks == blocks_cnt) {
        return true;
    }
    return false;
}

void ReadPCF7931(bool ledcontrol) {
    int max_blocks = 8;                 // readable blocks
    uint8_t memory_blocks[8][17];       // PCF content (17 as 1 byte as service)
    uint8_t single_blocks[8][17];       // PFC blocks with unknown position (17 as 1 byte as service)
    int single_blocks_cnt = 0;

    size_t n = 0;                       // transmitted blocks
    uint8_t tmp_blocks[4][16];          // temporary read buffer

    int errors = 0;                     // error counter
    int tries = 1;                      // tries counter

    bool blocks_sorted = false;         // were we able to sort the single blocks?

    memset(memory_blocks, 0, 8 * 17 * sizeof(uint8_t));
    memset(single_blocks, 0, 8 * 17 * sizeof(uint8_t));

    int i = 0, j = 0, k = 0;

    do {

        Dbprintf("Try number %d of 50", tries);

        // init memory
        memset(tmp_blocks, 0, 4 * 16 * sizeof(uint8_t));
        n = DemodPCF7931((uint8_t **)tmp_blocks, ledcontrol);
        if (!n) {
            ++errors;
        }

        // exit if no block is received
        if (errors >= 10 && single_blocks_cnt == 0) {
            if (g_dbglevel >= DBG_INFO) {
                Dbprintf("[!!] Error, no tag or bad tag");
            }
            return;
        }

        // exit if too many errors during reading
//        if (tries > 50 && (2 * errors > tries)) {
//            if (g_dbglevel >= DBG_INFO) {
//                Dbprintf("[!!] Error reading the tag, only partial content");
//            }
//            goto end;
//        }

        // exit if too many retries
        if (tries >= 50) {
            //if (g_dbglevel >= DBG_INFO) {
                Dbprintf("[!!] Too many retries while reading, only partial content");
            //}
            goto end;
        }

        // got some block from demod function so now try to put them in sequence

        // THIS CODE ASSUME THAT ALL BLOCKS ARE DIFFERENT
        // THIS CODE ASSUME THAT ALL BLOCKS ARE DIFFERENT
        // THIS CODE ASSUME THAT ALL BLOCKS ARE DIFFERENT

        // add blocks to single blocks list, keeping them in the right order
        if (single_blocks_cnt < max_blocks) {
            // if we have some block from previous demod
            // try to align with them and save only new ones
            bool single_blocks_aligned = false;
            if (single_blocks_cnt > 0) {
//                print_result("[cb] single_blocks[0].", single_blocks[0], 16);
//                print_result("[cb] single_blocks[1].", single_blocks[1], 16);
//                print_result("[cb] single_blocks[2].", single_blocks[2], 16);
//                print_result("[cb] single_blocks[3].", single_blocks[3], 16);
//                print_result("[cb] single_blocks[4].", single_blocks[4], 16);
//                print_result("[cb] single_blocks[5].", single_blocks[5], 16);
//                print_result("[cb] single_blocks[6].", single_blocks[6], 16);
//                print_result("[cb] single_blocks[7].", single_blocks[7], 16);
//                print_result("[cb] tmp_blocks[0]....", tmp_blocks[0], 16);
//                print_result("[cb] tmp_blocks[1]....", tmp_blocks[1], 16);
//                print_result("[cb] tmp_blocks[2]....", tmp_blocks[2], 16);
//                print_result("[cb] tmp_blocks[3]....", tmp_blocks[3], 16);

                // Try to find two equal blocks from previous saved demod
                // so we can be almost sure that the sequence of the block
                // is correct. We do not know which is the first block but
                // we try to keep the correct reading order

                // loop tmp_blocks
                bool broke = false;
                for (j = 0; j < n - 1; j++) {
                    // loop single blocks
                    for (i = 0; i < single_blocks_cnt - 1; i++) {
                        // memcmp return 0 if equal
                        if ( (memcmp(single_blocks[i], tmp_blocks[j], 16) == 0)
                          && (memcmp(single_blocks[i+1], tmp_blocks[j+1], 16) == 0) ) {
                            single_blocks_aligned = true;
                            broke = true;
                            break;
                        }
                    }
                    if (broke) {
                        break;
                    }
                }
//                if (!broke) {
//                    single_blocks_aligned = true;
//                }
            } else {
                j = 0;
                single_blocks_aligned = true;
            }

            // If we were able to align with a previous demod
            if (single_blocks_aligned) {
                //Dbprintf("aligned at i:%d j:%d", i, j);

                // New demod add block before
                if (i < j) {
                    //Dbprintf("adding blocks before", i, j);
                    // add space on top
                    for (k = (single_blocks_cnt - 1); k >= 0; k--) {
                        memcpy(single_blocks[k + (j - i)], single_blocks[k], 16);
                    }
                    // add blocks on top
                    for (k = 0; k < j; k++) {
                        memcpy(single_blocks[k], tmp_blocks[k], 16);
                        single_blocks_cnt++;
                    }
                }

                // New demod are aligned or add block at end
                if (i >= j) {
                    //Dbprintf("adding blocks after", i, j);
                    for (; j < n; j++) {
                        // save in list, but only untill we have space
                        if (i < max_blocks) {
                            // copy block to single block list
                            memcpy(single_blocks[i], tmp_blocks[j], 16);
                            // increment single block count if necessary
                            if (i == single_blocks_cnt) {
                                single_blocks_cnt++;
                            }
                            // move pointer to next single block
                            i++;
                        }
                    }
                }

            }
        }

        if (g_dbglevel >= DBG_EXTENDED) {
            Dbprintf("(dbg) got %d blocks (%d/%d found) (%d tries, %d errors)", n, single_blocks_cnt, max_blocks, tries, errors);
        }

        // count tries
        ++tries;

        // skip operations if BUTTON pressed
        // also usefull in case of a pcf that transmit few than 8 blocks
        if (BUTTON_PRESS()) {
            if (g_dbglevel >= DBG_EXTENDED) {
                Dbprintf("Button pressed, stopping.");
            }
            goto end;
        }

    } while (single_blocks_cnt < max_blocks);

    // Try to reorder blocks
    blocks_sorted = false;
    // search for a Block0 followed by a Block1
    for (i = 0; i < single_blocks_cnt - 1; ++i) {
        if ( CanBeBlock0PCF7931(single_blocks[i]) &&
             CanBeBlock1PCF7931(single_blocks[i + 1], single_blocks_cnt) ) {
            // move data in memory
            for (j = 0; j < single_blocks_cnt ; j++) {
                memcpy(memory_blocks[j], single_blocks[i], 16);
                i++;
                if (i == single_blocks_cnt) {
                    i = 0;
                }
            }
            blocks_sorted = true;
            break;
        }
    }
    // search for a single Block1
    if (!blocks_sorted) {
        for (i = 0; i < single_blocks_cnt - 1; ++i) {
            if ( CanBeBlock1PCF7931(single_blocks[i], single_blocks_cnt) ) {
                // move data in memory
                for (j = 0; j < single_blocks_cnt ; j++) {
                    memcpy(memory_blocks[j], single_blocks[i], 16);
                    i++;
                    if (i == single_blocks_cnt) {
                        i = 0;
                    }
                }
                blocks_sorted = true;
                break;
            }
        }
    }
    // search for a single Block0
    if (!blocks_sorted) {
        for (i = 0; i < single_blocks_cnt - 1; ++i) {
            if ( CanBeBlock0PCF7931(single_blocks[i]) ) {
                // move data in memory
                for (j = 0; j < single_blocks_cnt ; j++) {
                    memcpy(memory_blocks[j], single_blocks[i], 16);
                    i++;
                    if (i == single_blocks_cnt) {
                        i = 0;
                    }
                }
                blocks_sorted = true;
                break;
            }
        }
    }

end:

    if (blocks_sorted) {
        Dbprintf("-----------------------------------------");
        Dbprintf("Blocks read, in transmitted order:");
        Dbprintf("0: block could be a Block 0");
        Dbprintf("1: block could be a Block 1");
        Dbprintf("-----------------------------------------");
        for (i = 0; i < single_blocks_cnt; ++i) {
            Dbprintf("[%02d] %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x (%c %c)",
                i,
                memory_blocks[i][ 0],
                memory_blocks[i][ 1],
                memory_blocks[i][ 2],
                memory_blocks[i][ 3],
                memory_blocks[i][ 4],
                memory_blocks[i][ 5],
                memory_blocks[i][ 6],
                memory_blocks[i][ 7],
                memory_blocks[i][ 8],
                memory_blocks[i][ 9],
                memory_blocks[i][10],
                memory_blocks[i][11],
                memory_blocks[i][12],
                memory_blocks[i][13],
                memory_blocks[i][14],
                memory_blocks[i][15],
                CanBeBlock0PCF7931(memory_blocks[i]) ? '0' : ' ',
                CanBeBlock1PCF7931(memory_blocks[i], single_blocks_cnt) ? '1' : ' '
                );
        }
        Dbprintf("-----------------------------------------");
    }
    else if (single_blocks_cnt > 0) {
        Dbprintf("-----------------------------------------");
        Dbprintf("Blocks read, in transmitted order:");
        Dbprintf("0: block could be a Block 0");
        Dbprintf("1: block could be a Block 1");
        Dbprintf("-----------------------------------------");
        for (i = 0; i < single_blocks_cnt; ++i) {
            Dbprintf("[%02d] %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x (%c %c)",
                i,
                single_blocks[i][ 0],
                single_blocks[i][ 1],
                single_blocks[i][ 2],
                single_blocks[i][ 3],
                single_blocks[i][ 4],
                single_blocks[i][ 5],
                single_blocks[i][ 6],
                single_blocks[i][ 7],
                single_blocks[i][ 8],
                single_blocks[i][ 9],
                single_blocks[i][10],
                single_blocks[i][11],
                single_blocks[i][12],
                single_blocks[i][13],
                single_blocks[i][14],
                single_blocks[i][15],
                CanBeBlock0PCF7931(single_blocks[i]) ? '0' : ' ',
                CanBeBlock1PCF7931(single_blocks[i], single_blocks_cnt) ? '1' : ' '
                );
        }
        Dbprintf("-----------------------------------------");
    } else {
        Dbprintf("-----------------------------------------");
        Dbprintf("No blocks read.");
        Dbprintf("-----------------------------------------");
    }
    reply_mix(CMD_ACK, 0, 0, 0, 0, 0);
}




/* vim: set tabstop=4 shiftwidth=4 expandtab autoindent : */

