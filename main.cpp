#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/divider.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/interp.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include <hardware/sync.h>
#include <pico/multicore.h>
#include <hardware/flash.h>
#include <memory>
#include <math.h>
#if HSTX == 0
#include <util/dump_bin.h>
#include <util/exclusive_proc.h>
#include <util/work_meter.h>
#include <dvi/dvi.h>
#else
#include "hardware/irq.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/structs/hstx_ctrl.h"
#include "hardware/structs/hstx_fifo.h"
#include "hardware/structs/sio.h"
#include "pico/sem.h"
#include "myringbuffer.h"
#endif
#include <string.h>
#include <stdarg.h>
#include <algorithm>

#include <InfoNES.h>
#include <InfoNES_System.h>
#include <InfoNES_pAPU.h>
#include "bsp/board_api.h"
#include <tusb.h>
#include <gamepad.h>
#include "rom_selector.h"

const uint LED_PIN = 23; // PICO_DEFAULT_LED_PIN;

#ifndef DVICONFIG
// #define DVICONFIG dviConfig_PicoDVI
#define DVICONFIG dviConfig_AdafruitMetroRP2350 // dviConfig_PimoroniDemoDVSock
#endif

uint8_t framebuffer1[320 * 240];
uint8_t framebuffer2[320 * 240];
uint8_t *framebufferCore0 = framebuffer1;
uint8_t *framebufferCore1 = framebuffer2;

// Shared state
volatile bool framebuffer1_ready = false;
volatile bool framebuffer2_ready = false;
volatile bool use_framebuffer1 = true; // Toggle flag
volatile bool framebuffer1_rendering = false;
volatile bool framebuffer2_rendering = false;
// Mutex for synchronization
mutex_t framebuffer_mutex;

namespace
{
    constexpr uint32_t CPUFreqKHz = 252000; // 324000; // 252000;
#if HSTX == 0
    constexpr dvi::Config dviConfig_PicoDVI = {
        .pinTMDS = {10, 12, 14},
        .pinClock = 8,
        .invert = true,
    };

    constexpr dvi::Config dviConfig_PicoDVISock = {
        .pinTMDS = {12, 18, 16},
        .pinClock = 14,
        .invert = false,
    };

    // Pimoroni Digital Video, SD Card & Audio Demo Board
    constexpr dvi::Config dviConfig_PimoroniDemoDVSock = {
        .pinTMDS = {8, 10, 12},
        .pinClock = 6,
        .invert = true,
    };
    // FruitJam
    constexpr dvi::Config dviConfig_AdafruitMetroRP2350 = {
        .pinTMDS = {18, 16, 12},
        .pinClock = 14,
        .invert = false,
    };
    std::unique_ptr<dvi::DVI> dvi_;
    util::ExclusiveProc exclProc_;
#else

// ----------------------------------------------------------------------------
// DVI constants
#define RBG332 1
#define TMDS_CTRL_00 0x354u
#define TMDS_CTRL_01 0x0abu
#define TMDS_CTRL_10 0x154u
#define TMDS_CTRL_11 0x2abu

#define SYNC_V0_H0 (TMDS_CTRL_00 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V0_H1 (TMDS_CTRL_01 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V1_H0 (TMDS_CTRL_10 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V1_H1 (TMDS_CTRL_11 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))

#define MODE_H_SYNC_POLARITY 0
#define MODE_H_FRONT_PORCH 16
#define MODE_H_SYNC_WIDTH 96
#define MODE_H_BACK_PORCH 48
#define MODE_H_ACTIVE_PIXELS 640

#define MODE_V_SYNC_POLARITY 0
#define MODE_V_FRONT_PORCH 10
#define MODE_V_SYNC_WIDTH 2
#define MODE_V_BACK_PORCH 33
#define MODE_V_ACTIVE_LINES 480

#define MODE_H_TOTAL_PIXELS (                \
    MODE_H_FRONT_PORCH + MODE_H_SYNC_WIDTH + \
    MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS)
#define MODE_V_TOTAL_LINES (                 \
    MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH + \
    MODE_V_BACK_PORCH + MODE_V_ACTIVE_LINES)

#define HSTX_CMD_RAW (0x0u << 12)
#define HSTX_CMD_RAW_REPEAT (0x1u << 12)
#define HSTX_CMD_TMDS (0x2u << 12)
#define HSTX_CMD_TMDS_REPEAT (0x3u << 12)
#define HSTX_CMD_NOP (0xfu << 12)

    // ----------------------------------------------------------------------------
    // HSTX command lists

    // Lists are padded with NOPs to be >= HSTX FIFO size, to avoid DMA rapidly
    // pingponging and tripping up the IRQs.

    static uint32_t vblank_line_vsync_off[] = {
        HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH,
        SYNC_V1_H1,
        HSTX_CMD_RAW_REPEAT | MODE_H_SYNC_WIDTH,
        SYNC_V1_H0,
        HSTX_CMD_RAW_REPEAT | (MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS),
        SYNC_V1_H1,
        HSTX_CMD_NOP};

    static uint32_t vblank_line_vsync_on[] = {
        HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH,
        SYNC_V0_H1,
        HSTX_CMD_RAW_REPEAT | MODE_H_SYNC_WIDTH,
        SYNC_V0_H0,
        HSTX_CMD_RAW_REPEAT | (MODE_H_BACK_PORCH + MODE_H_ACTIVE_PIXELS),
        SYNC_V0_H1,
        HSTX_CMD_NOP};

    static uint32_t vactive_line[] = {
        HSTX_CMD_RAW_REPEAT | MODE_H_FRONT_PORCH,
        SYNC_V1_H1,
        HSTX_CMD_NOP,
        HSTX_CMD_RAW_REPEAT | MODE_H_SYNC_WIDTH,
        SYNC_V1_H0,
        HSTX_CMD_NOP,
        HSTX_CMD_RAW_REPEAT | MODE_H_BACK_PORCH,
        SYNC_V1_H1,
        HSTX_CMD_TMDS | MODE_H_ACTIVE_PIXELS};

    // ----------------------------------------------------------------------------
    // DMA logic

#define DMACH_PING 0
#define DMACH_PONG 1

    // First we ping. Then we pong. Then... we ping again.
    static bool dma_pong = false;

    // A ping and a pong are cued up initially, so the first time we enter this
    // handler it is to cue up the second ping after the first ping has completed.
    // This is the third scanline overall (-> =2 because zero-based).
    static uint v_scanline = 2;

    // During the vertical active period, we take two IRQs per scanline: one to
    // post the command list, and another to post the pixels.
    static bool vactive_cmdlist_posted = false;

    uint16_t tempbuf[320]; // 640 pixels, 2 bytes each
    inline uint8_t RGB565_to_RGB332(uint16_t rgb565)
    {
        // Extract the red, green, and blue components from RGB565
        uint8_t red = (rgb565 >> 11) & 0x1F;  // 5 bits
        uint8_t green = (rgb565 >> 5) & 0x3F; // 6 bits
        uint8_t blue = rgb565 & 0x1F;         // 5 bits

        // Scale down to RGB332
        uint8_t r = (red >> 2) & 0x07;   // Scale 5 bits to 3 bits
        uint8_t g = (green >> 3) & 0x07; // Scale 6 bits to 3 bits
        uint8_t b = (blue >> 3) & 0x03;  // Scale 5 bits to 2 bits

        // Combine into RGB332 format
        return (r << 5) | (g << 2) | b;
    }
    void __scratch_x("") dma_irq_handler()
    {
        // dma_pong indicates the channel that just finished, which is the one
        // we're about to reload.
        uint ch_num = dma_pong ? DMACH_PONG : DMACH_PING;
        dma_channel_hw_t *ch = &dma_hw->ch[ch_num];
        dma_hw->intr = 1u << ch_num;
        dma_pong = !dma_pong;
        uint8_t *framebuf = framebufferCore1;
        if (v_scanline == 0)
        {
            // printf("Vsync %d\n", v_scanline);
            mutex_enter_blocking(&framebuffer_mutex);
            framebuf = framebufferCore1;
            mutex_exit(&framebuffer_mutex);
        }
        if (v_scanline >= MODE_V_FRONT_PORCH && v_scanline < (MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH))
        {
            // printf("Vsync %d\n", v_scanline);
            ch->read_addr = (uintptr_t)vblank_line_vsync_on;
            ch->transfer_count = count_of(vblank_line_vsync_on);
        }
        else if (v_scanline < MODE_V_FRONT_PORCH + MODE_V_SYNC_WIDTH + MODE_V_BACK_PORCH)
        {
            // printf("Vsync %d\n", v_scanline);
            ch->read_addr = (uintptr_t)vblank_line_vsync_off;
            ch->transfer_count = count_of(vblank_line_vsync_off);
        }
        else if (!vactive_cmdlist_posted)
        {
            ch->read_addr = (uintptr_t)vactive_line;
            ch->transfer_count = count_of(vactive_line);
            vactive_cmdlist_posted = true;
        }
        else
        {
#ifdef RBG332
            ch->read_addr = (uintptr_t)tempbuf; // [(v_scanline - (MODE_V_TOTAL_LINES - MODE_V_ACTIVE_LINES)) * MODE_H_ACTIVE_PIXELS];
            int scanlineIndex = (v_scanline - (MODE_V_TOTAL_LINES - MODE_V_ACTIVE_LINES));
            uint8_t *scanlinepointer = framebuf + ((scanlineIndex >> 1) * 320);
            uint16_t *p = &tempbuf[0];
            for (int i = 0; i < 320; i++)
            {
                *p++ = scanlinepointer[i] << 8 | scanlinepointer[i];
            }
            ch->transfer_count = MODE_H_ACTIVE_PIXELS / sizeof(uint32_t);
#else
            // 640x480 RGB565 is too large to fit into memory. The include file is 640 x 240 pixels.
            // The image is duplicated to the lower half of the screen.
            if (v_scanline > 523 - 480 + 240)
            {
                // ch->read_addr = (uintptr_t)&framebuf[(v_scanline - 239 - (MODE_V_TOTAL_LINES - MODE_V_ACTIVE_LINES)) * MODE_H_ACTIVE_PIXELS * 2];
            }
            else
            {
                // ch->read_addr = (uintptr_t)&framebuf[(v_scanline - (MODE_V_TOTAL_LINES - MODE_V_ACTIVE_LINES)) * MODE_H_ACTIVE_PIXELS * 2];
            }
            ch->transfer_count = MODE_H_ACTIVE_PIXELS * 2 / sizeof(uint32_t);
#endif

            vactive_cmdlist_posted = false;
            // printf("Scanline %d\n", v_scanline);
        }

        if (!vactive_cmdlist_posted)
        {

            v_scanline = (v_scanline + 1) % MODE_V_TOTAL_LINES;
        }
    }

#endif
    extern "C" char nes_rom[];
    static uintptr_t NES_FILE_ADDR = reinterpret_cast<uintptr_t>(&nes_rom[0]); // 0x10080000;

    ROMSelector romSelector_;

    enum class ScreenMode
    {
        SCANLINE_8_7,
        NOSCANLINE_8_7,
        SCANLINE_1_1,
        NOSCANLINE_1_1,
        MAX,
    };
    ScreenMode screenMode_{}; // = ScreenMode::SCANLINE_1_1;

    bool scaleMode8_7_ = true; // true

    void applyScreenMode()
    {
#if HSTX == 0
        bool scanLine = false;

        switch (screenMode_)
        {
        case ScreenMode::SCANLINE_1_1:
            scaleMode8_7_ = false;
            scanLine = true;
            break;

        case ScreenMode::SCANLINE_8_7:
            scaleMode8_7_ = true;
            scanLine = true;
            break;

        case ScreenMode::NOSCANLINE_1_1:
            scaleMode8_7_ = false;
            scanLine = false;
            break;

        case ScreenMode::NOSCANLINE_8_7:
            scaleMode8_7_ = true;
            scanLine = false;
            break;
        default:
            break;
        }

        dvi_->setScanLine(scanLine);

#endif
    }
}

#define CC(x) ((x >> 1) & 15) | (((x >> 6) & 15) << 4) | (((x >> 11) & 15) << 8)
const WORD __not_in_flash_func(NesPalette)[64] = {
    CC(0x39ce), CC(0x1071), CC(0x0015), CC(0x2013), CC(0x440e), CC(0x5402), CC(0x5000), CC(0x3c20),
    CC(0x20a0), CC(0x0100), CC(0x0140), CC(0x00e2), CC(0x0ceb), CC(0x0000), CC(0x0000), CC(0x0000),
    CC(0x5ef7), CC(0x01dd), CC(0x10fd), CC(0x401e), CC(0x5c17), CC(0x700b), CC(0x6ca0), CC(0x6521),
    CC(0x45c0), CC(0x0240), CC(0x02a0), CC(0x0247), CC(0x0211), CC(0x0000), CC(0x0000), CC(0x0000),
    CC(0x7fff), CC(0x1eff), CC(0x2e5f), CC(0x223f), CC(0x79ff), CC(0x7dd6), CC(0x7dcc), CC(0x7e67),
    CC(0x7ae7), CC(0x4342), CC(0x2769), CC(0x2ff3), CC(0x03bb), CC(0x0000), CC(0x0000), CC(0x0000),
    CC(0x7fff), CC(0x579f), CC(0x635f), CC(0x6b3f), CC(0x7f1f), CC(0x7f1b), CC(0x7ef6), CC(0x7f75),
    CC(0x7f94), CC(0x73f4), CC(0x57d7), CC(0x5bf9), CC(0x4ffe), CC(0x0000), CC(0x0000), CC(0x0000)};
// NES Palette (RGB332 values):
// https://roger-random.github.io/RGB332_color_wheel_three.js/
const char __not_in_flash_func(NesPaletteRGB332)[] = {
    //  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
    0xb6, 0x27, 0x03, 0x2a, 0x61, 0x80, 0xa0, 0x60, 0x68, 0x50, 0x30, 0x10, 0x2a, 0x00, 0x00, 0x00,
    0xb6, 0x4b, 0x2f, 0x8f, 0xea, 0xa1, 0xe8, 0xc8, 0x90, 0x18, 0x14, 0x34, 0x53, 0x00, 0x00, 0x00,
    0xff, 0x7b, 0x73, 0xb3, 0xcf, 0xca, 0xcd, 0xf5, 0xf8, 0xbc, 0x39, 0x9e, 0x5f, 0xb6, 0x00, 0x00,
    0xff, 0x9b, 0xb7, 0xd7, 0xf7, 0xd2, 0xfa, 0xfe, 0xd9, 0xdd, 0xbd, 0xdf, 0xbf, 0xb6, 0x00, 0x00};
uint32_t getCurrentNVRAMAddr()
{
    if (!romSelector_.getCurrentROM())
    {
        return {};
    }
    int slot = romSelector_.getCurrentNVRAMSlot();
    if (slot < 0)
    {
        return {};
    }
    printf("SRAM slot %d\n", slot);
    return NES_FILE_ADDR - SRAM_SIZE * (slot + 1);
}

void saveNVRAM()
{
    return;
    if (!SRAMwritten)
    {
        printf("SRAM not updated.\n");
        return;
    }

    // mutex_enter_blocking(&framebuffer_mutex);
    printf("save SRAM\n");
#if HSTX == 0
    exclProc_.setProcAndWait([]
                             {
#endif
                                 static_assert((SRAM_SIZE & (FLASH_SECTOR_SIZE - 1)) == 0);
                                 if (auto addr = getCurrentNVRAMAddr())
                                 {
                                     auto ofs = addr - XIP_BASE;
                                     printf("write flash %x\n", ofs);
                                     {
                                         flash_range_erase(ofs, SRAM_SIZE);
                                         flash_range_program(ofs, SRAM, SRAM_SIZE);
                                     }
                                 }
#if HSTX == 0
                             });
#endif
    printf("done\n");
    // mutex_exit(&framebuffer_mutex);
    SRAMwritten = false;
}

void loadNVRAM()
{
    if (auto addr = getCurrentNVRAMAddr())
    {
        // mutex_enter_blocking(&framebuffer_mutex);
        printf("load SRAM %x\n", addr);
        memcpy(SRAM, reinterpret_cast<void *>(addr), SRAM_SIZE);
        // mutex_exit(&framebuffer_mutex);
    }
    SRAMwritten = false;
}

void InfoNES_PadState(DWORD *pdwPad1, DWORD *pdwPad2, DWORD *pdwSystem)
{
    static constexpr int LEFT = 1 << 6;
    static constexpr int RIGHT = 1 << 7;
    static constexpr int UP = 1 << 4;
    static constexpr int DOWN = 1 << 5;
    static constexpr int SELECT = 1 << 2;
    static constexpr int START = 1 << 3;
    static constexpr int A = 1 << 0;
    static constexpr int B = 1 << 1;

    static DWORD prevButtons[2]{};
    static int rapidFireMask[2]{};
    static int rapidFireCounter = 0;

    ++rapidFireCounter;
    bool reset = false;

    for (int i = 0; i < 2; ++i)
    {
        auto &dst = i == 0 ? *pdwPad1 : *pdwPad2;
        auto &gp = io::getCurrentGamePadState(i);

        int v = (gp.buttons & io::GamePadState::Button::LEFT ? LEFT : 0) |
                (gp.buttons & io::GamePadState::Button::RIGHT ? RIGHT : 0) |
                (gp.buttons & io::GamePadState::Button::UP ? UP : 0) |
                (gp.buttons & io::GamePadState::Button::DOWN ? DOWN : 0) |
                (gp.buttons & io::GamePadState::Button::A ? A : 0) |
                (gp.buttons & io::GamePadState::Button::B ? B : 0) |
                (gp.buttons & io::GamePadState::Button::SELECT ? SELECT : 0) |
                (gp.buttons & io::GamePadState::Button::START ? START : 0) |
                0;

        int rv = v;
        if (rapidFireCounter & 2)
        {
            // 15 fire/sec
            rv &= ~rapidFireMask[i];
        }

        dst = rv;

        auto p1 = v;
        auto pushed = v & ~prevButtons[i];
        if (p1 & SELECT)
        {
            if (pushed & LEFT)
            {
                saveNVRAM();
                romSelector_.prev();
                reset = true;
            }
            if (pushed & RIGHT)
            {
                saveNVRAM();
                romSelector_.next();
                reset = true;
            }
            if (pushed & START)
            {
                saveNVRAM();
                reset = true;
            }
            if (pushed & A)
            {
                rapidFireMask[i] ^= io::GamePadState::Button::A;
            }
            if (pushed & B)
            {
                rapidFireMask[i] ^= io::GamePadState::Button::B;
            }
            if (pushed & UP)
            {
                screenMode_ = static_cast<ScreenMode>((static_cast<int>(screenMode_) - 1) & 3);
                applyScreenMode();
            }
            else if (pushed & DOWN)
            {
                screenMode_ = static_cast<ScreenMode>((static_cast<int>(screenMode_) + 1) & 3);
                applyScreenMode();
            }
        }

        prevButtons[i] = v;
    }

    *pdwSystem = reset ? PAD_SYS_QUIT : 0;
}

void InfoNES_MessageBox(const char *pszMsg, ...)
{
    printf("[MSG]");
    va_list args;
    va_start(args, pszMsg);
    vprintf(pszMsg, args);
    va_end(args);
    printf("\n");
}

bool parseROM(const uint8_t *nesFile)
{
    memcpy(&NesHeader, nesFile, sizeof(NesHeader));
    if (!checkNESMagic(NesHeader.byID))
    {
        return false;
    }

    nesFile += sizeof(NesHeader);

    memset(SRAM, 0, SRAM_SIZE);

    if (NesHeader.byInfo1 & 4)
    {
        memcpy(&SRAM[0x1000], nesFile, 512);
        nesFile += 512;
    }

    auto romSize = NesHeader.byRomSize * 0x4000;
    ROM = (BYTE *)nesFile;
    nesFile += romSize;

    if (NesHeader.byVRomSize > 0)
    {
        auto vromSize = NesHeader.byVRomSize * 0x2000;
        VROM = (BYTE *)nesFile;
        nesFile += vromSize;
    }

    return true;
}

void InfoNES_ReleaseRom()
{
    ROM = nullptr;
    VROM = nullptr;
}

void InfoNES_SoundInit()
{
}

int InfoNES_SoundOpen(int samples_per_sync, int sample_rate)
{
    return 0;
}

void InfoNES_SoundClose()
{
}

int __not_in_flash_func(InfoNES_GetSoundBufferSize)()
{
#if HSTX == 0
    return dvi_->getAudioRingBuffer().getFullWritableSize();
#else
    return my_rb_free();
#endif
}

void __not_in_flash_func(InfoNES_SoundOutput)(int samples, BYTE *wave1, BYTE *wave2, BYTE *wave3, BYTE *wave4, BYTE *wave5)
{
#if HSTX == 0
    while (samples)
    {
        auto &ring = dvi_->getAudioRingBuffer();
        auto n = std::min<int>(samples, ring.getWritableSize());
        if (!n)
        {
            return;
        }
        auto p = ring.getWritePointer();

        int ct = n;
        while (ct--)
        {
            int w1 = *wave1++;
            int w2 = *wave2++;
            int w3 = *wave3++;
            int w4 = *wave4++;
            int w5 = *wave5++;
            //            w3 = w2 = w4 = w5 = 0;
            int l = w1 * 6 + w2 * 3 + w3 * 5 + w4 * 3 * 17 + w5 * 2 * 32;
            int r = w1 * 3 + w2 * 6 + w3 * 5 + w4 * 3 * 17 + w5 * 2 * 32;
            *p++ = {static_cast<short>(l), static_cast<short>(r)};

            // pulse_out = 0.00752 * (pulse1 + pulse2)
            // tnd_out = 0.00851 * triangle + 0.00494 * noise + 0.00335 * dmc

            // 0.00851/0.00752 = 1.131648936170213
            // 0.00494/0.00752 = 0.6569148936170213
            // 0.00335/0.00752 = 0.4454787234042554

            // 0.00752/0.00851 = 0.8836662749706228
            // 0.00494/0.00851 = 0.5804935370152762
            // 0.00335/0.00851 = 0.3936545240893067
        }

        ring.advanceWritePointer(n);
        samples -= n;
    }
#else
    for (int i = 0; i < samples; ++i)
    {
        int w1 = wave1[i];
        int w2 = wave2[i];
        int w3 = wave3[i];
        int w4 = wave4[i];
        int w5 = wave5[i];

        // Mix your channels to a 12-bit value (example mix, adjust as needed)
        // This works but some effects are silent:
        // int sample12 =  (w1 + w2 + w3 + w4 + w5); // Range depends on input
        // Below is a more complex mix that gives a better sound
        int sample12 = w1 * 6 + w2 * 3 + w3 * 5 + w4 * 3 * 17 + w5 * 2 * 32; //

        // Clamp to 0-4095 if needed
        if (sample12 < 0)
            sample12 = 0;
        if (sample12 > 4095)
            sample12 = 4095;

        // // Convert to 8-bit unsigned
        // uint8_t sample8 = (sample12 * 255) / 4095;
        my_rb_put(sample12);
        // outBuffer[outIndex++] = sample8;
    }

#endif
}

extern WORD PC;
int frameCounter = 0;
void InfoNES_LoadFrame()
{
#if HSTX == 0
    gpio_put(LED_PIN, hw_divider_s32_quotient_inlined(dvi_->getFrameCounter(), 60) & 1);
#else
    gpio_put(LED_PIN, hw_divider_s32_quotient_inlined(frameCounter++, 60) & 1);
#endif
    tuh_task();
    // switch framebuffers
    // Lock the mutex only to update shared state
    mutex_enter_blocking(&framebuffer_mutex);
    if (use_framebuffer1)
    {
        framebuffer1_ready = true;
        framebuffer2_ready = false;
    }
    else
    {
        framebuffer1_ready = false;
        framebuffer2_ready = true;
    }
    use_framebuffer1 = !use_framebuffer1; // Toggle the framebuffer
    if (use_framebuffer1)
    {
        framebufferCore0 = framebuffer1;
        framebufferCore1 = framebuffer2;
    }
    else
    {
        framebufferCore0 = framebuffer2;
        framebufferCore1 = framebuffer1;
    }
    mutex_exit(&framebuffer_mutex);
// Wait if core1 is still rendering the framebuffer whe just switched to
#if 0
    int start = time_us_64();
    while ((use_framebuffer1 && framebuffer1_rendering) || (!use_framebuffer1 && framebuffer2_rendering))
    {
        //printf("Core 0: Waiting for core 1 to finish rendering: %d\n", tell++);
        tight_loop_contents();
    }
    int end = time_us_64();
#endif
#if 0
    printf("Core 0: Switching framebuffers took %ld us\n", end - start);
#endif
    // continue emulation while the other core renders the framebuffer
}

namespace
{
#if HSTX == 0
    dvi::DVI::LineBuffer *currentLineBuffer_{};
#endif
}

void __not_in_flash_func(drawWorkMeterUnit)(int timing,
                                            [[maybe_unused]] int span,
                                            uint32_t tag)
{
    // if (timing >= 0 && timing < 640)
    // {
    //     auto p = currentLineBuffer_->data();
    //     p[timing] = tag; // tag = color
    // }
}

void __not_in_flash_func(drawWorkMeter)(int line)
{
    // if (!currentLineBuffer_)
    // {
    //     return;
    // }

    // memset(currentLineBuffer_->data(), 0, 64);
    // memset(&currentLineBuffer_->data()[320 - 32], 0, 64);
    // (*currentLineBuffer_)[160] = 0;
    // if (line == 4)
    // {
    //     for (int i = 1; i < 10; ++i)
    //     {
    //         (*currentLineBuffer_)[16 * i] = 31;
    //     }
    // }

    // constexpr uint32_t clocksPerLine = 800 * 10;
    // constexpr uint32_t meterScale = 160 * 65536 / (clocksPerLine * 2);
    // util::WorkMeterEnum(meterScale, 1, drawWorkMeterUnit);
    // //    util::WorkMeterEnum(160, clocksPerLine * 2, drawWorkMeterUnit);
}

WORD lineBuffer[320];
void __not_in_flash_func(InfoNES_PreDrawLine)(int line)
{
    // util::WorkMeterMark(0xaaaa);
    // auto b = dvi_->getLineBuffer();
    // util::WorkMeterMark(0x5555);
    uint8_t *tmpWorkline = &framebufferCore0[line * 320];
    InfoNES_SetLineBuffer(lineBuffer + 32, tmpWorkline + 32, 320);
    //    (*b)[319] = line + dvi_->getFrameCounter();

    // currentLineBuffer_ = b;
}

void __not_in_flash_func(InfoNES_PostDrawLine)(int line)
{
    // #if !defined(NDEBUG)
    //     util::WorkMeterMark(0xffff);
    //     drawWorkMeter(line);
    // #endif

    // assert(currentLineBuffer_);
    // dvi_->setLineBuffer(line, currentLineBuffer_);
    // currentLineBuffer_ = nullptr;
}

bool loadAndReset()
{
    auto rom = romSelector_.getCurrentROM();
    if (!rom)
    {
        printf("ROM does not exists.\n");
        return false;
    }

    if (!parseROM(rom))
    {
        printf("NES file parse error.\n");
        return false;
    }
    loadNVRAM();

    if (InfoNES_Reset() < 0)
    {
        printf("NES reset error.\n");
        return false;
    }

    return true;
}

int InfoNES_Menu()
{
    // InfoNES_Main() のループで最初に呼ばれる
    loadAndReset();
    return 0;
}

void __not_in_flash_func(core1_main)()
{
#if HSTX == 0
    while (true)
    {
        dvi_->registerIRQThisCore();
        dvi_->waitForValidLine();

        dvi_->start();
        while (!exclProc_.isExist())
        {
            if (scaleMode8_7_)
            {
                dvi_->convertScanBuffer12bppScaled16_7(34, 32, 288 * 2);
                // 34 + 252 + 34
                // 32 + 576 + 32
            }
            else
            {
                dvi_->convertScanBuffer12bpp();
            }
        }

        dvi_->unregisterIRQThisCore();
        dvi_->stop();

        exclProc_.processOrWaitIfExist();
    }
#else

#endif
}

// WORD buffer[320];
#if HSTX == 0
void __not_in_flash_func(coreFB_main)()
{

    // dvi_->registerIRQThisCore();
    //  dvi_->waitForValidLine();
    int fb1 = 0;
    int fb2 = 0;
    int frame = 0;
    // dvi_->start();
    while (true)
    {
        dvi_->registerIRQThisCore();
        dvi_->start();
        while (!exclProc_.isExist())
        {
            bool may_render = true;

            // Try to acquire the mutex to check for new frames
            if (mutex_try_enter(&framebuffer_mutex, NULL))
            {
                // Check if the other framebuffer is ready
                if (framebuffer1_ready && !framebuffer1_rendering)
                {
                    // printf("Core 1: Switching to framebuffer1\n");
                    framebuffer1_rendering = true;
                    may_render = true;
                    framebufferCore1 = framebuffer1;
                }
                else if (framebuffer2_ready && !framebuffer2_rendering)
                {
                    // printf("Core 1: Switching to framebuffer2\n");
                    framebuffer2_rendering = true;
                    may_render = true;
                    framebufferCore1 = framebuffer2;
                }
                mutex_exit(&framebuffer_mutex);
            }
            if (may_render)
            {
                // printf("Core 1: Rendering frame %s %d\n", current_framebuffer == framebuffer1 ? "framebuffer1" : "framebuffer2", frame++);
                for (int line = 4; line < 240 - 4; ++line)
                {
                    uint8_t *current_line = &framebufferCore1[line * 320];
                    for (int kol = 0; kol < 320; kol += 4)
                    {
                        // buffer[kol] = NesPalette[current_line[kol]];
                        // NesPalette[current_framebuffer[line * 320 + kol] & 0x3F];
                        buffer[kol] = NesPalette[current_line[kol]];
                        buffer[kol + 1] = NesPalette[current_line[kol + 1]];
                        buffer[kol + 2] = NesPalette[current_line[kol + 2]];
                        buffer[kol + 3] = NesPalette[current_line[kol + 3]];
                    }
                    if (scaleMode8_7_)
                    {
                        dvi_->convertScanBuffer12bppScaled16_7(34, 32, 288 * 2, line, buffer, 640);
                        // 34 + 252 + 34
                        // 32 + 576 + 32
                    }
                    else
                    {
                        // printf("line: %d\n", line);
                        dvi_->convertScanBuffer12bpp(line, buffer, 640);
                    }
                }
                // Mark the framebuffer as no longer being rendered
                mutex_enter_blocking(&framebuffer_mutex);
                if (framebufferCore1 == framebuffer1)
                {
                    framebuffer1_rendering = false;
                    fb1++;
                    fb2 = 0;
                }
                else
                {
                    framebuffer2_rendering = false;
                    fb2++;
                    fb1 = 0;
                }
                mutex_exit(&framebuffer_mutex);
                if (fb1 > 1)
                {
                    printf("fb1: %d\n", fb1);
                }
                if (fb2 > 1)
                {
                    printf("fb2: %d\n", fb2);
                }
            }
        }
        dvi_->unregisterIRQThisCore();
        dvi_->stop();

        exclProc_.processOrWaitIfExist();
    }
}
#else
void __not_in_flash_func(coreFB_main)()
{
    printf("DVI output example\n");

#ifdef RBG332
    printf("640x480 RGB332\n");
    // Configure HSTX's TMDS encoder for RGB332
    hstx_ctrl_hw->expand_tmds =
        2 << HSTX_CTRL_EXPAND_TMDS_L2_NBITS_LSB |
        0 << HSTX_CTRL_EXPAND_TMDS_L2_ROT_LSB |
        2 << HSTX_CTRL_EXPAND_TMDS_L1_NBITS_LSB |
        29 << HSTX_CTRL_EXPAND_TMDS_L1_ROT_LSB |
        1 << HSTX_CTRL_EXPAND_TMDS_L0_NBITS_LSB |
        26 << HSTX_CTRL_EXPAND_TMDS_L0_ROT_LSB;
#else
    // This should be RGB565
    printf("640x240 RGB565\n");
    hstx_ctrl_hw->expand_tmds =
        5 << HSTX_CTRL_EXPAND_TMDS_L2_NBITS_LSB | // 5 bits for red
        0 << HSTX_CTRL_EXPAND_TMDS_L2_ROT_LSB |   // No rotation for red
        6 << HSTX_CTRL_EXPAND_TMDS_L1_NBITS_LSB | // 6 bits for green
        29 << HSTX_CTRL_EXPAND_TMDS_L1_ROT_LSB |  // Rotation for green
        5 << HSTX_CTRL_EXPAND_TMDS_L0_NBITS_LSB | // 5 bits for blue
        26 << HSTX_CTRL_EXPAND_TMDS_L0_ROT_LSB;   // Rotation for blue
#endif
    // Pixels (TMDS) come in 4 8-bit chunks. Control symbols (RAW) are an
    // entire 32-bit word.
    hstx_ctrl_hw->expand_shift =
        4 << HSTX_CTRL_EXPAND_SHIFT_ENC_N_SHIFTS_LSB |
        8 << HSTX_CTRL_EXPAND_SHIFT_ENC_SHIFT_LSB |
        1 << HSTX_CTRL_EXPAND_SHIFT_RAW_N_SHIFTS_LSB |
        0 << HSTX_CTRL_EXPAND_SHIFT_RAW_SHIFT_LSB;

    // Serial output config: clock period of 5 cycles, pop from command
    // expander every 5 cycles, shift the output shiftreg by 2 every cycle.
    hstx_ctrl_hw->csr = 0;
    hstx_ctrl_hw->csr =
        HSTX_CTRL_CSR_EXPAND_EN_BITS |
        5u << HSTX_CTRL_CSR_CLKDIV_LSB |
        5u << HSTX_CTRL_CSR_N_SHIFTS_LSB |
        2u << HSTX_CTRL_CSR_SHIFT_LSB |
        HSTX_CTRL_CSR_EN_BITS;

    // Note we are leaving the HSTX clock at the SDK default of 125 MHz; since
    // we shift out two bits per HSTX clock cycle, this gives us an output of
    // 250 Mbps, which is very close to the bit clock for 480p 60Hz (252 MHz).
    // If we want the exact rate then we'll have to reconfigure PLLs.

    // HSTX outputs 0 through 7 appear on GPIO 12 through 19.
    // Pinout on Pico DVI sock:
    //
    //   GP12 D0+  GP13 D0-
    //   GP14 CK+  GP15 CK-
    //   GP16 D2+  GP17 D2-
    //   GP18 D1+  GP19 D1-

    // Assign clock pair to two neighbouring pins:
    hstx_ctrl_hw->bit[2] = HSTX_CTRL_BIT0_CLK_BITS;
    hstx_ctrl_hw->bit[3] = HSTX_CTRL_BIT0_CLK_BITS | HSTX_CTRL_BIT0_INV_BITS;
    for (uint lane = 0; lane < 3; ++lane)
    {
        // For each TMDS lane, assign it to the correct GPIO pair based on the
        // desired pinout:

        // HSTX Output Bit 0 → GPIO12
        // HSTX Output Bit 1 → GPIO13
        // HSTX Output Bit 2 → GPIO14
        // HSTX Output Bit 3 → GPIO15
        // HSTX Output Bit 4 → GPIO16
        // HSTX Output Bit 5 → GPIO17
        // HSTX Output Bit 6 → GPIO18
        // HSTX Output Bit 7 → GPIO19
        // lane_to_output_bit Array
        // The lane_to_output_bit array specifies which HSTX output bits are used for each TMDS lane:

        // Index 0: TMDS lane D0 (data lane 0)
        // Index 1: TMDS lane D1 (data lane 1)
        // Index 2: TMDS lane D2 (data lane 2)

        // Hardcoded mapping for now using Adafruit Metro RP2350
        // The mapping is fixed as follows:
        // D0+ = CPIO18, D0-=GPIO19, D1+=GPIO16, D1-=GPIO17, D2+-GPIO12, D2-=GPIO13
        // For the array {6, 4, 0}:

        // D0 (Index 0) is assigned to HSTX output bit 6 → GPIO18 (D0+) and GPIO19 (D0-).
        // D1 (Index 1) is assigned to HSTX output bit 4 → GPIO16 (D1+) and GPIO17 (D1-).
        // D2 (Index 2) is assigned to HSTX output bit 0 → GPIO12 (D2+) and GPIO13 (D2-).
        // https://learn.adafruit.com/adafruit-metro-rp2350/pinouts#hstx-connector-3193107
        static const int lane_to_output_bit[3] = {6, 4, 0}; // {0, 6, 4};
        int bit = lane_to_output_bit[lane];
        // Output even bits during first half of each HSTX cycle, and odd bits
        // during second half. The shifter advances by two bits each cycle.
        uint32_t lane_data_sel_bits =
            (lane * 10) << HSTX_CTRL_BIT0_SEL_P_LSB |
            (lane * 10 + 1) << HSTX_CTRL_BIT0_SEL_N_LSB;
        // The two halves of each pair get identical data, but one pin is inverted.
        hstx_ctrl_hw->bit[bit] = lane_data_sel_bits;
        hstx_ctrl_hw->bit[bit + 1] = lane_data_sel_bits | HSTX_CTRL_BIT0_INV_BITS;
    }

    for (int i = 12; i <= 19; ++i)
    {
        gpio_set_function(i, GPIO_FUNC_HSTX); // HSTX (was 0)
    }

    // Both channels are set up identically, to transfer a whole scanline and
    // then chain to the opposite channel. Each time a channel finishes, we
    // reconfigure the one that just finished, meanwhile the opposite channel
    // is already making progress.
    dma_channel_config c;
    c = dma_channel_get_default_config(DMACH_PING);
    channel_config_set_chain_to(&c, DMACH_PONG);
    channel_config_set_dreq(&c, DREQ_HSTX);
    dma_channel_configure(
        DMACH_PING,
        &c,
        &hstx_fifo_hw->fifo,
        vblank_line_vsync_off,
        count_of(vblank_line_vsync_off),
        false);
    c = dma_channel_get_default_config(DMACH_PONG);
    channel_config_set_chain_to(&c, DMACH_PING);
    channel_config_set_dreq(&c, DREQ_HSTX);
    dma_channel_configure(
        DMACH_PONG,
        &c,
        &hstx_fifo_hw->fifo,
        vblank_line_vsync_off,
        count_of(vblank_line_vsync_off),
        false);

    dma_hw->ints0 = (1u << DMACH_PING) | (1u << DMACH_PONG);
    dma_hw->inte0 = (1u << DMACH_PING) | (1u << DMACH_PONG);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    dma_channel_start(DMACH_PING);

    while (1)
        __wfi();
}
#endif
#if HSTX == 1
extern "C" void init_mcp4822();
#endif
int main()
{
#if HSTX == 0
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(10);
    set_sys_clock_khz(CPUFreqKHz, true);
#endif

    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    board_init();

#if CFG_TUH_RPI_PIO_USB
    printf("Using PIO USB.\n");
    board_init();
    tusb_rhport_init_t host_init = {
        .role = TUSB_ROLE_HOST,
        .speed = TUSB_SPEED_AUTO};
    tusb_init(BOARD_TUH_RHPORT, &host_init);

    if (board_init_after_tusb)
    {
        board_init_after_tusb();
    }
#else
    printf("Using internal USB.\n");
    tusb_init();
#endif
    romSelector_.init(NES_FILE_ADDR);

    // util::dumpMemory((void *)NES_FILE_ADDR, 1024);

#if 0
    //
    auto *i2c = i2c0;
    static constexpr int I2C_SDA_PIN = 16;
    static constexpr int I2C_SCL_PIN = 17;
    i2c_init(i2c, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    // gpio_pull_up(I2C_SDA_PIN);
    // gpio_pull_up(I2C_SCL_PIN);
    i2c_set_slave_mode(i2c, false, 0);

    {
        constexpr int addrSegmentPointer = 0x60 >> 1;
        constexpr int addrEDID = 0xa0 >> 1;
        constexpr int addrDisplayID = 0xa4 >> 1;

        uint8_t buf[128];
        int addr = 0;
        do
        {
            printf("addr: %04x\n", addr);
            uint8_t tmp = addr >> 8;
            i2c_write_blocking(i2c, addrSegmentPointer, &tmp, 1, false);

            tmp = addr & 255;
            i2c_write_blocking(i2c, addrEDID, &tmp, 1, true);
            i2c_read_blocking(i2c, addrEDID, buf, 128, false);

            util::dumpMemory(buf, 128);
            printf("\n");

            addr += 128;
        } while (buf[126]); 
    }
#endif
#if HSTX == 0
    //
    dvi_ = std::make_unique<dvi::DVI>(pio0, &DVICONFIG,
                                      dvi::getTiming640x480p60Hz());
    //    dvi_->setAudioFreq(48000, 25200, 6144);
    dvi_->setAudioFreq(44100, 28000, 6272);
    dvi_->allocateAudioBuffer(256 * 4);
    //    dvi_->setExclusiveProc(&exclProc_);

    dvi_->getBlankSettings().top = 4 * 2;
    dvi_->getBlankSettings().bottom = 4 * 2;
    // dvi_->setScanLine(true);

    applyScreenMode();

    // 空サンプル詰めとく
    dvi_->getAudioRingBuffer().advanceWritePointer(255);
#else
    my_rb_init();
    printf("Free ringbuffer size: %d\n", my_rb_free());
#endif
    memset(framebuffer1, 0x00, sizeof(framebuffer1));
    memset(framebuffer2, 0x00, sizeof(framebuffer2));
    // Initialize the mutex
    mutex_init(&framebuffer_mutex);

    multicore_launch_core1(coreFB_main);
#if HSTX == 1
    init_mcp4822();
#endif
    InfoNES_Main();

    return 0;
}
