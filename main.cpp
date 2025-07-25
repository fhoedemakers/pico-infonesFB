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
#include <util/dump_bin.h>
#include <util/exclusive_proc.h>
#include <util/work_meter.h>
#include <string.h>
#include <stdarg.h>
#include <algorithm>

#include <InfoNES.h>
#include <InfoNES_System.h>
#include <InfoNES_pAPU.h>

#include <dvi/dvi.h>
#include <tusb.h>
#include <gamepad.h>
#include "rom_selector.h"

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

#ifndef DVICONFIG
// #define DVICONFIG dviConfig_PicoDVI
#define DVICONFIG dviConfig_AdafruitMetroRP2350 // dviConfig_PimoroniDemoDVSock
#endif

uint8_t framebuffer1[320 * 240];
uint8_t framebuffer2[320 * 240];
uint8_t *framebufferCore0 = framebuffer1;

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
    extern "C" char contra_nes[];
    static  uintptr_t NES_FILE_ADDR = reinterpret_cast<uintptr_t>(&contra_nes[0]); //0x10080000;

    ROMSelector romSelector_;
    util::ExclusiveProc exclProc_;

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
        }

        dvi_->setScanLine(scanLine);
    }
}

#define CC(x) (((x >> 1) & 15) | (((x >> 6) & 15) << 4) | (((x >> 11) & 15) << 8))
const WORD __not_in_flash_func(NesPalette)[64] = {
    CC(0x39ce), CC(0x1071), CC(0x0015), CC(0x2013), CC(0x440e), CC(0x5402), CC(0x5000), CC(0x3c20),
    CC(0x20a0), CC(0x0100), CC(0x0140), CC(0x00e2), CC(0x0ceb), CC(0x0000), CC(0x0000), CC(0x0000),
    CC(0x5ef7), CC(0x01dd), CC(0x10fd), CC(0x401e), CC(0x5c17), CC(0x700b), CC(0x6ca0), CC(0x6521),
    CC(0x45c0), CC(0x0240), CC(0x02a0), CC(0x0247), CC(0x0211), CC(0x0000), CC(0x0000), CC(0x0000),
    CC(0x7fff), CC(0x1eff), CC(0x2e5f), CC(0x223f), CC(0x79ff), CC(0x7dd6), CC(0x7dcc), CC(0x7e67),
    CC(0x7ae7), CC(0x4342), CC(0x2769), CC(0x2ff3), CC(0x03bb), CC(0x0000), CC(0x0000), CC(0x0000),
    CC(0x7fff), CC(0x579f), CC(0x635f), CC(0x6b3f), CC(0x7f1f), CC(0x7f1b), CC(0x7ef6), CC(0x7f75),
    CC(0x7f94), CC(0x73f4), CC(0x57d7), CC(0x5bf9), CC(0x4ffe), CC(0x0000), CC(0x0000), CC(0x0000)};

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
    if (!SRAMwritten)
    {
        printf("SRAM not updated.\n");
        return;
    }

    // mutex_enter_blocking(&framebuffer_mutex);
    printf("save SRAM\n");
    exclProc_.setProcAndWait([]
                             {
        static_assert((SRAM_SIZE & (FLASH_SECTOR_SIZE - 1)) == 0);
        if (auto addr = getCurrentNVRAMAddr())
        {
            auto ofs = addr - XIP_BASE;
            printf("write flash %x\n", ofs);
            {
                flash_range_erase(ofs, SRAM_SIZE);
                flash_range_program(ofs, SRAM, SRAM_SIZE);
            }
        } });
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
    return dvi_->getAudioRingBuffer().getFullWritableSize();
}

void __not_in_flash_func(InfoNES_SoundOutput)(int samples, BYTE *wave1, BYTE *wave2, BYTE *wave3, BYTE *wave4, BYTE *wave5)
{
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
}

extern WORD PC;

void InfoNES_LoadFrame()
{

    gpio_put(LED_PIN, hw_divider_s32_quotient_inlined(dvi_->getFrameCounter(), 60) & 1);

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
    framebufferCore0 = use_framebuffer1 ? framebuffer1 : framebuffer2;
    mutex_exit(&framebuffer_mutex);
// Wait if core1 is still rendering the framebuffer whe just switched to
#if 1
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
    dvi::DVI::LineBuffer *currentLineBuffer_{};
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
}

WORD buffer[320];
void __not_in_flash_func(coreFB_main)()
{
    uint8_t *framebufferCore1 = framebuffer1;
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

            bool may_render = false;
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

int main()
{
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(10);
    set_sys_clock_khz(CPUFreqKHz, true);

    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    tusb_init();

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

    // Initialize the mutex
    mutex_init(&framebuffer_mutex);

    multicore_launch_core1(coreFB_main);

    memset(framebuffer1, 0x3f, sizeof(framebuffer1));
    memset(framebuffer2, 0x3f, sizeof(framebuffer2));
    InfoNES_Main();

    return 0;
}
