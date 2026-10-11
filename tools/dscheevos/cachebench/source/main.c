// Cache microbenchmark: cycles per load for patterns with known cache behavior.
#include <nds.h>
#include <stdio.h>
#include <string.h>
#define BUS_HZ 33513982u
static void out(const char* s) { char b[128] = "@@"; strcat(b, s); strcat(b, "\n"); nocashMessage(b); iprintf("%s", b + 2); }
static void tstart(void) { TIMER0_CR = 0; TIMER1_CR = 0; TIMER0_DATA = 0; TIMER1_DATA = 0;
    TIMER1_CR = TIMER_ENABLE | TIMER_CASCADE; TIMER0_CR = TIMER_ENABLE | TIMER_DIV_1; }
static u32 ticks(void) { u16 h, l, h2; do { h = TIMER1_DATA; l = TIMER0_DATA; h2 = TIMER1_DATA; } while (h != h2); return ((u32)h << 16) | l; }
static u8 big[65536] __attribute__((aligned(32)));

// ARM-mode loop so loop overhead is small and fixed: one LDR per iteration
__attribute__((target("arm"), noinline)) static u32 walk(volatile u32* p, u32 n, u32 stride_words)
{
    u32 s = 0;
    while (n--) { s += *p; p += stride_words; }
    return s;
}
__attribute__((target("arm"), noinline)) static u32 empty(volatile u32* p, u32 n, u32 stride_words)
{
    u32 s = 0;
    while (n--) { s += (u32)p; p += stride_words; }
    return s;
}
static void bench(const char* name, volatile u32* p, u32 n, u32 stride, int cold)
{
    char line[128];
    if (cold) DC_FlushAll();
    u32 t0 = ticks(); walk(p, n, stride); u32 t = ticks() - t0;
    if (cold) DC_FlushAll();
    u32 e0 = ticks(); empty(p, n, stride); u32 e = ticks() - e0;
    // bus ticks -> ARM9 cycles: x2 at 67 MHz, x4 at 133 MHz
    u32 mul = (isDSiMode() && (REG_SCFG_CLK & 1)) ? 4 : 2;
    u32 x100 = (u32)(((u64)(t - e) * mul * 100) / n);
    siprintf(line, "BENCH %-28s %lu.%02lu cycles/load (over loop)", name, (unsigned long)(x100 / 100), (unsigned long)(x100 % 100));
    out(line);
}
int main(void)
{
    char line[96];
    consoleDemoInit();
    if (isDSiMode()) setCpuClock(false);
    tstart();
    memset(big, 1, sizeof big);
    siprintf(line, "START console=%s cpu=%dMHz", isDSiMode() ? "dsi" : "ds", (isDSiMode() && (REG_SCFG_CLK & 1)) ? 133 : 67);
    out(line);
    volatile u32* cached = (volatile u32*)big;
    volatile u32* uncached = (volatile u32*)memUncached(big);
    for (int warm = 0; warm < 2; warm++) walk(cached, 512, 1);   // 2 KB into cache
    bench("2KB seq, warm (hits)", cached, 512, 1, 0);
    bench("64KB stride32, cold (misses)", cached, 2048, 8, 1);
    bench("64KB seq, cold (1 miss/8)", cached, 16384, 1, 1);
    bench("64KB stride32, uncached", uncached, 2048, 8, 0);
    out("DONE");
    while (1) swiWaitForVBlank();
}
