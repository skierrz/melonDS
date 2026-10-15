#include <array>
// Headless melonDS runner for DScheevos development.
// Boots a .nds in DS or DSi mode, runs frames with no window, passes through
// test output printed via the no$gba debug port, and can stop on a marker.
//
// Test programs print lines starting with "@@" (nocashMessage). The runner
// echoes those, plus emulator warnings/errors, and stops when a line contains
// the --until marker.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include <memory>
#include <optional>
#include <fstream>
#include <filesystem>

#include "NDS.h"
#include "DSi.h"
#include "DSi_NWifi.h"
#include "DSi_I2C.h"
#include "Args.h"
#include "NDSCart.h"
#include "SPI_Firmware.h"
#include "DSi_NAND.h"
#include "FATStorage.h"
#include <ctime>
#include <csignal>
#include <unistd.h>
#include <fcntl.h>
#include "Platform.h"
#include "Net.h"
#include "Net_Slirp.h"
#include "Savestate.h"

using namespace melonDS;
namespace fs = std::filesystem;

// ---- shared state the Platform log hook uses ----
namespace hl
{
bool verbose = false;
std::string until;           // stop marker
bool hit_until = false;
std::string linebuf;         // nocash prints may arrive in pieces
bool stopped = false;
long frame = 0;              // frame being run, for traces
}

// Wi-Fi: frames from the emulated Wi-Fi chip go through a user-mode network
// stack (libslirp). The guest gets DHCP on 10.64.0.0/24; 10.64.0.1 reaches
// the host, so a mock server on the host is at that address.
static melonDS::Net* g_net = nullptr;
int hl_net_send(melonDS::u8* data, int len) { return g_net ? g_net->SendPacket(data, len, 0) : len; }
int hl_net_recv(melonDS::u8* data) { return g_net ? g_net->RecvPacket(data, 0) : 0; }

void hl_signal_stop() { hl::stopped = true; }

// --profile: per VBlank interval, ARM9 cycles inside the DScheevos module
// (its VBlank call and its idle work) and whether the game reached idle (see
// ARMv5::ProfileInstr). Counting starts at the first VBlank after the game's
// first OS_Halt call.
namespace melonDS { extern bool SDTrace; extern long SDCmdBusyCycles; }
namespace prof
{
NDS* nds = nullptr;
u32 base = 0x02D80000;      // DSCM_BASE (dscheevos/module/module.ld)
long stopafter = -1;        // --profile-frames
FILE* log = nullptr;        // --profile-log
FILE* waitlog = nullptr;    // --trace-wait (LAB coverage)
long startframe = -1;       // frame of the game's first OS_Halt call
u64 n = 0, lag = 0, lagrun = 0, maxlagrun = 0, swi5 = 0, swilag = 0, swilagrun = 0, swilagmax = 0;
u64 vbl = 0, vblmax = 0, idle = 0, idlemax = 0, irqmode = 0, busy = 0, before = 0, halted = 0, haltedbios = 0;
bool done = false;
bool cachefromgame = false; // --cache-model-from-game
std::string pcspath;        // --profile-pcs
std::map<u32, ARMv5::ProfPCStat> pcs;
std::map<u32, u64> dmiss;
std::string idlepcspath;    // --profile-idle-pcs
std::string idledmisspath; // --profile-idle-dmiss: module idle-path D misses / uncached accesses per word
std::map<u32, u64> idledmiss;
std::string bootpcspath;    // --profile-boot-pcs
std::vector<ARMv5::ProfPCStat> bootpcs;
bool bootarmed = false;
std::vector<ARMv5::ProfPCStat> idlepcs;
std::string gamepcspath;    // --profile-game-pcs
std::string fppath;         // --profile-footprint: module lines/sets touched per stretch
long fpskip = 0;            // --profile-footprint-skip N: count from the Nth interval on
u32 fphot = 0;              // --profile-footprint-hot ADDR: __hot_start (pass vs light stretches)
u32 fplightlo = 0, fplighthi = 0;  // --profile-footprint-light LO:HI (hex): hot-region code light wakes run
std::map<u32, std::pair<u64, u64>> fplines;
u64 fpintervals = 0;
std::map<u32, ARMv5::ProfGamePC> gamepcs;
std::string arm7path;       // --profile-arm7 (cycles by 256-byte PC block)
bool arm7 = false;          // ARM7 counters (any --profile-arm7*)
u32 arm7lo = 0, arm7hi = 0; // --profile-arm7-range
std::vector<u64> arm7blocks;
std::string hist9path;      // busyloop lab: --hist9 FILE (ARM9 cycles by PC), from interval hist9skip on
long hist9skip = 0;
std::map<u32, std::array<u64, 2>> hist9;
std::vector<ARMv5::Range9> ranges9;
std::vector<ARMv5::Pass9> passes9;    // gsdd lab: --pass9 ENTRY:RET:WORK,...
std::vector<std::pair<u32, u64>> counts9;   // gsdd lab: --count9 PC
std::vector<ARMv5::Sp9> sps9;               // gsdd lab: --sp9 LO:HI
FILE* thr9csv = nullptr;
FILE* irq9csv = nullptr;             // dkr lab: --irq9-csv FILE            // dkr lab: --thr9-csv FILE (per interval, per thread)
bool forcestart = false;             // gsdd lab: --load-state: the profile starts at the first interval   // --range9 LO:HI[:SLO:SHI] (columns in the profile log)
u64 game = 0, gameirq = 0, gameinstr = 0, gimiss = 0, gdmiss = 0, gdwmiss = 0, gwb = 0;
s64 giextra = 0, gdextra = 0;
u64 polli = 0, polld = 0, pollwb = 0, revi = 0, revd = 0, a7busy = 0, a7range = 0, a7rangemax = 0;
std::string rolespath;      // --profile-idle-roles
std::vector<std::string> rolenames { "(no role)", "(halt_entry on the game stack)" };
std::vector<u16> roleof;
std::vector<ARMv5::ProfPCStat> roles;
std::vector<u64> rolecalls;
// "ADDR NAME" per line: a call to the function at ADDR runs as role NAME. The
// first name is the root (module_idle): its call starts a fresh shadow stack.
static void load_roles()
{
    roleof.assign(0x80000 / 2, 0);
    FILE* f = std::fopen(rolespath.c_str(), "r");
    if (!f) { std::fprintf(stderr, "[prof] no roles file %s\n", rolespath.c_str()); return; }
    char name[128]; unsigned addr;
    while (std::fscanf(f, "%x %127s", &addr, name) == 2)
    {
        size_t r = 0;
        while (r < rolenames.size() && rolenames[r] != name) r++;
        if (r == rolenames.size()) rolenames.push_back(name);
        u32 i = ((addr & ~1u) - base) >> 1;
        if (i < roleof.size()) roleof[i] = (u16)r;
    }
    std::fclose(f);
    roles.assign(rolenames.size(), {});
    rolecalls.assign(rolenames.size(), 0);
}

static u32 ram32(u32 addr) { u32 v; std::memcpy(&v, &nds->MainRAM[addr & nds->MainRAMMask & ~3u], 4); return v; }

// The module header (dscheevos/module/src/start.s): magic, version, image
// size, init, VBlank entry (0x10), halt entry (0x14, right after the VBlank
// entry stub). Stacks from module.ld: idle on base+0x200000 down, the VBlank
// call 0x2000 below that, down towards the image.
static void find_module(ARMv5& cpu)
{
    u32 size = ram32(base + 8), vblank = ram32(base + 0x10), halt = ram32(base + 0x14);
    if (ram32(base + 0x3C) == 0x56435344 && size >= 0x28 && size <= 0x80000 && halt >= base && halt < base + size
        && vblank >= base && vblank < halt)
    {
        // the whole 2 MB region is the module's: code it generates at run time lives in its heap
        cpu.ProfLo = base; cpu.ProfHi = base + 0x200000; cpu.ProfHaltEntry = halt;
        cpu.ProfVBlankLo = vblank; cpu.ProfVBlankHi = halt;
        cpu.ProfIrqStackLo = base + size; cpu.ProfIrqStackHi = base + 0x200000 - 0x2000;
        cpu.ProfGateAddr = base;   // LAB coverage: header word 0, the patched SVC_WaitVBlankIntr's target
    }
    else
        cpu.ProfLo = cpu.ProfHi = cpu.ProfHaltEntry = cpu.ProfVBlankLo = cpu.ProfVBlankHi = cpu.ProfIrqStackLo = cpu.ProfIrqStackHi = cpu.ProfGateAddr = 0;
}

static void write_pcs(const std::string& path, const std::vector<ARMv5::ProfPCStat>& v)
{
    FILE* pf = std::fopen(path.c_str(), "w");
    if (!pf) return;
    std::fprintf(pf, "kind,addr,count,cycles,imiss,dreadmiss,dwritemiss\n");
    for (size_t i = 0; i < v.size(); i++)
    {
        auto& e = v[i];
        if (e.Count)
            std::fprintf(pf, "pc,%08x,%llu,%llu,%llu,%llu,%llu\n", base + (u32)i * 2, (unsigned long long)e.Count,
                         (unsigned long long)e.Cycles, (unsigned long long)e.IMiss, (unsigned long long)e.DReadMiss,
                         (unsigned long long)e.DWriteMiss);
    }
    std::fclose(pf);
}

static void on_vblank(ARMv5& cpu)
{
    auto& p = cpu.Prof;
    if (startframe < 0 && forcestart) { p.Halts = std::max(p.Halts, 1u); }
    if (startframe < 0)
    {
        before += p.VBlank + p.Idle;
        if (!bootpcspath.empty() && !bootarmed && cpu.ProfHi > cpu.ProfLo)
        {
            // the module is loaded: its work until the game's first OS_Halt (the set's load) by PC
            bootpcs.assign(0x80000 / 2, {});
            cpu.ProfIdlePCs = &bootpcs; cpu.ProfPrevStats = cpu.CacheStats;
            bootarmed = true;
        }
        if (p.Halts)
        {
            startframe = hl::frame;
            if (bootarmed) { write_pcs(bootpcspath, bootpcs); cpu.ProfIdlePCs = nullptr; }
            if (cachefromgame)
            {
                cpu.CacheModelReset();
                cpu.CacheModel = true;
            }
            if (!pcspath.empty()) { cpu.ProfPCs = &pcs; cpu.ProfDMiss = &dmiss; cpu.ProfPrevStats = cpu.CacheStats; }
            if (!idlepcspath.empty()) { idlepcs.assign(0x80000 / 2, {}); cpu.ProfIdlePCs = &idlepcs; cpu.ProfPrevStats = cpu.CacheStats; }
            if (!idledmisspath.empty()) { if (idlepcs.empty()) idlepcs.assign(0x80000 / 2, {}); cpu.ProfIdlePCs = &idlepcs; cpu.ProfIdleDMiss = &idledmiss; }
            if (!gamepcspath.empty()) cpu.ProfGamePCs = &gamepcs;
            if (arm7)
            {
                auto& a7 = nds->ARM7;
                a7.Prof7On = true; a7.Prof7Lo = arm7lo; a7.Prof7Hi = arm7hi;
                if (!arm7path.empty()) { arm7blocks.assign(0x40000, 0); a7.Prof7Blocks = arm7blocks.data(); }
            }
            if (!rolespath.empty())
            {
                load_roles();
                cpu.ProfRoleOf = &roleof; cpu.ProfRoles = &roles; cpu.ProfRoleCalls = &rolecalls; cpu.ProfRoleDepth = 0;
                cpu.ProfIdleStackLo = base + 0x200000 - 0x2000; cpu.ProfIdleStackHi = base + 0x200000;
                cpu.ProfPrevStats = cpu.CacheStats;
            }
        }
    }
    else if (!done)
    {
        n++;
        if (!fppath.empty())
        {
            if (cpu.FpOn) fpintervals++;
            if (!cpu.FpOn && (long)n > fpskip) { cpu.FpOn = true; cpu.FpHot = fphot; cpu.FpLightLo = fplightlo; cpu.FpLightHi = fplighthi; cpu.FpLines = &fplines; cpu.FpCur.clear(); cpu.FpActive = cpu.FpPass = false; }
        }
        if (!hist9path.empty() && !cpu.Hist9 && (long)n > hist9skip) cpu.Hist9 = &hist9;
        vbl += p.VBlank; vblmax = std::max(vblmax, p.VBlank);
        idle += p.Idle; idlemax = std::max(idlemax, p.Idle);
        irqmode += p.IRQMode;
        busy += p.Busy;
        halted += p.Halted; haltedbios += p.HaltedBios;
        game += p.GameCycles; gameirq += p.GameIRQCycles; gameinstr += p.GameInstr;
        gimiss += p.GameIMiss; gdmiss += p.GameDReadMiss; gdwmiss += p.GameDWriteMiss; gwb += p.GameDWriteBack;
        giextra += p.GameIExtra; gdextra += p.GameDExtra;
        polli += p.PollI; polld += p.PollD; pollwb += p.PollWB; revi += p.RevI; revd += p.RevD;
        auto& a7 = nds->ARM7;
        a7busy += a7.Prof7Busy; a7range += a7.Prof7Range; a7rangemax = std::max(a7rangemax, a7.Prof7Range);
        swi5 += p.Swi5;
        if (p.Swi5 == 0) { swilag++; swilagmax = std::max(swilagmax, ++swilagrun); } else swilagrun = 0;
        bool lagged = p.IdleArrivals == 0;
        if (lagged) { lag++; maxlagrun = std::max(maxlagrun, ++lagrun); }
        else lagrun = 0;
        if (log)
            std::fprintf(log, "%ld,%llu,%llu,%llu,%llu,%u,%d,%08x,%08x,%08x,%llu,%llu", hl::frame, (unsigned long long)p.VBlank, (unsigned long long)p.Idle,
                         (unsigned long long)p.IRQMode, (unsigned long long)p.Busy, p.IdleArrivals, lagged,
                         cpu.R[15], nds->ARM7.R[15], cpu.CPSR, (unsigned long long)p.Halted, (unsigned long long)p.HaltedBios);
        if (log)
            std::fprintf(log, ",%llu,%llu,%llu,%llu,%llu,%llu,%llu,%lld,%lld,%llu,%llu,%llu,%llu,%llu,%llu,%llu",
                         (unsigned long long)p.GameCycles, (unsigned long long)p.GameIRQCycles, (unsigned long long)p.GameInstr,
                         (unsigned long long)p.GameIMiss, (unsigned long long)p.GameDReadMiss, (unsigned long long)p.GameDWriteMiss,
                         (unsigned long long)p.GameDWriteBack, (long long)p.GameIExtra, (long long)p.GameDExtra,
                         (unsigned long long)p.PollI, (unsigned long long)p.PollD, (unsigned long long)p.PollWB,
                         (unsigned long long)p.RevI, (unsigned long long)p.RevD,
                         (unsigned long long)a7.Prof7Busy, (unsigned long long)a7.Prof7Range);
        if (log) std::fprintf(log, ",%lld,%u,%u,%llu,%llu,%llu,%llu", (long long)p.GameExtraIRQ, p.Resumes, p.IRQs, (unsigned long long)p.IRQMax,
                              (unsigned long long)p.Masked, (unsigned long long)p.MaskedPend, (unsigned long long)p.MaskedPendMax);
        if (log)
            for (auto& r : cpu.Ranges9)
                std::fprintf(log, ",%llu,%llu,%d,%d", (unsigned long long)r.Cyc, (unsigned long long)r.Entries,
                             r.First == 0xFFFF ? -1 : (int)r.First, r.Last == 0xFFFF ? -1 : (int)r.Last);
        if (log)
            for (auto& q : cpu.Passes9)
                std::fprintf(log, ",%llu,%llu,%llu,%llu,%llu,%d,%d", (unsigned long long)q.Calls, (unsigned long long)q.Empty,
                             (unsigned long long)q.CycEmpty, (unsigned long long)q.CycWork,
                             (unsigned long long)std::max(q.MaxStretch, q.StretchOn ? q.StretchDone : 0),
                             q.FirstV == 0xFFFF ? -1 : (int)q.FirstV, q.LastV == 0xFFFF ? -1 : (int)q.LastV);
        if (log)
            for (auto& [pc, k] : cpu.Counts9) std::fprintf(log, ",%llu", (unsigned long long)k);
        if (log) std::fprintf(log, ",%llu", (unsigned long long)cpu.StallUsed);
        if (log) for (auto& sp : cpu.Sps9) std::fprintf(log, ",%llu", (unsigned long long)sp.Cyc);
        if (log && cpu.Thr9Info) std::fprintf(log, ",%llu,%llu", (unsigned long long)cpu.Thr9Sw, (unsigned long long)cpu.Thr9SwIrq);
        if (log) std::fprintf(log, "\n");
        if (stopafter >= 0 && (long)n >= stopafter)
        {
            done = true;
            cpu.FpOn = false;
            cpu.Hist9 = nullptr;
            cpu.ProfPCs = nullptr; cpu.ProfDMiss = nullptr; cpu.ProfIdleDMiss = nullptr; cpu.ProfIdlePCs = nullptr; cpu.ProfGamePCs = nullptr; cpu.ProfRoles = nullptr;
            nds->ARM7.Prof7Blocks = nullptr;
        }
    }
    if (waitlog && startframe >= 0 && !done)
    {
        if (!cpu.ProfWaitLog) cpu.ProfWaitLog = waitlog;
        cpu.ProfWaitEvent('V', p.Swi5);
    }
    else if (done) cpu.ProfWaitLog = nullptr;
    if (thr9csv && startframe >= 0 && !done)
    {
        auto rd = [](u32 a) { return *(u32*)&nds->MainRAM[a & nds->MainRAMMask & ~3u]; };
        for (auto& [t, s] : cpu.Thr9Stats)
            if (s.Cyc || s.Ins)
                std::fprintf(thr9csv, "%ld,%08x,%u,%u,%llu,%llu,%llu\n", hl::frame, t, t ? rd(t + 0x6C) : 0, t ? rd(t + 0x70) : 0,
                             (unsigned long long)s.Cyc, (unsigned long long)s.Ins, (unsigned long long)s.IrqIns);
        std::fprintf(thr9csv, "%ld,irq,0,0,%llu,0,0\n", hl::frame, (unsigned long long)cpu.Thr9IrqCyc);
    }
    if (irq9csv && startframe >= 0 && !done)
    {
        std::fprintf(irq9csv, "%ld", hl::frame);
        for (int b = 0; b < 24; b++) std::fprintf(irq9csv, ",%llu", (unsigned long long)cpu.Irq9[b]);
        std::fprintf(irq9csv, "\n");
    }
    for (auto& v : cpu.Irq9) v = 0;
    for (auto& [t, s] : cpu.Thr9Stats) s = {};
    cpu.Thr9IrqCyc = cpu.Thr9Sw = cpu.Thr9SwIrq = 0;
    cpu.ProfFrame = (u32)hl::frame;
    u32 halts = p.Halts;
    p = {};
    p.Halts = halts;
    nds->ARM7.Prof7Busy = nds->ARM7.Prof7Range = 0;
    for (auto& r : cpu.Ranges9) { r.Cyc = r.Entries = 0; r.First = r.Last = 0xFFFF; }
    for (auto& q : cpu.Passes9) { q.Calls = q.Empty = q.CycEmpty = q.CycWork = q.MaxStretch = 0; q.FirstV = q.LastV = 0xFFFF; if (q.StretchOn) { q.StretchRun -= q.StretchDone; q.StretchDone = 0; } }
    for (auto& [pc, k] : cpu.Counts9) k = 0;
    for (auto& sp : cpu.Sps9) sp.Cyc = 0;
    find_module(cpu);
}

static void report(NDS& nds)
{
    if (!hist9path.empty())
    {
        if (FILE* hf = std::fopen(hist9path.c_str(), "w"))
        {
            std::fprintf(hf, "pc,cycles,irq_cycles\n");
            for (auto& [pc, v] : hist9)
                std::fprintf(hf, "%08x,%llu,%llu\n", pc, (unsigned long long)v[0], (unsigned long long)v[1]);
            std::fclose(hf);
        }
    }
    // counted in 133 MHz units; print cycles at the clock in use
    int shift = 2 - nds.ARM9ClockShift;
    double frame = 560190.0 * 4;
    auto cyc = [&](double v) { return v / (1 << shift); };
    auto pct = [&](double v) { return 100.0 * v / frame; };
    std::fprintf(stderr, "[prof] ARM9 at %d MHz, cache model %s\n", nds.ARM9ClockShift == 2 ? 133 : 67, nds.ARM9.CacheModel ? "on" : "off");
    if (nds.ARM9.ProfHaltEntry)
        std::fprintf(stderr, "[prof] module %08x-%08x, VBlank entry %08x, OS_Halt -> %08x\n", nds.ARM9.ProfLo, nds.ARM9.ProfHi,
                     nds.ARM9.ProfVBlankLo, nds.ARM9.ProfHaltEntry);
    else
        std::fprintf(stderr, "[prof] no module, stock OS_Halt\n");
    // MPU at exit: data cache, write buffer (write-back), code cache, for game RAM and the module
    for (u32 a : {0x02000000u, nds.ARM9.ProfLo ? nds.ARM9.ProfLo : base, 0x03700000u, 0x03000000u,
                  (nds.ARM9.ProfLo ? nds.ARM9.ProfLo : base) + 0x0A000000u, 0x02FFF000u})
    {
        u8 m = nds.ARM9.PU_PrivMap[a >> 12];
        std::fprintf(stderr, "[prof] MPU %08x: read %d write %d exec %d, data cache %d, write-back %d, code cache %d\n", a,
                     m & 1, (m >> 1) & 1, (m >> 2) & 1, (m >> 4) & 1, (m >> 5) & 1, (m >> 6) & 1);
    }
    std::fprintf(stderr, "[prof] MPU regions:");
    for (int i = 0; i < 8; i++)
    {
        u32 r = nds.ARM9.PU_Region[i];
        if (r & 1) std::fprintf(stderr, " %d:%08x+%x", i, r & ~0xFFFu, 2u << ((r >> 1) & 0x1F));
    }
    std::fprintf(stderr, " | D-cache bits %02x write-back %02x I-cache %02x\n", nds.ARM9.PU_DataCacheable,
                 nds.ARM9.PU_DataCacheWrite, nds.ARM9.PU_CodeCacheable);
    if (startframe < 0 || !n)
    {
        std::fprintf(stderr, "[prof] the game never called OS_Halt\n");
        return;
    }
    std::fprintf(stderr, "[prof] game from frame %ld: %llu intervals, module before it %.0f cycles\n", startframe, (unsigned long long)n, cyc(before));
    std::fprintf(stderr, "[prof] module VBlank call cycles/interval: avg %.0f (%.2f%%) max %.0f (%.2f%%)\n",
                 cyc((double)vbl / n), pct((double)vbl / n), cyc(vblmax), pct(vblmax));
    std::fprintf(stderr, "[prof] module idle cycles/interval: avg %.0f (%.2f%%) max %.0f (%.2f%%)\n",
                 cyc((double)idle / n), pct((double)idle / n), cyc(idlemax), pct(idlemax));
    std::fprintf(stderr, "[prof] module cycles in CPSR IRQ mode/interval: avg %.0f (%.2f%%)\n",
                 cyc((double)irqmode / n), pct((double)irqmode / n));
    std::fprintf(stderr, "[prof] ARM9 busy: avg %.1f%% of a frame\n", pct((double)busy / n));
    std::fprintf(stderr, "[prof] ARM9 halted: avg %.0f cycles/interval (%.1f%%), %.1f%% of it in BIOS halts\n",
                 cyc((double)halted / n), pct((double)halted / n), halted ? 100.0 * haltedbios / halted : 0.0);
    std::fprintf(stderr, "[prof] swi5: %llu calls; intervals without one %llu (longest run %llu)\n", (unsigned long long)swi5,
                 (unsigned long long)swilag, (unsigned long long)swilagmax);
    std::fprintf(stderr, "[prof] lag: %llu of %llu intervals (%.2f%%), longest run %llu\n", (unsigned long long)lag,
                 (unsigned long long)n, 100.0 * lag / n, (unsigned long long)maxlagrun);
    std::fprintf(stderr, "[game] cycles/interval outside the module: %.0f (%.2f%%), of it IRQ mode %.0f; instructions %.0f (CPI %.3f)\n",
                 cyc((double)(game + gameirq) / n), pct((double)(game + gameirq) / n), cyc((double)gameirq / n), (double)gameinstr / n,
                 gameinstr ? cyc((double)(game + gameirq)) / gameinstr : 0.0);
    if (nds.ARM9.CacheModel)
    {
        std::fprintf(stderr, "[game] cache misses/interval: I %.1f, D read %.1f, D write %.1f, write-backs %.1f\n",
                     (double)gimiss / n, (double)gdmiss / n, (double)gdwmiss / n, (double)gwb / n);
        std::fprintf(stderr, "[game] vs a cache the module never touched: extra cycles/interval I %.0f D %.0f (total %.0f, %.3f%%);"
                     " misses caused/interval I %.1f D %.1f write-backs %.1f (reverse I %.1f D %.1f)\n",
                     cyc((double)giextra / n), cyc((double)gdextra / n), cyc((double)(giextra + gdextra) / n), pct((double)(giextra + gdextra) / n),
                     (double)polli / n, (double)polld / n, (double)pollwb / n, (double)revi / n, (double)revd / n);
    }
    if (arm7)
        std::fprintf(stderr, "[arm7] busy cycles/interval %.0f (%.2f%% of 560190); in %08x-%08x %.0f avg, %llu max\n",
                     (double)a7busy / n, 100.0 * a7busy / n / 560190, arm7lo, arm7hi, (double)a7range / n, (unsigned long long)a7rangemax);
}
}

// --watch-write: bus writes to a main RAM range (the first 40 printed, all counted).
// --watch-skip LO:SIZE: leave out ARM9 writes from code in that range (e.g. the
// module writing its own memory); a summary per writer is printed at the end.
static unsigned long g_watchcount = 0;
static u32 g_watchskiplo = 0, g_watchskipsize = 0;
struct WatchWriter { unsigned long n; u32 lo, hi; long first, last; };
static std::map<std::pair<int, u32>, WatchWriter> g_watchers;
static void watch_write(NDS& nds, int cpu, u32 addr, u32 val, int size)
{
    u32 pc = cpu ? nds.ARM7.R[15] : nds.ARM9.R[15];
    if (!cpu && pc - g_watchskiplo < g_watchskipsize)
        return;
    auto& w = g_watchers[{cpu, pc}];
    if (!w.n) { w.lo = w.hi = addr; w.first = hl::frame; }
    w.n++; w.lo = std::min(w.lo, addr); w.hi = std::max(w.hi, addr); w.last = hl::frame;
    if (melonDS::SDTrace)
        std::fprintf(stderr, "[watch] ARM%d pc=%08x write%d %08x = %0*x ts %llu\n", cpu ? 7 : 9, pc, size * 8, addr, size * 2, val,
                     (unsigned long long)nds.ARM7Timestamp);
    if (g_watchcount++ < 40)
        std::printf("[watch] frame %ld ARM%d pc=%08x lr=%08x write%d %08x = %0*x\n", hl::frame, cpu ? 7 : 9,
                    pc, cpu ? nds.ARM7.R[14] : nds.ARM9.R[14], size * 8, addr, size * 2, val);
}

// --trace-led: every write to BPTWL[30h] (the DSi's Wi-Fi LED)
static void trace_led(u8 reg, u8 val)
{
    if (reg == 0x30)
        std::printf("[led] frame %ld 30h=%02x\n", hl::frame, val);
}

// Called by Platform::Log for every message.
void hl_log(Platform::LogLevel level, const char* msg)
{
    using namespace hl;
    if (level == Platform::LogLevel::Debug)
    {
        // Assemble debug text into lines; only "@@" lines are test output.
        linebuf += msg;
        size_t nl;
        while ((nl = linebuf.find('\n')) != std::string::npos)
        {
            std::string line = linebuf.substr(0, nl);
            linebuf.erase(0, nl + 1);
            if (line.rfind("@@", 0) == 0)
            {
                std::printf("%s\n", line.c_str() + 2);
                std::fflush(stdout);
                if (!until.empty() && line.find(until) != std::string::npos)
                    hit_until = true;
            }
            else if (verbose)
                std::fprintf(stderr, "[dbg] %s\n", line.c_str());
        }
        return;
    }
    if (level == Platform::LogLevel::Info && !verbose)
        return;
    const char* tag = level == Platform::LogLevel::Error ? "err" : level == Platform::LogLevel::Warn ? "warn" : "info";
    std::fprintf(stderr, "[%s] %s", tag, msg);
}

static std::vector<u8> read_file(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) { std::fprintf(stderr, "cannot read %s\n", path.c_str()); std::exit(2); }
    return std::vector<u8>((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

template <size_t N>
static std::unique_ptr<std::array<u8, N>> load_bios(const std::string& path)
{
    auto data = read_file(path);
    if (data.size() != N) { std::fprintf(stderr, "%s: expected %zu bytes, got %zu\n", path.c_str(), N, data.size()); std::exit(2); }
    auto out = std::make_unique<std::array<u8, N>>();
    std::memcpy(out->data(), data.data(), N);
    return out;
}

static void write_ppm(NDS& nds, const std::string& path)
{
    void* top; void* bottom;
    if (!nds.GPU.GetFramebuffers(&top, &bottom)) { std::fprintf(stderr, "no framebuffer\n"); return; }
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return;
    std::fprintf(f, "P6\n256 384\n255\n");
    for (const u32* fb : {(const u32*)top, (const u32*)bottom})
        for (int i = 0; i < 256 * 192; i++)
        {
            u32 px = fb[i];  // soft renderer: 0xAARRGGBB
            u8 rgb[3] = {u8(px >> 16), u8(px >> 8), u8(px)};
            std::fwrite(rgb, 1, 3, f);
        }
    std::fclose(f);
}

// --soft-term: a first SIGTERM/SIGHUP from outside is logged and ignored (an accidental cleanup or
// another session's kill bounces off); a second one within 10 s, SIGINT, or SIGKILL stops the run as
// before. The line names the sender pid (si_pid) so the source can be found later. Test scripts pass
// the flag; without it the default signal behaviour is unchanged. DSC_RUN_LABEL names the run in the
// message; DSC_SOFTTERM_LOG (a file) also gets the line appended.
static const char* g_softLabel = "melonds-headless";
static const char* g_softLog = nullptr;
static void softTermHandler(int sig, siginfo_t* si, void*)
{
    static volatile time_t last = 0;
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    if (last && ts.tv_sec - last <= 10)
    {
        signal(sig, SIG_DFL);
        raise(sig);                      // stop as before
        return;
    }
    last = ts.tv_sec;
    char buf[320];
    time_t now = time(nullptr);
    int n = snprintf(buf, sizeof buf,
        "%s: ignored SIG%s from outside (sender pid %ld, at %ld); send it again within 10 s, or use SIGINT/SIGKILL, to stop\n",
        g_softLabel, sig == SIGTERM ? "TERM" : "HUP", (long)(si ? si->si_pid : 0), (long)now);
    if (n > 0) { (void)!write(2, buf, n < (int)sizeof buf ? n : (int)sizeof buf - 1); }
    if (g_softLog && n > 0)
    {
        int fd = open(g_softLog, O_WRONLY | O_APPEND | O_CREAT, 0600);
        if (fd >= 0) { (void)!write(fd, buf, n < (int)sizeof buf ? n : (int)sizeof buf - 1); close(fd); }
    }
}
static void installSoftTerm()
{
    if (const char* l = getenv("DSC_RUN_LABEL")) if (*l) g_softLabel = l;
    g_softLog = getenv("DSC_SOFTTERM_LOG");
    struct sigaction sa; memset(&sa, 0, sizeof sa);
    sa.sa_sigaction = softTermHandler; sa.sa_flags = SA_SIGINFO | SA_RESTART;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGTERM, &sa, nullptr);
    sigaction(SIGHUP, &sa, nullptr);
}

static void usage()
{
    std::fprintf(stderr,
        "usage: melonds-headless --rom FILE.nds [options]\n"
        "  --mode ds|dsi        console type (default ds)\n"
        "  --bios-dir DIR       dir with biosnds7/9.bin, dsfirmware.bin, biosdsi7/9.bin, dsifirmware.bin, dsinand.bin\n"
        "                       (DS mode without it uses melonDS FreeBIOS + generated firmware)\n"
        "  --nand FILE          NAND image to use (a working copy; it gets modified)\n"
        "  --sd DIR             DSi mode: SD card synced to host DIR (DS mode: DLDI card)\n"
        "  --frames N           max frames to run (default 600)\n"
        "  --until TEXT         stop once a test line contains TEXT\n"
        "  --screenshot FILE    write both screens as PPM at the end\n"
        "  --keys MASK          held keys bitmask, active-low style handled for you (bit0=A ... bit9=L)\n"
        "  --script FILE        timed input: lines \"FRAME KEYMASK HOLDFRAMES\" (# comments);\n"
        "                       \"FRAME WIFIDROP\": the emulated access point drops the DSi;\n"
        "                       \"FRAME WIFIREKEY\": it starts a WPA2 group rekey;\n"
        "                       \"FRAME DUMP\" / \"FRAME SHOT\": RAM dump / screenshot then, to the\n"
        "                       --dump-ram / --screenshot path plus .f<FRAME>.bin / .ppm (a RAM\n"
        "                       dump also writes the ARM9's DTCM to .f<FRAME>.dtcm);\n"
        "                       \"FRAME TOUCH X Y HOLDFRAMES\": touch the lower screen at (X, Y)\n"
        "  --dump-ram FILE      write main RAM (4 or 16 MB) at the end\n"
        "  --firmware-boot      boot through firmware instead of direct boot\n"
        "  --trace-pc FRAME     from this frame on, print the ARM9 PC, LR and SP every 30 frames\n"
        "  --trace-led          DSi mode: print every write to BPTWL[30h] (Wi-Fi LED) with the frame number\n"
        "  --wpa-ap SSID:PMK    DSi mode: a saved WPA2 access point (slot 4) with this PMK (64 hex digits);\n"
        "                       with --net and SSID melonAP, the emulated access point uses WPA2 too\n"
        "  --arg STR            direct boot: pass STR as the next argv entry (first one is argv[0],\n"
        "                       e.g. sd:/_nds/nds-bootstrap-hb-nightly.nds), like a homebrew loader does\n"
        "  --net                connect Wi-Fi to a user-mode network (host reachable at 10.64.0.1)\n"
        "  --cache-model        time ARM9 cache hits/misses instead of melonDS's flat cache cost\n"
        "  --rtc TIME           console clock at boot: \"host\" (default, the host's local time),\n"
        "                       or YYYY-MM-DDTHH:MM:SS\n"
        "  --profile            count ARM9 cycles in the DScheevos module (its VBlank call, its idle work,\n"
        "                       and CPSR IRQ mode) and lag intervals (VBlank to VBlank, the game never\n"
        "                       reached idle); summary at exit\n"
        "  --module-base ADDR   module address for --profile (default 0x02D80000)\n"
        "  --profile-frames N   with --profile: stop N VBlanks after the game's first OS_Halt call\n"
        "  --profile-footprint FILE  with --profile: distinct cache lines/sets module idle code touches per stretch\n"
        "                       (module code between game instructions); per-line counts. --profile-footprint-skip N\n"
        "                       counts from interval N; --profile-footprint-hot HEX (__hot_start) splits light/pass stretches;\n"
        "                       --profile-footprint-light LO:HI (hex) code in the hot region light wakes run\n"
        "  --profile-log FILE   with --profile: CSV per VBlank interval\n"
        "  --trace-wait FILE    with --profile: per event (G gate entry, S SWI 5, R its return, I VBlank IRQ,\n"
        "                       H OS_Halt, V VBlank with the interval's SWI 5 count in addr) with VCOUNT (LAB coverage)\n"
        "  --profile-pcs FILE   with --profile: the VBlank call's cycles and cache misses by PC, and its\n"
        "                       data misses and uncached accesses by address (CSV, from the game's start)\n"
        "  --profile-idle-pcs FILE  with --profile: the module's idle work by PC (count, cycles, cache misses)\n"
        "  --profile-boot-pcs FILE  with --profile: the module's work before the game's first OS_Halt (its load), by PC\n"
        "  --profile-game-pcs FILE  with --profile and the cache model: the game's extra cycles from sharing the\n"
        "                       caches with the module, by fetched code line and by data-access PC (CSV)\n"
        "  --profile-arm7 FILE  with --profile: ARM7 cycles by 256-byte PC block from the game's start (CSV);\n"
        "                       ARM7 busy cycles per interval in the profile log\n"
        "  --profile-arm7-range LO:HI  with --profile: ARM7 cycles per interval with the PC in [LO, HI)\n"
        "  --whatif-module-isets MASK / --whatif-module-dsets MASK  timing study: the module's code fetches /\n"
        "                       data accesses use only cache sets (set & MASK), as if laid out in them\n"
        "  --profile-idle-roles FILE  with --profile: the idle work by role; FILE has \"ADDR NAME\" lines (a call to\n"
        "                       the function at ADDR runs as NAME, callees included unless they have a role;\n"
        "                       the first name is module_idle, the root); writes FILE.csv\n"
        "  --thumb-cycle        ARM9: charge 1 cycle for a Thumb instruction in the upper half of a fetched\n"
        "                       word (stock melonDS charges 0, so Thumb code runs up to twice too fast)\n"
        "  --script-from-game   --script frames count from the game's first OS_Halt call (needs --profile)\n"
        "  --cache-model-from-game  like --cache-model, switched on at the game's first OS_Halt call\n"
        "                       (needs --profile; nds-bootstrap's boot hangs with the cache model on)\n"
        "  --watch-write ADDR:SIZE  DSi mode: print bus writes (CPU or DMA) to [ADDR, ADDR+SIZE) with the\n"
        "                       frame and the CPU's PC (first 40; the count at exit)\n"
        "  --sd-cmd-busy N      DSi SD/MMC: CMD_BUSY reads as set for N ARM7 cycles after a command\n"
        "                       (default: ~96 SD clocks; 0 = never seen, as when an IRQ delays the poll)\n"
        "  --trace-io FILE      CSV per frame (sampled after each frame): ARM9/ARM7 IME/IE/IF, ARM9 TIMER0,\n"
        "                       engine B DISPCNT, BG3CNT, BG0-3 scroll, BLDCNT/BLDALPHA/BLDY, BG mosaic,\n"
        "                       POWCNT1, SOUNDCNT, ARM9 CP15 data access permissions (c5) and CPSR,\n"
        "                       then the --trace-word words\n"
        "  --trace-word ADDR    with --trace-io: also log this main-RAM word (repeatable)\n"
        "  --poke FRAME:CPU:ADDR:VALUE:BITS  before FRAME runs, write VALUE through the ARM9 (CPU 9) or\n"
        "                       ARM7 (CPU 7) bus, 8/16/32 bits (repeatable); e.g. stands in for a game\n"
        "                       that set a write-only register once\n"
        "  --dump-ram-at FRAME:FILE  write main RAM after FRAME (repeatable)\n"
        "  --soft-term          ignore a first SIGTERM/SIGHUP (log it); a second within 10 s, SIGINT or SIGKILL stops\n"
        "  --hist9 FILE         with --profile (busyloop lab): ARM9 cycles by PC, outside / in IRQ mode (CSV),\n"
        "                       from interval --hist9-skip N on (default 0) to --profile-frames\n"
        "  --range9 LO:HI[:SLO:SHI]  with --profile (busyloop lab, repeatable, hex): per interval, non-IRQ cycles\n"
        "                       with the PC in [LO,HI) or, once there, in [SLO,SHI) until it leaves both; entries;\n"
        "                       VCOUNT at the first entry and the last exit (profile log columns)\n"
        "  --pass9 ENTRY:RET:WORK,...  (gsdd lab, hex, with --profile) per interval: calls of the function at\n"
        "                       ENTRY (back at RET on the same thread), empty ones (no WORK pc ran), cycles, longest empty stretch\n"
        "  --count9 PC          (gsdd lab, hex, with --profile) per interval: non-IRQ executions of PC\n"
        "  --access9 LO:SIZE    (gsdd lab, hex) ARM9 data reads/writes in the range by pc, summary at exit\n"
        "  --stall9 PC:UNIT:PERFRAME (gsdd lab) emulate a preemptible gate at PC: burn UNIT ARM9 cycles per arrival,\n"
        "                       at most PERFRAME per frame; --stall9-zero ADDR (hex, repeatable): only while that word is 0\n"
        "                       --stall9-eq A:B (dkr lab, hex, repeatable): only while word A equals word B\n"
        "  --stall9-step N      (dkr lab) --stall9 burns in steps of N cycles (default 64), taking IRQs between steps\n"
        "  --stall9-vbl N       (dkr lab) burn N ARM9 cycles in IRQ mode when a VBlank IRQ is taken, once per frame\n"
        "  --hash9 FILE         (gsdd lab) per frame: screens, SPU output, main RAM (4 MB) hashes; --hash9-skip LO:SIZE\n"
        "  --save-state-at FRAME:FILE / --load-state FILE  (gsdd lab) melonDS savestate (+ FILE.meta)\n"
        "  --thr9 INFO          (dkr lab, hex) SDK thread accounting: current thread = [INFO+4]; prof log thr_sw, thr_sw_irq\n"
        "  --irq9-csv FILE      (dkr lab) per interval: ARM9 IRQ entries by IF&IE bit\n"
        "  --thr9-log FILE      (dkr lab) one line per thread switch: from/to thread, id, prio, state, last pc/lr\n"
        "  --thr9-csv FILE      (dkr lab) per interval and thread: non-IRQ cycles, switches in (from IRQ mode)\n"
        "  --callers9 PC        (dkr lab, hex, repeatable) executions of PC by lr and thread, summary at exit\n"
        "  --ev9 FILE --ev9-pc PC [--ev9-hash LO:LEN]  (dkr lab) one line per execution of PC: times, thread, regs,\n"
        "                       SPU ch1/ch3/capture0 positions, main-RAM range hashes\n"
        "  --ev9-word ADDR / --ev9-hashp PTR:OFF:LEN  (dkr lab, hex) more --ev9 columns: a word; a hash of LEN bytes at [PTR]+OFF\n"
        "  --spu7 FILE          (dkr lab) every ARM7 write to the SPU registers (frame, ARM7 time, addr, size, value, pc)\n"
        "  --shot-prefix P      (busyloop lab) script SHOT lines write P.f<FRAME>.ppm instead of next to --screenshot\n"
        "  --dsc-cache N        (audioid lab) module code/data accesses skip the cache model: 1 = cost 1 cycle, 2 = bus cost\n"
        "  --dsc-ideal N        (audioid lab) module instructions never run past a slice end; a pending IRQ is taken before\n"
        "                       the next module instruction (as from a halt); 2 = also when the module masked IRQs\n"
        "  --dsc-neutral        (audioid lab) after --load-state: the game's OS_Halt back to stock and the ARM7's DScheevos\n"
        "                       VBlank work off (ce7 valueBits bit 28): nds-bootstrap without DScheevos from that point\n"
        "  --dsc-a7free         (audioid lab) the ARM7's DScheevos VBlank work off; the runner bumps the module's VBlank count\n"
        "                       (header +0x24) after each frame instead, costing no emulated time\n"
        "  --verbose            show emulator info/debug logs\n");
    std::exit(2);
}

int main(int argc, char** argv)
{
    std::string shotprefix;  // busyloop lab: --shot-prefix
    std::string rom, mode = "ds", biosdir, nandpath, sddir, shot, scriptpath, dumppath, wpaap, wpassid;
    std::vector<u8> wpapmk;
    long tracepc = -1, sdtracefrom = -1;
    bool traceled = false;
    bool profile = false, scriptfromgame = false, thumbcycle = false;
    u32 whatifi = 0, whatifd = 0;
    std::string proflog, watch;
    std::vector<std::pair<u32, u32>> acc9;                 // gsdd lab: --access9 LO:SIZE
    std::vector<std::pair<long, std::string>> saveat;      // gsdd lab: --save-state-at FRAME:FILE (+ FILE.meta)
    std::string loadstate;                                 // gsdd lab: --load-state FILE (+ FILE.meta)
    int dsccache = 0, dscideal = 0; bool dscneutral = false, dsca7free = false;   // audioid lab
    u32 thr9info = 0; std::vector<u32> call9pcs, ev9pcs; std::vector<std::pair<u32, u32>> ev9hash; std::string thr9path, thr9logpath, ev9path, spu7path; std::vector<u32> ev9words; std::vector<std::tuple<u32, u32, u32>> ev9hashp;   // dkr lab
    std::vector<std::pair<u32, u32>> stalleq;   // dkr lab: --stall9-eq
    unsigned long long stallvbl = 0, stallstep = 64;   // dkr lab: --stall9-vbl, --stall9-step
    u32 stallpc = 0; unsigned long long stallunit = 0, stallpf = 0, stallcheck = 0; std::vector<u32> stallzero;   // gsdd lab: --stall9
    std::string hashpath, audiopath; std::vector<std::pair<u32, u32>> hashskip;                               // gsdd lab: --hash9
    long frames = 600; u32 keys = 0; bool direct = true; bool cachemodel = false; bool net = false;
    std::string rtc = "host";
    std::vector<std::string> guestargs;
    std::string traceio;
    std::vector<u32> tracewords;
    struct Poke { long frame; int cpu; u32 addr, val; int bits; };
    std::vector<Poke> pokes;
    std::vector<std::pair<long, std::string>> dumpsat;
    for (int i = 1; i < argc; i++)
    {
        std::string a = argv[i];
        auto next = [&]() -> std::string { if (i + 1 >= argc) usage(); return argv[++i]; };
        if (a == "--rom") rom = next();
        else if (a == "--mode") mode = next();
        else if (a == "--bios-dir") biosdir = next();
        else if (a == "--nand") nandpath = next();
        else if (a == "--sd") sddir = next();
        else if (a == "--soft-term") installSoftTerm();
        else if (a == "--frames") frames = std::stol(next());
        else if (a == "--until") hl::until = next();
        else if (a == "--screenshot") shot = next();
        else if (a == "--shot-prefix") shotprefix = next();
        else if (a == "--keys") keys = std::stoul(next(), nullptr, 0);
        else if (a == "--script") scriptpath = next();
        else if (a == "--dump-ram") dumppath = next();
        else if (a == "--wpa-ap") wpaap = next();
        else if (a == "--trace-pc") tracepc = std::stol(next());
        else if (a == "--trace-led") traceled = true;
        else if (a == "--firmware-boot") direct = false;
        else if (a == "--arg") guestargs.push_back(next());
        else if (a == "--verbose") hl::verbose = true;
        else if (a == "--cache-model") cachemodel = true;
        else if (a == "--rtc") rtc = next();
        else if (a == "--net") net = true;
        else if (a == "--watch-write") watch = next();
        else if (a == "--watch-skip") { std::string v = next(); size_t c = v.find(':'); g_watchskiplo = std::stoul(v.substr(0, c), nullptr, 0); g_watchskipsize = std::stoul(v.substr(c + 1), nullptr, 0); }
        else if (a == "--profile") profile = true;
        else if (a == "--module-base") prof::base = std::stoul(next(), nullptr, 0);
        else if (a == "--profile-frames") prof::stopafter = std::stol(next());
        else if (a == "--profile-log") proflog = next();
        else if (a == "--trace-wait") { std::string f = next(); prof::waitlog = std::fopen(f.c_str(), "w");
            if (!prof::waitlog) { std::fprintf(stderr, "cannot write %s\n", f.c_str()); return 2; }
            std::fprintf(prof::waitlog, "frame,kind,vcount,addr,game_instr,game_cycles,module_idle,flag3ff8,if,ie\n"); }
        else if (a == "--script-from-game") scriptfromgame = true;
        else if (a == "--cache-model-from-game") prof::cachefromgame = true;
        else if (a == "--profile-pcs") prof::pcspath = next();
        else if (a == "--profile-idle-pcs") prof::idlepcspath = next();
        else if (a == "--profile-idle-dmiss") prof::idledmisspath = next();
        else if (a == "--profile-boot-pcs") prof::bootpcspath = next();
        else if (a == "--profile-game-pcs") prof::gamepcspath = next();
        else if (a == "--profile-footprint") prof::fppath = next();
        else if (a == "--profile-footprint-skip") prof::fpskip = std::stol(next());
        else if (a == "--profile-footprint-hot") prof::fphot = (u32)std::stoul(next(), nullptr, 16);
        else if (a == "--profile-footprint-light") { std::string v = next(); size_t c = v.find(':');
            prof::fplightlo = (u32)std::stoul(v.substr(0, c), nullptr, 16); prof::fplighthi = (u32)std::stoul(v.substr(c + 1), nullptr, 16); }
        else if (a == "--whatif-module-isets") whatifi = std::stoul(next(), nullptr, 0);
        else if (a == "--whatif-module-dsets") whatifd = std::stoul(next(), nullptr, 0);
        else if (a == "--profile-arm7") { prof::arm7 = true; prof::arm7path = next(); }
        else if (a == "--profile-arm7-range") { std::string v = next(); size_t c = v.find(':'); prof::arm7 = true;
            prof::arm7lo = std::stoul(v.substr(0, c), nullptr, 0); prof::arm7hi = std::stoul(v.substr(c + 1), nullptr, 0); }
        else if (a == "--profile-idle-roles") prof::rolespath = next();
        else if (a == "--hist9") prof::hist9path = next();
        else if (a == "--hist9-skip") prof::hist9skip = std::stol(next());
        else if (a == "--range9")
        {
            std::string v = next(); unsigned lo = 0, hi = 0, slo = 0, shi = 0;
            int k = std::sscanf(v.c_str(), "%x:%x:%x:%x", &lo, &hi, &slo, &shi);
            if (k != 2 && k != 4) usage();
            prof::ranges9.push_back({ lo, hi, slo, shi, false, 0, 0, 0xFFFF, 0xFFFF });
        }
        else if (a == "--pass9")
        {
            std::string v = next(); unsigned lo = 0, ret = 0; int used = 0;
            if (std::sscanf(v.c_str(), "%x:%x%n", &lo, &ret, &used) != 2) usage();
            ARMv5::Pass9 q{}; q.Entry = lo; q.Ret = ret;
            const char* w = v.c_str() + used;
            while (*w == ':' || *w == ',') { unsigned x; int u = 0; if (std::sscanf(w + 1, "%x%n", &x, &u) != 1) break; q.Work.push_back(x); w += 1 + u; }
            prof::passes9.push_back(q);
        }
        else if (a == "--sp9") { std::string v = next(); unsigned lo = 0, hi = 0; if (std::sscanf(v.c_str(), "%x:%x", &lo, &hi) != 2) usage(); prof::sps9.push_back({ lo, hi, 0 }); }
        else if (a == "--count9") prof::counts9.push_back({ (u32)std::stoul(next(), nullptr, 16), 0 });
        else if (a == "--access9")
        {
            std::string v = next(); unsigned lo = 0, n = 0;
            if (std::sscanf(v.c_str(), "%x:%x", &lo, &n) != 2) usage();
            acc9.push_back({ lo, n });
        }
        else if (a == "--save-state-at")
        {
            std::string v = next(); size_t c = v.find(':');
            if (c == std::string::npos) usage();
            saveat.push_back({std::stol(v.substr(0, c)), v.substr(c + 1)});
        }
        else if (a == "--load-state") loadstate = next();
        else if (a == "--dsc-cache") dsccache = std::stoi(next());
        else if (a == "--dsc-ideal") dscideal = std::stoi(next());
        else if (a == "--dsc-neutral") dscneutral = true;
        else if (a == "--dsc-a7free") dsca7free = true;
        else if (a == "--stall9")
        {
            std::string v = next(); unsigned pc = 0; unsigned long long u = 0, pf = 0;
            if (std::sscanf(v.c_str(), "%x:%llu:%llu", &pc, &u, &pf) != 3) usage();
            stallpc = pc; stallunit = u; stallpf = pf;
        }
        else if (a == "--stall9-check") stallcheck = std::stoull(next());
        else if (a == "--thr9") thr9info = std::stoul(next(), nullptr, 16);
        else if (a == "--thr9-csv") thr9path = next();
        else if (a == "--thr9-log") thr9logpath = next();
        else if (a == "--irq9-csv") { prof::irq9csv = std::fopen(next().c_str(), "w"); if (prof::irq9csv) std::fprintf(prof::irq9csv, "frame,vblank,hblank,vcount,t0,t1,t2,t3,sio,dma0,dma1,dma2,dma3,keypad,slot2,b14,b15,ipcsync,fifo_send,fifo_recv,card_done,card_ire,geom,b22,b23\n"); }
        else if (a == "--callers9") call9pcs.push_back(std::stoul(next(), nullptr, 16));
        else if (a == "--ev9") ev9path = next();
        else if (a == "--ev9-pc") ev9pcs.push_back(std::stoul(next(), nullptr, 16));
        else if (a == "--ev9-hash") { std::string v = next(); unsigned lo = 0, n = 0; if (std::sscanf(v.c_str(), "%x:%x", &lo, &n) != 2) usage(); ev9hash.push_back({lo, n}); }
        else if (a == "--spu7") spu7path = next();
        else if (a == "--ev9-word") ev9words.push_back(std::stoul(next(), nullptr, 16));
        else if (a == "--ev9-hashp") { std::string v = next(); unsigned pa = 0, off = 0, n = 0; if (std::sscanf(v.c_str(), "%x:%x:%x", &pa, &off, &n) != 3) usage(); ev9hashp.push_back({pa, off, n}); }
        else if (a == "--stall9-zero") stallzero.push_back(std::stoul(next(), nullptr, 16));
        else if (a == "--stall9-step") stallstep = std::stoull(next());
        else if (a == "--stall9-vbl") stallvbl = std::stoull(next());
        else if (a == "--stall9-eq") { std::string v = next(); unsigned x = 0, y = 0; if (std::sscanf(v.c_str(), "%x:%x", &x, &y) != 2) usage(); stalleq.push_back({x, y}); }
        else if (a == "--hash9") hashpath = next();
        else if (a == "--audio-out") audiopath = next();
        else if (a == "--hash9-skip") { std::string v = next(); unsigned lo = 0, n = 0; if (std::sscanf(v.c_str(), "%x:%x", &lo, &n) != 2) usage(); hashskip.push_back({lo, n}); }
        else if (a == "--thumb-cycle") thumbcycle = true;
        else if (a == "--trace-sd") sdtracefrom = std::stol(next());
        else if (a == "--sd-cmd-busy") melonDS::SDCmdBusyCycles = std::stol(next());
        else if (a == "--trace-io") traceio = next();
        else if (a == "--trace-word") tracewords.push_back(std::stoul(next(), nullptr, 0));
        else if (a == "--poke")
        {
            Poke p{};
            if (std::sscanf(next().c_str(), "%ld:%d:%i:%i:%d", &p.frame, &p.cpu, (int*)&p.addr, (int*)&p.val, &p.bits) != 5) usage();
            pokes.push_back(p);
        }
        else if (a == "--dump-ram-at")
        {
            std::string v = next(); size_t c = v.find(':');
            if (c == std::string::npos) usage();
            dumpsat.push_back({std::stol(v.substr(0, c)), v.substr(c + 1)});
        }
        else usage();
    }
    if (rom.empty() || (mode != "ds" && mode != "dsi")) usage();
    bool dsi = mode == "dsi";
    if (dsi && biosdir.empty()) { std::fprintf(stderr, "DSi mode needs --bios-dir\n"); return 2; }

    NDSArgs ndsargs;
    ndsargs.JIT = std::nullopt;        // interpreter: timing-accurate path
    ndsargs.GDB = std::nullopt;
    if (!biosdir.empty())
    {
        ndsargs.ARM9BIOS = load_bios<ARM9BIOSSize>(biosdir + "/biosnds9.bin");
        ndsargs.ARM7BIOS = load_bios<ARM7BIOSSize>(biosdir + "/biosnds7.bin");
        std::string fw = biosdir + (dsi ? "/dsifirmware.bin" : "/dsfirmware.bin");
        Platform::FileHandle* fh = Platform::OpenFile(fw, Platform::Read);
        if (!fh) { std::fprintf(stderr, "cannot open %s\n", fw.c_str()); return 2; }
        ndsargs.Firmware = Firmware(fh);
        Platform::CloseFile(fh);
        if (net)
        {
            // Point the saved Wi-Fi settings at melonDS's emulated access point
            // (in memory only; the dump on disk is untouched).
            auto& aps = ndsargs.Firmware.GetAccessPoints();
            aps[0] = Firmware::WifiAccessPoint(dsi ? 1 : 0);
            aps[1] = Firmware::WifiAccessPoint();
            aps[2] = Firmware::WifiAccessPoint();
            for (auto& ap : ndsargs.Firmware.GetExtendedAccessPoints()) ap = Firmware::ExtendedWifiAccessPoint();
        }
        size_t colon = wpaap.find(':');
        if (dsi && colon != std::string::npos && wpaap.size() - colon - 1 == 64)
        {
            wpassid = wpaap.substr(0, colon);
            for (int i = 0; i < 32; i++)
                wpapmk.push_back((u8)std::stoi(wpaap.substr(colon + 1 + i * 2, 2), nullptr, 16));
            if (net && wpassid == "melonAP")
                ndsargs.Firmware.GetAccessPoints()[0] = Firmware::WifiAccessPoint(); // only the WPA2 slot
            // In memory only, like the access points above.
            auto& ap = ndsargs.Firmware.GetExtendedAccessPoints()[0];
            ap = Firmware::ExtendedWifiAccessPoint();
            std::string ssid = wpaap.substr(0, std::min<size_t>(colon, 32));
            memcpy(&ap.Bytes[0x40], ssid.data(), ssid.size());
            ap.Bytes[0xE7] = 0x10;              // WPA/WPA2
            ap.Bytes[0xE8] = (u8)ssid.size();
            ap.Bytes[0xEF] = 1;                 // slot in use
            for (int i = 0; i < 32; i++)
                ap.Bytes[0x100 + i] = (u8)std::stoi(wpaap.substr(colon + 1 + i * 2, 2), nullptr, 16);
            ap.Bytes[0x181] = 7;                // WPA2-AES
            ap.UpdateChecksum();
        }
    }
    else
        ndsargs.Firmware = Firmware(0);

    std::unique_ptr<NDS> nds;
    if (dsi)
    {
        DSiArgs dargs {std::move(ndsargs)};
        dargs.ARM9iBIOS = load_bios<DSiBIOSSize>(biosdir + "/biosdsi9.bin");
        dargs.ARM7iBIOS = load_bios<DSiBIOSSize>(biosdir + "/biosdsi7.bin");
        if (nandpath.empty()) { std::fprintf(stderr, "DSi mode needs --nand (a working copy)\n"); return 2; }
        Platform::FileHandle* nf = Platform::OpenFile(nandpath, Platform::ReadWriteExisting);
        if (!nf) { std::fprintf(stderr, "cannot open NAND %s\n", nandpath.c_str()); return 2; }
        DSi_NAND::NANDImage nand(nf, &(*dargs.ARM7iBIOS)[0x8308]);
        if (!nand) { std::fprintf(stderr, "NAND did not parse\n"); return 2; }
        dargs.NANDImage = std::move(nand);
        if (!sddir.empty())
        {
            fs::create_directories(sddir);
            dargs.DSiSDCard = FATStorage(FATStorageArgs{sddir + ".img", 1ull << 30, false, sddir} /* 1 GB: melonDS formats FAT32 only from 1 GB up */);
        }
        nds = std::make_unique<DSi>(std::move(dargs));
    }
    else
        nds = std::make_unique<NDS>(std::move(ndsargs));

    auto romdata = read_file(rom);
    std::optional<NDSCart::NDSCartArgs> cartargs;
    if (!dsi && !sddir.empty())
    {
        fs::create_directories(sddir);
        cartargs = NDSCart::NDSCartArgs{.SDCard = FATStorageArgs{sddir + ".img", 1ull << 30, false, sddir} /* 1 GB: melonDS formats FAT32 only from 1 GB up */};
    }
    auto cart = NDSCart::ParseROM(romdata.data(), (u32)romdata.size(), nullptr, std::move(cartargs));
    if (!cart) { std::fprintf(stderr, "ROM did not parse\n"); return 2; }

    nds->Reset();
    {
        // melonDS starts the clock at 2000-01-01; games and DScheevos' unlock times want the real date
        int y, mo, d, h, mi, sec;
        if (rtc == "host")
        {
            std::time_t now = std::time(nullptr);
            std::tm lt = *std::localtime(&now);
            y = lt.tm_year + 1900; mo = lt.tm_mon + 1; d = lt.tm_mday; h = lt.tm_hour; mi = lt.tm_min; sec = lt.tm_sec;
        }
        else if (std::sscanf(rtc.c_str(), "%d-%d-%dT%d:%d:%d", &y, &mo, &d, &h, &mi, &sec) != 6)
        {
            std::fprintf(stderr, "bad --rtc %s\n", rtc.c_str());
            return 2;
        }
        nds->RTC.SetDateTime(y, mo, d, h, mi, sec);
    }
    nds->SetNDSCart(std::move(cart));
    if (direct || nds->NeedsDirectBoot())
        nds->SetupDirectBoot(fs::path(rom).filename().string());
    if (!guestargs.empty())
    {
        // Pass argv the way devkitPro's homebrew bootloader (and TWiLight Menu) does:
        // NUL-separated command line placed word-aligned after the ARM9 (or ARM9i,
        // if loaded higher) binary, described by the libnds argv struct at
        // 0x02FFFE70. crt0 copies it to the heap before clearing BSS.
        const NDSHeader& hdr = nds->NDSCartSlot.GetCart()->GetHeader();
        u32 dst = (hdr.ARM9RAMAddress + hdr.ARM9Size + 3) & ~3u;
        if (dsi && (hdr.UnitCode & 0x02) && hdr.DSiARM9iSize)
            dst = std::max(dst, (hdr.DSiARM9iRAMAddress + hdr.DSiARM9iSize + 3) & ~3u);
        std::string cmdline;
        for (auto& s : guestargs) { cmdline += s; cmdline += '\0'; }
        for (size_t i = 0; i < cmdline.size(); i++) nds->ARM9Write8(dst + i, (u8)cmdline[i]);
        nds->ARM9Write32(0x02FFFE70, 0x5F617267);           // '_arg'
        nds->ARM9Write32(0x02FFFE74, dst);                  // command line
        nds->ARM9Write32(0x02FFFE78, (u32)cmdline.size());  // length
    }
    nds->ARM9.CacheModel = cachemodel;
    nds->ARM9.WhatIfISets = whatifi; nds->ARM9.WhatIfDSets = whatifd;
    if (thumbcycle)
        nds->ARM9.ThumbOddCycle = 1;
    if (profile)
    {
        prof::nds = nds.get();
        nds->ARM9.ProfOn = true;
        nds->ARM9.ProfVBlank = prof::on_vblank;
        nds->ARM9.Ranges9 = prof::ranges9;
        for (auto& q : prof::passes9) { q.FirstV = q.LastV = 0xFFFF; }
        nds->ARM9.Passes9 = prof::passes9;
        nds->ARM9.Counts9 = prof::counts9;
        nds->ARM9.Sps9 = prof::sps9;
        if (!proflog.empty())
        {
            prof::log = std::fopen(proflog.c_str(), "w");
            if (!prof::log) { std::fprintf(stderr, "cannot write %s\n", proflog.c_str()); return 2; }
            std::fprintf(prof::log, "frame,module_vblank,module_idle,module_irq_mode,busy,idle_arrivals,lag,arm9_pc,arm7_pc,arm9_cpsr,halted,halted_bios,"
                         "game,game_irq,game_instr,game_imiss,game_dreadmiss,game_dwritemiss,game_writeback,game_iextra,game_dextra,"
                         "poll_i,poll_d,poll_wb,rev_i,rev_d,arm7_busy,arm7_range,game_extra_irq,resumes,irqs,irq_max,"
                         "masked,masked_pend,masked_pend_max");
            for (size_t k = 0; k < prof::ranges9.size(); k++)
                std::fprintf(prof::log, ",r%zu_cyc,r%zu_entries,r%zu_first_line,r%zu_last_line", k, k, k, k);
            for (size_t k = 0; k < prof::passes9.size(); k++)
                std::fprintf(prof::log, ",p%zu_calls,p%zu_empty,p%zu_cyc_empty,p%zu_cyc_work,p%zu_max_stretch,p%zu_first_line,p%zu_last_line",
                             k, k, k, k, k, k, k);
            for (auto& [pc, k] : prof::counts9) std::fprintf(prof::log, ",c%x", pc);
            std::fprintf(prof::log, ",stall_cyc");
            for (auto& sp : prof::sps9) std::fprintf(prof::log, ",sp%x_cyc", sp.Lo);
            if (thr9info) std::fprintf(prof::log, ",thr_sw,thr_sw_irq");
            std::fprintf(prof::log, "\n");
        }
    }
    if ((scriptfromgame || prof::cachefromgame) && !profile) usage();

    melonDS::Net netstack;
    if (net)
    {
        netstack.RegisterInstance(0);
        netstack.SetDriver(std::make_unique<Net_Slirp>([&](const u8* d, int len) { netstack.RXEnqueue(d, len); }));
        g_net = &netstack;
    }
    nds->Start();

    // melonDS key mask: bit set = released
    nds->SetKeyMask(~keys & 0xFFF);

    struct Press { long frame; u32 keys; long hold; bool wifidrop; bool wifirekey; bool dump; bool shot; bool touch; int x, y; };
    std::vector<Press> script;
    if (!scriptpath.empty())
    {
        FILE* sf = std::fopen(scriptpath.c_str(), "r");
        if (!sf) { std::fprintf(stderr, "cannot open %s\n", scriptpath.c_str()); return 2; }
        char line[256];
        while (std::fgets(line, sizeof line, sf))
        {
            Press p{};
            char word[16] = {};
            if (line[0] == '#') continue;
            int nw = std::sscanf(line, "%ld %15s", &p.frame, word);
            if (nw == 2 && !std::strcmp(word, "WIFIDROP"))
                p.wifidrop = true;
            else if (nw == 2 && !std::strcmp(word, "DUMP"))
                p.dump = true;
            else if (nw == 2 && !std::strcmp(word, "SHOT"))
                p.shot = true;
            else if (nw == 2 && !std::strcmp(word, "TOUCH"))
            {
                if (std::sscanf(line, "%ld %15s %d %d %ld", &p.frame, word, &p.x, &p.y, &p.hold) != 5) continue;
                p.touch = true;
            }
            else if (!std::strcmp(word, "WIFIREKEY"))
                p.wifirekey = true;
            else if (std::sscanf(line, "%ld %i %ld", &p.frame, (int*)&p.keys, &p.hold) != 3) continue;
            script.push_back(p);
        }
        std::fclose(sf);
    }

    DSi_NWifi* nwifi = dsi ? dynamic_cast<DSi_NWifi*>(static_cast<DSi*>(nds.get())->SDIO.Port(0)) : nullptr;
    if (nwifi && net && wpapmk.size() == 32 && wpassid == "melonAP")
        nwifi->EnableWPA2(wpapmk.data());
    if (dsi && !watch.empty())
    {
        size_t colon = watch.find(':');
        if (colon == std::string::npos) usage();
        nds->WatchLo = std::stoul(watch.substr(0, colon), nullptr, 0);
        nds->WatchSize = std::stoul(watch.substr(colon + 1), nullptr, 0);
        nds->WatchHook = watch_write;
    }
    if (dsi && traceled)
        static_cast<DSi*>(nds.get())->I2C.GetBPTWL()->WriteHook = trace_led;

    FILE* tio = nullptr;
    if (!traceio.empty())
    {
        tio = std::fopen(traceio.c_str(), "w");
        if (!tio) { std::fprintf(stderr, "cannot open %s\n", traceio.c_str()); return 2; }
        std::fprintf(tio, "frame,ime9,ie9,if9,ime7,ie7,if7,tm0cnt9,tm0val9,dispcnt_b,bg3cnt_b,bg0hofs_b,bg0vofs_b,bg1hofs_b,bg1vofs_b,"
                          "bg2hofs_b,bg2vofs_b,bg3hofs_b,bg3vofs_b,bldcnt_b,bldalpha_b,bldy_b,mosaic_b,powcnt1,soundcnt,dperm9,cpsr9");
        for (u32 w : tracewords) std::fprintf(tio, ",w%08x", w);
        std::fprintf(tio, "\n");
    }

    long f = 0, scriptbase = -1;
    nds->ARM9.Irq9On = prof::irq9csv != nullptr;
    nds->ARM9.Thr9Info = thr9info; nds->ARM9.Call9PCs = call9pcs; nds->ARM9.Ev9PCs = ev9pcs; nds->ARM9.Ev9Hash = ev9hash; nds->ARM9.Ev9Words = ev9words; nds->ARM9.Ev9HashP = ev9hashp;
    if (!thr9path.empty()) prof::thr9csv = std::fopen(thr9path.c_str(), "w");
    if (prof::thr9csv) std::fprintf(prof::thr9csv, "frame,thread,id,prio,cyc,switch_in,switch_in_irq\n");
    if (!thr9logpath.empty()) { nds->ARM9.Thr9Log = std::fopen(thr9logpath.c_str(), "w"); if (nds->ARM9.Thr9Log) std::fprintf(nds->ARM9.Thr9Log, "frame,ts9,irq,vcount,from,from_id,from_prio,from_state,from_last_pc,from_last_lr,to,to_id,to_prio,to_state,pc\n"); }
    if (!ev9path.empty()) nds->ARM9.Ev9Log = std::fopen(ev9path.c_str(), "w");
    if (!spu7path.empty()) nds->Spu7Log = std::fopen(spu7path.c_str(), "w");
    nds->ARM9.StallPC = stallpc; nds->ARM9.StallUnit = stallunit; nds->ARM9.StallPerFrame = stallpf; nds->ARM9.StallZero = stallzero; nds->ARM9.StallEq = stalleq; nds->ARM9.StallVbl = stallvbl; nds->ARM9.StallStep = stallstep ? stallstep : 1; nds->ARM9.StallCheck = stallcheck;
    FILE* hashf = hashpath.empty() ? nullptr : std::fopen(hashpath.c_str(), "w");
    FILE* audiof = audiopath.empty() ? nullptr : std::fopen(audiopath.c_str(), "wb");
    std::vector<s16> abuf(1 << 16);
    std::vector<u8> ramcopy;
    if (!acc9.empty())
    {
        u32 lo = 0xFFFFFFFF, hi = 0;
        for (auto [a, n] : acc9) { lo = std::min(lo, a); hi = std::max(hi, a + n); }
        nds->ARM9.Acc9Ranges = acc9; nds->ARM9.Acc9Lo = lo; nds->ARM9.Acc9Span = hi - lo;
    }
    if (!loadstate.empty())
    {
        FILE* sf = std::fopen(loadstate.c_str(), "rb");
        FILE* mf = std::fopen((loadstate + ".meta").c_str(), "r");
        long f0 = -1, sb = -1;
        if (!sf || !mf || std::fscanf(mf, "%ld %ld", &f0, &sb) != 2) { std::fprintf(stderr, "cannot read state %s(.meta)\n", loadstate.c_str()); return 2; }
        std::fclose(mf);
        std::fseek(sf, 0, SEEK_END); long len = std::ftell(sf); std::fseek(sf, 0, SEEK_SET);
        std::vector<u8> buf(len);
        if (std::fread(buf.data(), 1, len, sf) != (size_t)len) { std::fprintf(stderr, "short state %s\n", loadstate.c_str()); return 2; }
        std::fclose(sf);
        Savestate st(buf.data(), (u32)len, false);
        if (!nds->DoSavestate(&st) || st.Error) { std::fprintf(stderr, "state load failed %s\n", loadstate.c_str()); return 2; }
        f = f0 + 1; scriptbase = sb; frames += f; prof::forcestart = true;
        std::fprintf(stderr, "[runner] state loaded %s: frame %ld, script base %ld\n", loadstate.c_str(), f0, sb);
    }
    nds->ARM9.DscCache = dsccache; nds->ARM9.DscIdeal = dscideal;
    if (dscneutral || dsca7free)
    {
        // audioid lab: switch DScheevos off on the ARM7 (and with --dsc-neutral on the ARM9 too), in place
        int ce7n = 0;
        for (u32 c : {0x037E0400u, 0x037F0000u, 0x03020400u, 0x03030000u})
        {
            if (nds->ARM7Read32(c) != c) continue;
            u32 vb = nds->ARM7Read32(c + 0x4C);
            std::fprintf(stderr, "[dsc] ce7 at %08x, valueBits %08x -> %08x\n", c, vb, vb & ~(1u << 28));
            nds->ARM7Write32(c + 0x4C, vb & ~(1u << 28)); ce7n++;
        }
        if (!ce7n) std::fprintf(stderr, "[dsc] WARNING: no ce7 header found\n");
        if (dscneutral)
        {
            u32 halt = prof::ram32(prof::base + 0x14); int n = 0;
            for (u32 a = 0x02000000; a < 0x02400000; a += 4)
                if (prof::ram32(a) == 0xE51FF004 && prof::ram32(a + 4) == halt && prof::ram32(a + 8) == 0xE12FFF1E)
                {
                    u32 o = a & nds->MainRAMMask; u32 w0 = 0xE3A00000, w1 = 0xEE070F90;
                    std::memcpy(&nds->MainRAM[o], &w0, 4); std::memcpy(&nds->MainRAM[o + 4], &w1, 4); n++;
                    std::fprintf(stderr, "[dsc] OS_Halt at %08x restored to stock\n", a);
                }
            if (!n) std::fprintf(stderr, "[dsc] WARNING: no patched OS_Halt found (halt entry %08x)\n", halt);
        }
    }
    for (; f < frames && !hl::hit_until && !hl::stopped && !prof::done; f++)
    {
        for (auto& p : pokes)
        {
            if (p.frame != f) continue;
            if (p.cpu == 7)
            {
                if (p.bits == 8) nds->ARM7Write8(p.addr, p.val); else if (p.bits == 16) nds->ARM7Write16(p.addr, p.val); else nds->ARM7Write32(p.addr, p.val);
            }
            else
            {
                if (p.bits == 8) nds->ARM9Write8(p.addr, p.val); else if (p.bits == 16) nds->ARM9Write16(p.addr, p.val); else nds->ARM9Write32(p.addr, p.val);
            }
            std::printf("[poke] frame %ld ARM%d %08x = %x (%d bits)\n", f, p.cpu == 7 ? 7 : 9, p.addr, p.val, p.bits);
        }
        u32 held = keys;
        int touchx = -1, touchy = -1;
        // script time: frames since boot, or since the game's first OS_Halt call
        long st = !scriptfromgame ? f : scriptbase >= 0 ? f - scriptbase : prof::startframe >= 0 ? f - prof::startframe : -1;
        for (auto& p : script)
        {
            if (st < 0) break;
            if (p.dump || p.shot)
            {
                // RAM dump / screenshot of what the previous frame left, next to
                // --dump-ram / --screenshot, named by script frame: FILE.f<frame>
                const std::string& base = p.dump ? dumppath : (shotprefix.empty() ? shot : shotprefix);
                if (st == p.frame && !base.empty())
                {
                    std::string out = base + ".f" + std::to_string(p.frame);
                    if (p.shot) write_ppm(*nds, out + ".ppm");
                    else
                    {
                        // main RAM (4 or 16 MB), and the ARM9's DTCM next to it
                        if (FILE* df = std::fopen((out + ".bin").c_str(), "wb"))
                        {
                            std::fwrite(nds->MainRAM, 1, nds->MainRAMMask + 1, df);
                            std::fclose(df);
                        }
                        if (FILE* df = std::fopen((out + ".dtcm").c_str(), "wb"))
                        {
                            std::fwrite(nds->ARM9.DTCM, 1, DTCMPhysicalSize, df);
                            std::fclose(df);
                        }
                    }
                    std::fprintf(stderr, "[runner] frame %ld (script %ld): %s %s\n", f, st, p.dump ? "RAM dump" : "screenshot", out.c_str());
                }
                continue;
            }
            if (p.touch)
            {
                if (st >= p.frame && st < p.frame + p.hold) { touchx = p.x; touchy = p.y; }
                continue;
            }
            if (!p.wifidrop && !p.wifirekey && st >= p.frame && st < p.frame + p.hold) held |= p.keys;
            if ((p.wifidrop || p.wifirekey) && st == p.frame && nwifi)
            {
                if (p.wifidrop) nwifi->DropConnection();
                else nwifi->StartGroupRekey();
            }
        }
        nds->SetKeyMask(~held & 0xFFF);
        if (touchx >= 0) nds->TouchScreen((u16)touchx, (u16)touchy);
        else nds->ReleaseScreen();
        hl::frame = f;
        if (sdtracefrom >= 0 && f >= sdtracefrom && !melonDS::SDTrace) { melonDS::SDTrace = true; std::fprintf(stderr, "[sd] trace from frame %ld\n", f); }
        if (melonDS::SDTrace)
        {
            std::fprintf(stderr, "[sd] frame %ld ts %llu arm7 pc %08x lr %08x sp %08x cpsr %08x ie %08x if %08x ime %u\n", f, (unsigned long long)nds->ARM7Timestamp,
                         nds->ARM7.R[15], nds->ARM7.R[14], nds->ARM7.R[13], nds->ARM7.CPSR, nds->IE[1], nds->IF[1], nds->IME[1]);
            if (dsi)
            {
                auto* d = static_cast<melonDS::DSi*>(nds.get());
                char b[256];
                d->SDMMC.TraceState(b, sizeof b); std::fprintf(stderr, "[sd]   SDMMC %s ie2 %08x if2 %08x\n", b, d->IE2, d->IF2);
                for (int c = 4; c < 8; c++) { d->NDMAs[c].TraceState(b, sizeof b); std::fprintf(stderr, "[sd]   NDMA7.%d %s\n", c - 4, b); }
            }
        }
        nds->ARM9.StallUsed = 0; nds->ARM9.StallVblUsed = 0;
        nds->RunFrame();
        if (dsca7free)
        {
            u32 o = (prof::base + 0x24) & nds->MainRAMMask; u32 v; std::memcpy(&v, &nds->MainRAM[o], 4); v++; std::memcpy(&nds->MainRAM[o], &v, 4);
        }
        if (hashf)
        {
            // per frame: both screens, the audio the SPU produced, main RAM (first 4 MB, skips zeroed) as
            // one hash and as 64 KB block hashes (blocks every 60th frame)
            auto fnv = [](u64 h, const void* p, size_t n) { const u8* b = (const u8*)p; for (size_t i = 0; i < n; i++) { h ^= b[i]; h *= 1099511628211ull; } return h; };
            auto mix64 = [](u64 h, const u64* p, size_t n) { for (size_t i = 0; i < n; i++) { h ^= p[i]; h *= 0x100000001b3ull; h ^= h >> 29; } return h; };
            void* top; void* bot; u64 fh = 1469598103934665603ull;
            if (nds->GPU.GetFramebuffers(&top, &bot)) { fh = fnv(fh, top, 256 * 192 * 4); fh = fnv(fh, bot, 256 * 192 * 4); }
            u64 ah = 1469598103934665603ull; long ns = 0;
            for (;;) { int k = nds->SPU.ReadOutput(abuf.data(), 4096); if (k <= 0) break; ah = fnv(ah, abuf.data(), (size_t)k * 4); ns += k; if (audiof) std::fwrite(abuf.data(), 4, k, audiof); }
            ramcopy.assign(nds->MainRAM, nds->MainRAM + 0x400000);
            for (auto [lo, n] : hashskip) { u32 o = lo & 0x3FFFFF; if (o + n <= 0x400000) std::memset(&ramcopy[o], 0, n); }
            u64 rh = 1469598103934665603ull; std::vector<u64> blk(64);
            for (int b = 0; b < 64; b++) { blk[b] = mix64(1469598103934665603ull, (const u64*)&ramcopy[b * 0x10000], 0x10000 / 8); rh = mix64(rh, &blk[b], 1); }
            std::fprintf(hashf, "%ld,%016llx,%016llx,%ld,%016llx,%llu,%llu", f, (unsigned long long)fh, (unsigned long long)ah, ns, (unsigned long long)rh,
                         (unsigned long long)nds->ARM9.StallUsed, (unsigned long long)nds->ARM9.StallUnits);
            if (f % 60 == 0) for (int b = 0; b < 64; b++) std::fprintf(hashf, ",%016llx", (unsigned long long)blk[b]);
            std::fprintf(hashf, "\n");
        }
        if (tio)
        {
            auto& b = nds->GPU.GPU2D_B;
            std::fprintf(tio, "%ld,%u,%08x,%08x,%u,%08x,%08x,%04x,%04x,%08x,%04x,%u,%u,%u,%u,%u,%u,%u,%u,%04x,%04x,%u,%02x%02x,%04x,%04x,%08x,%08x", f,
                         nds->IME[0], nds->IE[0], nds->IF[0], nds->IME[1], nds->IE[1], nds->IF[1],
                         nds->ARM9Read16(0x04000102), nds->ARM9Read16(0x04000100), b.DispCnt, b.BGCnt[3],
                         b.BGXPos[0], b.BGYPos[0], b.BGXPos[1], b.BGYPos[1], b.BGXPos[2], b.BGYPos[2], b.BGXPos[3], b.BGYPos[3],
                         b.BlendCnt, b.BlendAlpha, b.EVY, b.BGMosaicSize[1], b.BGMosaicSize[0], nds->PowerControl9, nds->ARM7Read16(0x04000500), nds->ARM9.PU_DataRW, nds->ARM9.CPSR);
            for (u32 w : tracewords)
            {
                u32 o = w & nds->MainRAMMask & ~3u;
                u32 v; std::memcpy(&v, &nds->MainRAM[o], 4);
                std::fprintf(tio, ",%08x", v);
            }
            std::fprintf(tio, "\n");
        }
        for (auto& [sf, sp] : saveat)
        {
            if (sf != f) continue;
            Savestate st;
            nds->DoSavestate(&st);
            FILE* o = st.Error ? nullptr : std::fopen(sp.c_str(), "wb");
            FILE* m = o ? std::fopen((sp + ".meta").c_str(), "w") : nullptr;
            if (o && m)
            {
                std::fwrite(st.Buffer(), 1, st.Length(), o);
                std::fprintf(m, "%ld %ld\n", f, scriptbase >= 0 ? scriptbase : prof::startframe);
                std::fprintf(stderr, "[runner] state saved %s at frame %ld (script %ld)\n", sp.c_str(), f,
                             f - (scriptbase >= 0 ? scriptbase : prof::startframe));
            }
            else std::fprintf(stderr, "[runner] state save failed %s\n", sp.c_str());
            if (o) std::fclose(o);
            if (m) std::fclose(m);
        }
        for (auto& [df, dp] : dumpsat)
        {
            if (df != f) continue;
            FILE* d = std::fopen(dp.c_str(), "wb");
            if (d) { std::fwrite(nds->MainRAM, 1, nds->MainRAMMask + 1, d); std::fclose(d); }
        }
        if (tracepc >= 0 && f >= tracepc && (f - tracepc) % 30 == 0)
            std::printf("[pc] frame %ld pc=%08x lr=%08x sp=%08x cpsr=%08x\n", f, nds->ARM9.R[15], nds->ARM9.R[14],
                        nds->ARM9.R[13], nds->ARM9.CPSR);
    }

    if (tio) std::fclose(tio);
    if (hashf) std::fclose(hashf);
    if (audiof) std::fclose(audiof);
    if (prof::thr9csv) std::fclose(prof::thr9csv);
    if (prof::irq9csv) std::fclose(prof::irq9csv);
    if (nds->ARM9.Thr9Log) std::fclose(nds->ARM9.Thr9Log);
    if (nds->ARM9.Ev9Log) std::fclose(nds->ARM9.Ev9Log);
    if (nds->Spu7Log) std::fclose(nds->Spu7Log);
    for (auto& [k, n] : nds->ARM9.Call9)
        std::fprintf(stderr, "[runner] callers9 pc=%08x lr=%08x thread=%08x: %llu\n", std::get<0>(k), std::get<1>(k), std::get<2>(k), (unsigned long long)n);
    if (dscideal || dsccache)
        std::fprintf(stderr, "[dsc] ideal %d cache %d: IRQs taken at once in module code %llu (masked %llu); slice ends clamped %llu (%llu cycles dropped)\n",
                     dscideal, dsccache, (unsigned long long)nds->ARM9.DscIrqIdeal, (unsigned long long)nds->ARM9.DscIrqMasked,
                     (unsigned long long)nds->ARM9.DscClampN, (unsigned long long)nds->ARM9.DscClampCyc);
    if (stallvbl) std::fprintf(stderr, "[runner] stall at VBlank IRQ: %llu cycles, %llu frames\n", stallvbl, (unsigned long long)nds->ARM9.StallVblN);
    if (stallpc) std::fprintf(stderr, "[runner] stall pc %08x unit %llu per interval %llu check %llu: %llu units, %llu cycles in all, %llu aborted\n", stallpc, stallunit, stallpf, stallcheck,
                              (unsigned long long)nds->ARM9.StallUnits, (unsigned long long)nds->ARM9.StallTotal, (unsigned long long)nds->ARM9.StallAborts);
    if (!hl::linebuf.empty()) hl_log(Platform::LogLevel::Debug, "\n");
    if (!shot.empty()) write_ppm(*nds, shot);
    if (nwifi && !wpapmk.empty())
    {
        auto& w = nwifi->WpaStats;
        std::printf("[wpa] handshakes %u, group rekeys %u, pairwise keys ok %u, group keys ok %u, "
                    "key mismatches %u, bad frames %u, timeouts %u\n",
                    w.Handshakes, w.GroupRekeys, w.PairwiseKeysOK, w.GroupKeysOK, w.KeyMismatches, w.BadFrames, w.Timeouts);
    }
    if (!dumppath.empty())
    {
        FILE* df = std::fopen(dumppath.c_str(), "wb");
        if (df) { std::fwrite(nds->MainRAM, 1, nds->MainRAMMask + 1, df); std::fclose(df); }
    }
    if (nds->ARM9.CacheModel)
    {
        auto& c = nds->ARM9.CacheStats;
        std::fprintf(stderr, "[cache] I hit %llu miss %llu | D read hit %llu miss %llu | D write hit %llu miss %llu | writebacks %llu\n",
            (unsigned long long)c.IHit, (unsigned long long)c.IMiss, (unsigned long long)c.DReadHit, (unsigned long long)c.DReadMiss,
            (unsigned long long)c.DWriteHit, (unsigned long long)c.DWriteMiss, (unsigned long long)c.DWriteBack);
    }
    if (!nds->ARM9.Acc9.empty())
    {
        for (auto& [k, v] : nds->ARM9.Acc9)
            std::fprintf(stderr, "[runner] acc9 range %u pc=%08x %s%s: %llu (value changed %llu, last %08x) at %08x-%08x, frames %u-%u\n", k.Range, k.PC,
                         k.Write ? "write" : "read", k.IRQ ? " IRQ" : "", (unsigned long long)v.N, (unsigned long long)v.Changed, v.LastVal, v.Lo, v.Hi, v.FirstF, v.LastF);
    }
    if (nds->WatchHook)
    {
        std::printf("[watch] %lu writes\n", g_watchcount);
        for (auto& [k, w] : g_watchers)
            std::printf("[watch] ARM%d pc=%08x: %lu writes to %08x-%08x, frames %ld-%ld\n", k.first ? 7 : 9, k.second, w.n, w.lo, w.hi, w.first, w.last);
    }
    if (profile)
    {
        prof::report(*nds);
        if (prof::log) std::fclose(prof::log);
        if (prof::waitlog) { nds->ARM9.ProfWaitLog = nullptr; std::fclose(prof::waitlog); }
        if (!prof::pcspath.empty())
        {
            FILE* pf = std::fopen(prof::pcspath.c_str(), "w");
            if (pf)
            {
                std::fprintf(pf, "kind,addr,count,cycles,imiss,dreadmiss,dwritemiss\n");
                for (auto& [a, e] : prof::pcs)
                    std::fprintf(pf, "pc,%08x,%llu,%llu,%llu,%llu,%llu\n", a, (unsigned long long)e.Count, (unsigned long long)e.Cycles,
                                 (unsigned long long)e.IMiss, (unsigned long long)e.DReadMiss, (unsigned long long)e.DWriteMiss);
                for (auto& [a, n] : prof::dmiss)
                    std::fprintf(pf, "%s,%08x,%llu,,,,\n", (a & 2) ? ((a & 1) ? "uncached_w" : "uncached_r") : ((a & 1) ? "dmiss_w" : "dmiss_r"),
                                 a & ~3u, (unsigned long long)n);
                std::fclose(pf);
            }
        }
    }
    if (profile && !prof::fppath.empty())
    {
        FILE* pf = std::fopen(prof::fppath.c_str(), "w");
        if (pf)
        {
            auto& cpu = nds->ARM9;
            std::fprintf(pf, "summary,class,intervals,stretches,ilines,isets,downlines,downsets,dgamelines,dgamesets,dsets\n");
            for (int c = 0; c < 2; c++)
            {
                auto& f = cpu.Fp[c];
                std::fprintf(pf, "summary,%s,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu\n", c ? "pass" : "light",
                             (unsigned long long)prof::fpintervals, (unsigned long long)f.Stretches, (unsigned long long)f.ILines,
                             (unsigned long long)f.ISets, (unsigned long long)f.DOwnLines, (unsigned long long)f.DOwnSets,
                             (unsigned long long)f.DGameLines, (unsigned long long)f.DGameSets, (unsigned long long)f.DSets);
            }
            std::fprintf(pf, "line,kind,addr,light,pass\n");
            for (auto& [k, v] : prof::fplines)
                std::fprintf(pf, "line,%c,%08x,%llu,%llu\n", (k & 1) ? 'D' : 'I', k & ~0x1Fu, (unsigned long long)v.first, (unsigned long long)v.second);
            std::fclose(pf);
        }
    }
    if (profile && !prof::gamepcspath.empty())
    {
        FILE* pf = std::fopen(prof::gamepcspath.c_str(), "w");
        if (pf)
        {
            // code: key = the fetched line; data: key = the instruction's PC (cycles in 133 MHz units)
            std::fprintf(pf, "addr,iextra,dextra,poll_i,poll_d\n");
            for (auto& [a, e] : prof::gamepcs)
                std::fprintf(pf, "%08x,%lld,%lld,%llu,%llu\n", a, (long long)e.IExtra, (long long)e.DExtra,
                             (unsigned long long)e.PollI, (unsigned long long)e.PollD);
            std::fclose(pf);
        }
    }
    if (profile && !prof::arm7path.empty())
    {
        FILE* pf = std::fopen(prof::arm7path.c_str(), "w");
        if (pf)
        {
            std::fprintf(pf, "block,cycles\n");
            for (size_t i = 0; i < prof::arm7blocks.size(); i++)
                if (prof::arm7blocks[i]) std::fprintf(pf, "%08x,%llu\n", (u32)i << 8, (unsigned long long)prof::arm7blocks[i]);
            std::fclose(pf);
        }
    }
    if (profile && !prof::idlepcspath.empty())
    {
        FILE* pf = std::fopen(prof::idlepcspath.c_str(), "w");
        if (pf)
        {
            std::fprintf(pf, "kind,addr,count,cycles,imiss,dreadmiss,dwritemiss,masked_pend\n");
            for (size_t i = 0; i < prof::idlepcs.size(); i++)
            {
                auto& e = prof::idlepcs[i];
                if (e.Count)
                    std::fprintf(pf, "pc,%08x,%llu,%llu,%llu,%llu,%llu,%llu\n", prof::base + (u32)i * 2, (unsigned long long)e.Count,
                                 (unsigned long long)e.Cycles, (unsigned long long)e.IMiss, (unsigned long long)e.DReadMiss,
                                 (unsigned long long)e.DWriteMiss, (unsigned long long)e.MaskedPend);
            }
            std::fclose(pf);
        }
    }
    if (profile && !prof::idledmisspath.empty())
    {
        FILE* pf = std::fopen(prof::idledmisspath.c_str(), "w");
        if (pf)
        {
            std::fprintf(pf, "kind,addr,count\n");
            for (auto& [a, n] : prof::idledmiss)
                std::fprintf(pf, "%s,%08x,%llu\n", (a & 2) ? ((a & 1) ? "uncached_w" : "uncached_r") : ((a & 1) ? "dmiss_w" : "dmiss_r"),
                             a & ~3u, (unsigned long long)n);
            std::fclose(pf);
        }
    }
    if (profile && !prof::rolespath.empty() && !prof::roles.empty())
    {
        FILE* pf = std::fopen((prof::rolespath + ".csv").c_str(), "w");
        if (pf)
        {
            std::fprintf(pf, "role,calls,count,cycles,imiss,dreadmiss,dwritemiss,frames\n");
            for (size_t r = 0; r < prof::roles.size(); r++)
            {
                auto& e = prof::roles[r];
                std::fprintf(pf, "%s,%llu,%llu,%llu,%llu,%llu,%llu,%llu\n", prof::rolenames[r].c_str(), (unsigned long long)prof::rolecalls[r],
                             (unsigned long long)e.Count, (unsigned long long)e.Cycles, (unsigned long long)e.IMiss,
                             (unsigned long long)e.DReadMiss, (unsigned long long)e.DWriteMiss, (unsigned long long)prof::n);
            }
            std::fclose(pf);
        }
    }
    g_net = nullptr;
    std::fprintf(stderr, "[runner] %ld frames, %s\n", f,
                 hl::hit_until ? "marker reached" : hl::stopped ? "emulator stopped" : prof::done ? "profile frames done" : "frame limit");
    return hl::hit_until || hl::until.empty() ? 0 : 1;
}
