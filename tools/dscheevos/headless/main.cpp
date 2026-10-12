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
#include "Platform.h"
#include "Net.h"
#include "Net_Slirp.h"

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
long startframe = -1;       // frame of the game's first OS_Halt call
u64 n = 0, lag = 0, lagrun = 0, maxlagrun = 0;
u64 vbl = 0, vblmax = 0, idle = 0, idlemax = 0, irqmode = 0, busy = 0, before = 0, halted = 0, haltedbios = 0;
bool done = false;
bool cachefromgame = false; // --cache-model-from-game
std::string pcspath;        // --profile-pcs
std::map<u32, ARMv5::ProfPCStat> pcs;
std::map<u32, u64> dmiss;

static u32 ram32(u32 addr) { u32 v; std::memcpy(&v, &nds->MainRAM[addr & nds->MainRAMMask & ~3u], 4); return v; }

// The module header (dscheevos/module/src/start.s): magic, version, image
// size, init, VBlank entry (0x10), halt entry (0x14, right after the VBlank
// entry stub). Stacks from module.ld: idle on base+0x200000 down, the VBlank
// call 0x2000 below that, down towards the image.
static void find_module(ARMv5& cpu)
{
    u32 size = ram32(base + 8), vblank = ram32(base + 0x10), halt = ram32(base + 0x14);
    if (ram32(base) == 0x56435344 && size >= 0x28 && size <= 0x80000 && halt >= base && halt < base + size
        && vblank >= base && vblank < halt)
    {
        cpu.ProfLo = base; cpu.ProfHi = base + size; cpu.ProfHaltEntry = halt;
        cpu.ProfVBlankLo = vblank; cpu.ProfVBlankHi = halt;
        cpu.ProfIrqStackLo = base + size; cpu.ProfIrqStackHi = base + 0x200000 - 0x2000;
    }
    else
        cpu.ProfLo = cpu.ProfHi = cpu.ProfHaltEntry = cpu.ProfVBlankLo = cpu.ProfVBlankHi = cpu.ProfIrqStackLo = cpu.ProfIrqStackHi = 0;
}

static void on_vblank(ARMv5& cpu)
{
    auto& p = cpu.Prof;
    if (startframe < 0)
    {
        before += p.VBlank + p.Idle;
        if (p.Halts)
        {
            startframe = hl::frame;
            if (cachefromgame)
            {
                cpu.CacheModelReset();
                cpu.CacheModel = true;
            }
            if (!pcspath.empty()) { cpu.ProfPCs = &pcs; cpu.ProfDMiss = &dmiss; cpu.ProfPrevStats = cpu.CacheStats; }
        }
    }
    else if (!done)
    {
        n++;
        vbl += p.VBlank; vblmax = std::max(vblmax, p.VBlank);
        idle += p.Idle; idlemax = std::max(idlemax, p.Idle);
        irqmode += p.IRQMode;
        busy += p.Busy;
        halted += p.Halted; haltedbios += p.HaltedBios;
        bool lagged = p.IdleArrivals == 0;
        if (lagged) { lag++; maxlagrun = std::max(maxlagrun, ++lagrun); }
        else lagrun = 0;
        if (log)
            std::fprintf(log, "%ld,%llu,%llu,%llu,%llu,%u,%d,%08x,%08x,%08x,%llu,%llu\n", hl::frame, (unsigned long long)p.VBlank, (unsigned long long)p.Idle,
                         (unsigned long long)p.IRQMode, (unsigned long long)p.Busy, p.IdleArrivals, lagged,
                         cpu.R[15], nds->ARM7.R[15], cpu.CPSR, (unsigned long long)p.Halted, (unsigned long long)p.HaltedBios);
        if (stopafter >= 0 && (long)n >= stopafter)
        {
            done = true;
            cpu.ProfPCs = nullptr; cpu.ProfDMiss = nullptr;
        }
    }
    p.VBlank = p.Idle = p.IRQMode = p.Busy = p.Halted = p.HaltedBios = 0;
    p.IdleArrivals = 0;
    find_module(cpu);
}

static void report(NDS& nds)
{
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
    std::fprintf(stderr, "[prof] lag: %llu of %llu intervals (%.2f%%), longest run %llu\n", (unsigned long long)lag,
                 (unsigned long long)n, 100.0 * lag / n, (unsigned long long)maxlagrun);
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
        "  --profile-log FILE   with --profile: CSV per VBlank interval\n"
        "  --profile-pcs FILE   with --profile: the VBlank call's cycles and cache misses by PC, and its\n"
        "                       data misses and uncached accesses by address (CSV, from the game's start)\n"
        "  --script-from-game   --script frames count from the game's first OS_Halt call (needs --profile)\n"
        "  --cache-model-from-game  like --cache-model, switched on at the game's first OS_Halt call\n"
        "                       (needs --profile; nds-bootstrap's boot hangs with the cache model on)\n"
        "  --watch-write ADDR:SIZE  DSi mode: print bus writes (CPU or DMA) to [ADDR, ADDR+SIZE) with the\n"
        "                       frame and the CPU's PC (first 40; the count at exit)\n"
        "  --sd-cmd-busy N      DSi SD/MMC: CMD_BUSY reads as set for N ARM7 cycles after a command\n"
        "                       (default: ~96 SD clocks; 0 = never seen, as when an IRQ delays the poll)\n"
        "  --verbose            show emulator info/debug logs\n");
    std::exit(2);
}

int main(int argc, char** argv)
{
    std::string rom, mode = "ds", biosdir, nandpath, sddir, shot, scriptpath, dumppath, wpaap, wpassid;
    std::vector<u8> wpapmk;
    long tracepc = -1, sdtracefrom = -1;
    bool traceled = false;
    bool profile = false, scriptfromgame = false;
    std::string proflog, watch;
    long frames = 600; u32 keys = 0; bool direct = true; bool cachemodel = false; bool net = false;
    std::string rtc = "host";
    std::vector<std::string> guestargs;
    for (int i = 1; i < argc; i++)
    {
        std::string a = argv[i];
        auto next = [&]() -> std::string { if (i + 1 >= argc) usage(); return argv[++i]; };
        if (a == "--rom") rom = next();
        else if (a == "--mode") mode = next();
        else if (a == "--bios-dir") biosdir = next();
        else if (a == "--nand") nandpath = next();
        else if (a == "--sd") sddir = next();
        else if (a == "--frames") frames = std::stol(next());
        else if (a == "--until") hl::until = next();
        else if (a == "--screenshot") shot = next();
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
        else if (a == "--script-from-game") scriptfromgame = true;
        else if (a == "--cache-model-from-game") prof::cachefromgame = true;
        else if (a == "--profile-pcs") prof::pcspath = next();
        else if (a == "--trace-sd") sdtracefrom = std::stol(next());
        else if (a == "--sd-cmd-busy") melonDS::SDCmdBusyCycles = std::stol(next());
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
    if (profile)
    {
        prof::nds = nds.get();
        nds->ARM9.ProfOn = true;
        nds->ARM9.ProfVBlank = prof::on_vblank;
        if (!proflog.empty())
        {
            prof::log = std::fopen(proflog.c_str(), "w");
            if (!prof::log) { std::fprintf(stderr, "cannot write %s\n", proflog.c_str()); return 2; }
            std::fprintf(prof::log, "frame,module_vblank,module_idle,module_irq_mode,busy,idle_arrivals,lag,arm9_pc,arm7_pc,arm9_cpsr,halted,halted_bios\n");
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

    long f = 0;
    for (; f < frames && !hl::hit_until && !hl::stopped && !prof::done; f++)
    {
        u32 held = keys;
        int touchx = -1, touchy = -1;
        // script time: frames since boot, or since the game's first OS_Halt call
        long st = !scriptfromgame ? f : prof::startframe >= 0 ? f - prof::startframe : -1;
        for (auto& p : script)
        {
            if (st < 0) break;
            if (p.dump || p.shot)
            {
                // RAM dump / screenshot of what the previous frame left, next to
                // --dump-ram / --screenshot, named by script frame: FILE.f<frame>
                const std::string& base = p.dump ? dumppath : shot;
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
                            std::fwrite(nds->ARM9.DTCM, 1, sizeof(nds->ARM9.DTCM), df);
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
        nds->RunFrame();
        if (tracepc >= 0 && f >= tracepc && (f - tracepc) % 30 == 0)
            std::printf("[pc] frame %ld pc=%08x lr=%08x sp=%08x cpsr=%08x\n", f, nds->ARM9.R[15], nds->ARM9.R[14],
                        nds->ARM9.R[13], nds->ARM9.CPSR);
    }

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
    g_net = nullptr;
    std::fprintf(stderr, "[runner] %ld frames, %s\n", f,
                 hl::hit_until ? "marker reached" : hl::stopped ? "emulator stopped" : prof::done ? "profile frames done" : "frame limit");
    return hl::hit_until || hl::until.empty() ? 0 : 1;
}
