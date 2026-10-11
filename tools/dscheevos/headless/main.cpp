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
#include <vector>
#include <memory>
#include <optional>
#include <fstream>
#include <filesystem>

#include "NDS.h"
#include "DSi.h"
#include "Args.h"
#include "NDSCart.h"
#include "SPI_Firmware.h"
#include "DSi_NAND.h"
#include "FATStorage.h"
#include "Platform.h"

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
        "  --firmware-boot      boot through firmware instead of direct boot\n"
        "  --cache-model        time ARM9 cache hits/misses instead of melonDS's flat cache cost\n"
        "  --verbose            show emulator info/debug logs\n");
    std::exit(2);
}

int main(int argc, char** argv)
{
    std::string rom, mode = "ds", biosdir, nandpath, sddir, shot;
    long frames = 600; u32 keys = 0; bool direct = true; bool cachemodel = false;
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
        else if (a == "--firmware-boot") direct = false;
        else if (a == "--verbose") hl::verbose = true;
        else if (a == "--cache-model") cachemodel = true;
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
            dargs.DSiSDCard = FATStorage(FATStorageArgs{sddir + ".img", 0, false, sddir});
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
        cartargs = NDSCart::NDSCartArgs{.SDCard = FATStorageArgs{sddir + ".img", 0, false, sddir}};
    }
    auto cart = NDSCart::ParseROM(romdata.data(), (u32)romdata.size(), nullptr, std::move(cartargs));
    if (!cart) { std::fprintf(stderr, "ROM did not parse\n"); return 2; }

    nds->Reset();
    nds->SetNDSCart(std::move(cart));
    if (direct || nds->NeedsDirectBoot())
        nds->SetupDirectBoot(fs::path(rom).filename().string());
    nds->ARM9.CacheModel = cachemodel;
    nds->Start();

    // melonDS key mask: bit set = released
    nds->SetKeyMask(~keys & 0xFFF);

    long f = 0;
    for (; f < frames && !hl::hit_until && !hl::stopped; f++)
        nds->RunFrame();

    if (!hl::linebuf.empty()) hl_log(Platform::LogLevel::Debug, "\n");
    if (!shot.empty()) write_ppm(*nds, shot);
    if (cachemodel)
    {
        auto& c = nds->ARM9.CacheStats;
        std::fprintf(stderr, "[cache] I hit %llu miss %llu | D read hit %llu miss %llu | D write hit %llu miss %llu | writebacks %llu\n",
            (unsigned long long)c.IHit, (unsigned long long)c.IMiss, (unsigned long long)c.DReadHit, (unsigned long long)c.DReadMiss,
            (unsigned long long)c.DWriteHit, (unsigned long long)c.DWriteMiss, (unsigned long long)c.DWriteBack);
    }
    std::fprintf(stderr, "[runner] %ld frames, %s\n", f,
                 hl::hit_until ? "marker reached" : hl::stopped ? "emulator stopped" : "frame limit");
    return hl::hit_until || hl::until.empty() ? 0 : 1;
}
