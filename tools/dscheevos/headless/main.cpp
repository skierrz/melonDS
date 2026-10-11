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
#include <algorithm>
#include <memory>
#include <optional>
#include <fstream>
#include <filesystem>

#include "NDS.h"
#include "DSi.h"
#include "DSi_NWifi.h"
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
}

// Wi-Fi: frames from the emulated Wi-Fi chip go through a user-mode network
// stack (libslirp). The guest gets DHCP on 10.64.0.0/24; 10.64.0.1 reaches
// the host, so a mock server on the host is at that address.
static melonDS::Net* g_net = nullptr;
int hl_net_send(melonDS::u8* data, int len) { return g_net ? g_net->SendPacket(data, len, 0) : len; }
int hl_net_recv(melonDS::u8* data) { return g_net ? g_net->RecvPacket(data, 0) : 0; }

void hl_signal_stop() { hl::stopped = true; }

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
        "                       \"FRAME WIFIREKEY\": it starts a WPA2 group rekey\n"
        "  --dump-ram FILE      write main RAM (4 or 16 MB) at the end\n"
        "  --firmware-boot      boot through firmware instead of direct boot\n"
        "  --trace-pc FRAME     from this frame on, print the ARM9 PC, LR and SP every 30 frames\n"
        "  --wpa-ap SSID:PMK    DSi mode: a saved WPA2 access point (slot 4) with this PMK (64 hex digits);\n"
        "                       with --net and SSID melonAP, the emulated access point uses WPA2 too\n"
        "  --arg STR            direct boot: pass STR as the next argv entry (first one is argv[0],\n"
        "                       e.g. sd:/_nds/nds-bootstrap-hb-nightly.nds), like a homebrew loader does\n"
        "  --net                connect Wi-Fi to a user-mode network (host reachable at 10.64.0.1)\n"
        "  --cache-model        time ARM9 cache hits/misses instead of melonDS's flat cache cost\n"
        "  --rtc TIME           console clock at boot: \"host\" (default, the host's local time),\n"
        "                       or YYYY-MM-DDTHH:MM:SS\n"
        "  --verbose            show emulator info/debug logs\n");
    std::exit(2);
}

int main(int argc, char** argv)
{
    std::string rom, mode = "ds", biosdir, nandpath, sddir, shot, scriptpath, dumppath, wpaap, wpassid;
    std::vector<u8> wpapmk;
    long tracepc = -1;
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
        else if (a == "--firmware-boot") direct = false;
        else if (a == "--arg") guestargs.push_back(next());
        else if (a == "--verbose") hl::verbose = true;
        else if (a == "--cache-model") cachemodel = true;
        else if (a == "--rtc") rtc = next();
        else if (a == "--net") net = true;
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

    struct Press { long frame; u32 keys; long hold; bool wifidrop; bool wifirekey; };
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
            if (std::sscanf(line, "%ld %15s", &p.frame, word) == 2 && !std::strcmp(word, "WIFIDROP"))
                p.wifidrop = true;
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

    long f = 0;
    for (; f < frames && !hl::hit_until && !hl::stopped; f++)
    {
        u32 held = keys;
        for (auto& p : script)
        {
            if (!p.wifidrop && !p.wifirekey && f >= p.frame && f < p.frame + p.hold) held |= p.keys;
            if ((p.wifidrop || p.wifirekey) && f == p.frame && nwifi)
            {
                if (p.wifidrop) nwifi->DropConnection();
                else nwifi->StartGroupRekey();
            }
        }
        nds->SetKeyMask(~held & 0xFFF);
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
    if (cachemodel)
    {
        auto& c = nds->ARM9.CacheStats;
        std::fprintf(stderr, "[cache] I hit %llu miss %llu | D read hit %llu miss %llu | D write hit %llu miss %llu | writebacks %llu\n",
            (unsigned long long)c.IHit, (unsigned long long)c.IMiss, (unsigned long long)c.DReadHit, (unsigned long long)c.DReadMiss,
            (unsigned long long)c.DWriteHit, (unsigned long long)c.DWriteMiss, (unsigned long long)c.DWriteBack);
    }
    g_net = nullptr;
    std::fprintf(stderr, "[runner] %ld frames, %s\n", f,
                 hl::hit_until ? "marker reached" : hl::stopped ? "emulator stopped" : "frame limit");
    return hl::hit_until || hl::until.empty() ? 0 : 1;
}
