// --survey: the DScheevos display survey (M1). Watches the game's own use of
// the engine A/B display registers, IE, DMA and raster IRQs from the ARM9, in
// plain DS mode, and writes:
//   FILE.json   aggregates (DISPCNT accesses by PC and line bucket, absolute
//               IE writes, display register writes by line, capture, DMA
//               mode 4, the IRQ handler at DTCM+3FFC)
//   FILE.frames.csv  one row per frame, sampled at line 0
// The only change to the game's view of the machine: ARM9 IE bit 28 is set at
// boot and put back after every game IE write (a "phantom" bit: nothing raises
// IF bit 28 in DS mode, so it never fires). A game write that clears it is an
// absolute IE write.

#include <cstdio>
#include <cstring>
#include <map>
#include <set>
#include <vector>
#include <array>
#include <string>
#include <tuple>

#include "NDS.h"
#include "GPU.h"

using namespace melonDS;

namespace survey
{
static FILE* csv = nullptr;
static std::string jsonpath;
static long frame = -1;          // frame count, advanced at each line 0

// per-frame counters (reset at line 0)
static int fr_hbl = 0, fr_hbl_win = 0;      // lines with the ARM9 HBlank IRQ armed; of them, lines 28-40
static std::vector<int> fr_vc;              // ARM9 VCOUNT IRQ match lines
static int fr_dwA = 0, fr_drA = 0, fr_dwB = 0, fr_drB = 0;

static std::array<u64, 263> hbl_lines{}, vc_lines{};
static u64 hbl_frames = 0, vc_frames = 0;

static int bucket(u32 line)
{
    if (line <= 35) return 0;
    if (line <= 191) return 1;
    if (line <= 260) return 2;
    return 3;
}
static bool in_window(u32 line) { return line <= 35 || line >= 261; }

struct Ctx { u32 pc; int ctx; }; // ctx: CPU mode (0x10..0x1F) or 0x100+DMA channel
static Ctx context(NDS& nds)
{
    if (nds.SurveyDMA9 >= 0) return {0xD0000000u | (u32)nds.SurveyDMA9, 0x100 + nds.SurveyDMA9};
    auto& c = nds.ARM9;
    bool t = c.CPSR & 0x20;
    return {(c.R[15] - (t ? 4 : 8)) | (t ? 1 : 0), (int)(c.CPSR & 0x1F)};
}

// ---- DISPCNT ----
struct DKey
{
    int eng, write, size, off; u32 pc; int ctx;
    bool operator<(const DKey& o) const { return std::tie(eng, write, size, off, pc, ctx) < std::tie(o.eng, o.write, o.size, o.off, o.pc, o.ctx); }
};
struct DAgg
{
    u64 n = 0, b[4] = {}, win = 0;
    long first = -1, last = -1;
    // writes touching byte 2: pure RMW (mode bits as read), RMW changing the mode, absolute
    u64 rmw = 0, rmwmode = 0, abs = 0, win_rmw = 0, win_rmwmode = 0, win_abs = 0;
    u64 modechg = 0, modechg_outvbl = 0;
    u32 lastold = 0, lastnew = 0, lastline = 0;
    std::array<u64, 263> lines{};
};
static std::map<DKey, DAgg> dagg;
static u32 dpre[2];
// last DISPCNT A read covering byte 2, and the last write
static long lr_t = -1; static u32 lr_val = 0; static u64 lr_seq = 0, lw_seq = 0, seq = 0;
static int winlog_n = 0;
struct WinEv { long frame; u32 line; int eng, write, size, off; u32 pc; int ctx; u32 oldv, newv; const char* cls; };
static std::vector<WinEv> winev;            // first 400 DISPCNT accesses in the window
static std::vector<WinEv> modeev;           // first 200 DISPCNT A mode changes

// ---- display register writes: per (addr, pc, ctx) line histogram ----
struct RKey
{
    u32 addr; u32 pc; int ctx;
    bool operator<(const RKey& o) const { return std::tie(addr, pc, ctx) < std::tie(o.addr, o.pc, o.ctx); }
};
struct RAgg { u64 n = 0; long lastframe = -1; u64 frames = 0; std::array<u32, 263> lines{}; };
static std::map<RKey, RAgg> ragg;

// ---- IE ----
struct IEEv { long frame; u32 line; u32 pc; int ctx; u32 val; int size; u32 addr; };
static std::vector<IEEv> ieev;
static u64 ie_abs = 0, ie_writes = 0;
static std::map<u32, u64> ie_abs_pc;

// ---- capture, FIFO, DMA mode 4 ----
static std::map<std::pair<u32, u32>, u64> capw;   // (CaptureCnt after write, pc) -> count, enabling writes only
static u64 cap_fifo = 0, cap_en = 0;
static u64 fifo_writes = 0;
static std::map<u32, u64> fifo_pc;
static u64 dma4 = 0; static std::vector<std::pair<long, u32>> dma4ev;

// ---- IRQ handler samples ----
struct HSample { long frame; u32 dtcm, ptr; std::vector<u8> code; };
static std::vector<HSample> hs;

static void sample_handler(NDS& nds)
{
    auto& c = nds.ARM9;
    HSample s{frame, c.DTCMBase, 0, {}};
    if (c.DTCMBase == 0xFFFFFFFF || !c.DTCM) { hs.push_back(s); return; }
    std::memcpy(&s.ptr, &c.DTCM[0x3FFC], 4);
    for (u32 i = 0; i < 0x80; i += 4)
    {
        u32 a = (s.ptr & ~1u) + i, v;
        if (a < c.ITCMSize) std::memcpy(&v, &c.ITCM[a & (ITCMPhysicalSize - 1)], 4);
        else if ((a & c.DTCMMask) == c.DTCMBase) std::memcpy(&v, &c.DTCM[a & (DTCMPhysicalSize - 1)], 4);
        else
        {
            auto hook = nds.IOHook; nds.IOHook = nullptr;
            v = nds.ARM9Read32(a);
            nds.IOHook = hook;
        }
        for (int k = 0; k < 4; k++) s.code.push_back(u8(v >> (k * 8)));
    }
    hs.push_back(s);
}

static void on_line(NDS& nds, u32 vc)
{
    auto& g = nds.GPU;
    bool ime = nds.IME[0] & 1;
    if (vc == 0)
    {
        if (frame >= 0 && csv)
        {
            std::string v;
            for (size_t i = 0; i < fr_vc.size() && i < 8; i++) { if (i) v += ';'; v += std::to_string(fr_vc[i]); }
            if (fr_hbl) hbl_frames++;
            if (!fr_vc.empty()) vc_frames++;
            std::fprintf(csv, ",%d,%d,%s,%d,%d,%d,%d\n", fr_hbl, fr_hbl_win, v.c_str(), fr_dwA, fr_drA, fr_dwB, fr_drB);
        }
        frame++;
        fr_hbl = fr_hbl_win = 0; fr_vc.clear(); fr_dwA = fr_drA = fr_dwB = fr_drB = 0;
        u16 pow = nds.ARM9IORead16(0x04000304);
        if (csv)
            std::fprintf(csv, "%ld,%08x,%08x,%04x,%04x,%08x,%08x", frame, g.GPU2D_A.DispCnt, g.GPU2D_B.DispCnt,
                         g.MasterBrightnessA, pow, g.CaptureCnt, nds.IE[0]);
        if (frame == 600 || frame == 3600 || frame == 18000)
            sample_handler(nds);
    }
    if (ime && (nds.IE[0] & 2) && (g.DispStat[0] & 0x10))
    {
        fr_hbl++; hbl_lines[vc]++;
        if (vc >= 28 && vc <= 40) fr_hbl_win++;
    }
    u32 lyc = (g.DispStat[0] >> 8) | ((g.DispStat[0] & 0x80) << 1);
    if (ime && (nds.IE[0] & 4) && (g.DispStat[0] & 0x20) && vc == lyc)
    {
        fr_vc.push_back((int)vc); vc_lines[vc]++;
    }
}

static bool covers(u32 addr, int size, u32 lo, u32 hi) { return addr < hi && addr + size > lo; }

static void on_io(NDS& nds, int kind, u32 addr, u32 val, int size)
{
    u32 line = nds.GPU.VCount;
    // DISPCNT A / B
    for (int eng = 0; eng < 2; eng++)
    {
        u32 base = eng ? 0x04001000 : 0x04000000;
        if (!covers(addr, size, base, base + 4)) continue;
        u32 cur = eng ? nds.GPU.GPU2D_B.DispCnt : nds.GPU.GPU2D_A.DispCnt;
        if (kind == 1) { dpre[eng] = cur; return; }
        Ctx c = context(nds);
        int off = (int)(addr - base);
        DKey k{eng, kind == 2, size, off, c.pc, c.ctx};
        auto& a = dagg[k];
        a.n++; a.b[bucket(line)]++; a.lines[line % 263]++;
        if (a.first < 0) a.first = frame;
        a.last = frame;
        bool win = in_window(line);
        if (win) a.win++;
        bool b2 = covers(addr, size, base + 2, base + 3);
        long t = frame * 263 + line;
        const char* cls = "";
        if (kind == 0)
        {
            (eng ? fr_drB : fr_drA)++;
            a.lastnew = val; a.lastline = line;
            if (!eng && b2) { lr_t = t; lr_val = cur; lr_seq = ++seq; }
            cls = b2 ? "read-b2" : "read";
        }
        else
        {
            (eng ? fr_dwB : fr_dwA)++;
            a.lastold = dpre[eng]; a.lastnew = cur; a.lastline = line;
            if (b2)
            {
                bool rmw = !eng && lr_seq > lw_seq && t - lr_t <= 2;
                if (eng) { cls = "write-b2"; }
                else if (rmw && !((cur ^ lr_val) & 0x30000)) { a.rmw++; if (win) a.win_rmw++; cls = "rmw"; }
                else if (rmw) { a.rmwmode++; if (win) a.win_rmwmode++; cls = "rmw-mode"; }
                else { a.abs++; if (win) a.win_abs++; cls = "abs"; }
                if ((cur ^ dpre[eng]) & 0x30000)
                {
                    a.modechg++;
                    bool vbl = line >= 192 && line <= 260;
                    if (!vbl) a.modechg_outvbl++;
                    if (!eng && modeev.size() < 200)
                        modeev.push_back({frame, line, eng, 1, size, off, c.pc, c.ctx, dpre[eng], cur, vbl ? "vblank" : "OUTSIDE-VBLANK"});
                }
            }
            else cls = "write-lo";
            if (!eng) lw_seq = ++seq;
        }
        if (win && winev.size() < 400)
            winev.push_back({frame, line, eng, kind == 2, size, off, c.pc, c.ctx, kind == 2 ? dpre[eng] : 0, kind == 2 ? cur : val, cls});
        return;
    }
    if (kind == 0) return;
    // IE (after the write): bit 28 gone = an absolute write
    if (covers(addr, size, 0x04000210, 0x04000214))
    {
        if (kind != 2) return;
        ie_writes++;
        if (!(nds.IE[0] & (1u << 28)))
        {
            Ctx c = context(nds);
            ie_abs++; ie_abs_pc[c.pc]++;
            if (ieev.size() < 300) ieev.push_back({frame, line, c.pc, c.ctx, nds.IE[0], size, addr});
            nds.IE[0] |= 1u << 28;
        }
        return;
    }
    if (kind != 2) return;
    // display registers 0x04000000-6F / 0x04001000-6F (writes, after the write)
    bool dispA = addr >= 0x04000000 && addr < 0x04000070, dispB = addr >= 0x04001000 && addr < 0x04001070;
    if (dispA || dispB)
    {
        Ctx c = context(nds);
        auto& r = ragg[{addr, c.pc, c.ctx}];
        r.n++; r.lines[line % 263]++;
        if (r.lastframe != frame) { r.frames++; r.lastframe = frame; }
        if (covers(addr, size, 0x04000064, 0x04000068))
        {
            u32 cc = nds.GPU.CaptureCnt;
            if (cc & 0x80000000u)
            {
                cap_en++;
                capw[{cc, c.pc}]++;
                u32 sel = (cc >> 29) & 3;
                if (sel != 0 && (cc & (1u << 25))) cap_fifo++;
            }
        }
        if (covers(addr, size, 0x04000068, 0x0400006C)) { fifo_writes++; fifo_pc[c.pc]++; }
        return;
    }
    // ARM9 DMA control: start mode 4 (main memory display)
    if (addr >= 0x040000B0 && addr < 0x040000E0)
    {
        u32 rel = (addr - 0x040000B0) % 12;
        u32 hi = 0; bool have = false;
        if (size == 4 && rel == 8) { hi = val >> 16; have = true; }
        else if (size == 2 && rel == 10) { hi = val & 0xFFFF; have = true; }
        else if (size == 1 && rel == 11) { hi = (val & 0xFF) << 8; have = true; }
        if (have && (hi & 0x8000) && ((hi >> 11) & 7) == 4)
        {
            dma4++;
            if (dma4ev.size() < 50) dma4ev.push_back({frame, context(nds).pc});
        }
    }
}

static const char* ctxname(int ctx)
{
    static char buf[8];
    switch (ctx)
    {
    case 0x10: return "usr"; case 0x11: return "fiq"; case 0x12: return "irq"; case 0x13: return "svc";
    case 0x17: return "abt"; case 0x1B: return "und"; case 0x1F: return "sys";
    }
    if (ctx >= 0x100) { std::snprintf(buf, sizeof buf, "dma%d", ctx - 0x100); return buf; }
    std::snprintf(buf, sizeof buf, "m%02x", ctx); return buf;
}

bool install(NDS& nds, const std::string& prefix)
{
    jsonpath = prefix + ".json";
    csv = std::fopen((prefix + ".frames.csv").c_str(), "w");
    if (!csv) return false;
    std::fprintf(csv, "frame,dispA,dispB,brightA,powcnt1,capcnt,ie,hbl_lines,hbl_lines_28_40,vc_lines,dwA,drA,dwB,drB\n");
    nds.IE[0] |= 1u << 28;     // the phantom bit
    nds.IOHook = on_io;
    nds.LineHook = on_line;
    return true;
}

void finish(NDS& nds)
{
    if (!csv) return;
    std::fprintf(csv, ",%d,%d,,%d,%d,%d,%d\n", fr_hbl, fr_hbl_win, fr_dwA, fr_drA, fr_dwB, fr_drB);
    std::fclose(csv); csv = nullptr;
    sample_handler(nds);
    FILE* j = std::fopen(jsonpath.c_str(), "w");
    if (!j) return;
    std::fprintf(j, "{\n\"frames\": %ld,\n\"ie_writes\": %llu,\n\"ie_abs\": %llu,\n", frame + 1,
                 (unsigned long long)ie_writes, (unsigned long long)ie_abs);
    std::fprintf(j, "\"ie_abs_pc\": {");
    { bool f = true; for (auto& [pc, n] : ie_abs_pc) { std::fprintf(j, "%s\"%08x\": %llu", f ? "" : ", ", pc, (unsigned long long)n); f = false; } }
    std::fprintf(j, "},\n\"ie_abs_events\": [");
    for (size_t i = 0; i < ieev.size(); i++)
    {
        auto& e = ieev[i];
        std::fprintf(j, "%s\n {\"frame\": %ld, \"line\": %u, \"pc\": \"%08x\", \"ctx\": \"%s\", \"ie_after\": \"%08x\", \"size\": %d, \"addr\": \"%08x\"}",
                     i ? "," : "", e.frame, e.line, e.pc, ctxname(e.ctx), e.val, e.size, e.addr);
    }
    std::fprintf(j, "],\n\"dispcnt\": [");
    { bool f = true;
    for (auto& [k, a] : dagg)
    {
        std::fprintf(j, "%s\n {\"eng\": \"%c\", \"op\": \"%s\", \"size\": %d, \"off\": %d, \"pc\": \"%08x\", \"ctx\": \"%s\", \"n\": %llu, "
                     "\"b0_35\": %llu, \"b36_191\": %llu, \"b192_260\": %llu, \"b261_262\": %llu, \"win\": %llu, \"first\": %ld, \"last\": %ld, "
                     "\"rmw\": %llu, \"rmw_mode\": %llu, \"abs\": %llu, \"win_rmw\": %llu, \"win_rmw_mode\": %llu, \"win_abs\": %llu, "
                     "\"modechg\": %llu, \"modechg_outvbl\": %llu, \"last_old\": \"%08x\", \"last_new\": \"%08x\", \"last_line\": %u, \"lines\": [",
                     f ? "" : ",", k.eng ? 'B' : 'A', k.write ? "w" : "r", k.size, k.off, k.pc, ctxname(k.ctx), (unsigned long long)a.n,
                     (unsigned long long)a.b[0], (unsigned long long)a.b[1], (unsigned long long)a.b[2], (unsigned long long)a.b[3],
                     (unsigned long long)a.win, a.first, a.last, (unsigned long long)a.rmw, (unsigned long long)a.rmwmode, (unsigned long long)a.abs,
                     (unsigned long long)a.win_rmw, (unsigned long long)a.win_rmwmode, (unsigned long long)a.win_abs,
                     (unsigned long long)a.modechg, (unsigned long long)a.modechg_outvbl, a.lastold, a.lastnew, a.lastline);
        for (int i = 0; i < 263; i++) std::fprintf(j, "%s%llu", i ? "," : "", (unsigned long long)a.lines[i]);
        std::fprintf(j, "]}");
        f = false;
    } }
    auto evs = [&](const char* name, std::vector<WinEv>& v)
    {
        std::fprintf(j, "],\n\"%s\": [", name);
        for (size_t i = 0; i < v.size(); i++)
        {
            auto& e = v[i];
            std::fprintf(j, "%s\n {\"frame\": %ld, \"line\": %u, \"eng\": \"%c\", \"op\": \"%s\", \"size\": %d, \"off\": %d, \"pc\": \"%08x\", \"ctx\": \"%s\", \"old\": \"%08x\", \"new\": \"%08x\", \"cls\": \"%s\"}",
                         i ? "," : "", e.frame, e.line, e.eng ? 'B' : 'A', e.write ? "w" : "r", e.size, e.off, e.pc, ctxname(e.ctx), e.oldv, e.newv, e.cls);
        }
    };
    evs("dispcnt_window_events", winev);
    evs("dispcnt_mode_events", modeev);
    std::fprintf(j, "],\n\"regwrites\": [");
    { bool f = true;
    for (auto& [k, r] : ragg)
    {
        std::fprintf(j, "%s\n {\"addr\": \"%08x\", \"pc\": \"%08x\", \"ctx\": \"%s\", \"n\": %llu, \"frames\": %llu, \"lines\": [",
                     f ? "" : ",", k.addr, k.pc, ctxname(k.ctx), (unsigned long long)r.n, (unsigned long long)r.frames);
        for (int i = 0; i < 263; i++) std::fprintf(j, "%s%u", i ? "," : "", r.lines[i]);
        std::fprintf(j, "]}");
        f = false;
    } }
    std::fprintf(j, "],\n\"hbl_frames\": %llu, \"vc_frames\": %llu,\n\"hbl_lines\": [", (unsigned long long)hbl_frames, (unsigned long long)vc_frames);
    for (int i = 0; i < 263; i++) std::fprintf(j, "%s%llu", i ? "," : "", (unsigned long long)hbl_lines[i]);
    std::fprintf(j, "],\n\"vc_lines\": [");
    for (int i = 0; i < 263; i++) std::fprintf(j, "%s%llu", i ? "," : "", (unsigned long long)vc_lines[i]);
    std::fprintf(j, "],\n\"capture_enables\": %llu, \"capture_fifo\": %llu, \"capture_writes\": [", (unsigned long long)cap_en, (unsigned long long)cap_fifo);
    { bool f = true; for (auto& [k, n] : capw) { std::fprintf(j, "%s{\"capcnt\": \"%08x\", \"pc\": \"%08x\", \"n\": %llu}", f ? "" : ", ", k.first, k.second, (unsigned long long)n); f = false; } }
    std::fprintf(j, "],\n\"fifo_writes\": %llu, \"fifo_pc\": {", (unsigned long long)fifo_writes);
    { bool f = true; for (auto& [pc, n] : fifo_pc) { std::fprintf(j, "%s\"%08x\": %llu", f ? "" : ", ", pc, (unsigned long long)n); f = false; } }
    std::fprintf(j, "},\n\"dma_mode4\": %llu, \"dma_mode4_events\": [", (unsigned long long)dma4);
    for (size_t i = 0; i < dma4ev.size(); i++) std::fprintf(j, "%s{\"frame\": %ld, \"pc\": \"%08x\"}", i ? ", " : "", dma4ev[i].first, dma4ev[i].second);
    std::fprintf(j, "],\n\"irq_handler\": [");
    for (size_t i = 0; i < hs.size(); i++)
    {
        auto& s = hs[i];
        std::fprintf(j, "%s\n {\"frame\": %ld, \"dtcm\": \"%08x\", \"ptr\": \"%08x\", \"code\": \"", i ? "," : "", s.frame, s.dtcm, s.ptr);
        for (u8 b : s.code) std::fprintf(j, "%02x", b);
        std::fprintf(j, "\"}");
    }
    std::fprintf(j, "]\n}\n");
    std::fclose(j);
    nds.IOHook = nullptr; nds.LineHook = nullptr;
}
}
