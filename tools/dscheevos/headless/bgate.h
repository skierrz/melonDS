// Behaviour gate (--bgate PREFIX): what the game itself does, recorded so that two runs from the same
// savestate with the same input (DScheevos on vs the neutral reference) can be compared for identical
// game behaviour. Included once by main.cpp. See dscheevos tests/behaviour/README.md for the design.
//
// Outputs (never contents a person reads; the compare tool prints addresses and counts only):
//   PREFIX.meta        text: regions, exclusion rules, what was found (thread lists, stacks, idle thread)
//   PREFIX.snap        binary records: per runner frame end ("A", kind 0) and per VBlank interval at the
//                      game's quiescent instant ("Q", kind 1): 64-bit hashes of every 4 KB block of the
//                      game-visible memories, with the exclusions masked, and hashes of what each rule masked
//   PREFIX.frames.csv  per runner frame: ARM7 time at its start, screen hashes, melonDS lag flag, the keypad
//                      values the game read (ARM9 KEYINPUT, ARM7 KEYINPUT/EXTKEYIN)
//   PREFIX.io.csv      timing-sensitive register reads by game code (VCOUNT, DISPSTAT, timer counters) and IPC
//                      writes (IPCSYNC, FIFO send), consecutive repeats folded into a count
//
// Exclusions (derived in README.md; each one is applied identically to both runs and its masked bytes are still
// hashed separately so the compare tool can say what each rule hid):
//   0 hook       the 8 bytes of the game's OS_Halt that DScheevos patches (the patch itself)
//   1 stk9       each ARM9 SDK thread's stack below its stack pointer (dead: never read before written)
//   2 idle9      the ARM9 idle thread: its saved context and its whole stack (a stateless loop; in DScheevos runs
//                it is the thread our module runs on)
//   3 ctx9       the running ARM9 thread's saved context (stale: overwritten at its next switch-out, before any read)
//   4 irq9       ARM9 IRQ stack below SP_irq
//   5 svc9       ARM9 SVC stack below SP_svc
//   6 stk7, 7 ctx7, 8 irq7, 9 svc7: the same on the ARM7
// Not memory, so nothing to exclude in melonDS: cache lines (melonDS's cache model keeps tags only; RAM is always
// the coherent view).
namespace bg
{
NDS* nds = nullptr;
std::string prefix;
FILE* snap = nullptr;
FILE* frames = nullptr;
FILE* io = nullptr;
FILE* meta = nullptr;
u32 haltaddr = 0;           // the game's OS_Halt (patched or stock): found by the runner after a state load
u32 info9 = 0, info7 = 0;   // OSThreadInfo of each CPU (0 = not found)
u32 idle9 = 0;              // ARM9 idle thread: current thread at the first OS_Halt call
u32 irqtop9 = 0, irqtop7 = 0;   // IRQ stack tops (SP_irq seen outside IRQ mode)
int next9 = -1, next7 = -1, stk9 = -1, stk7 = -1;   // OSThread offsets found: next pointer, stack bounds pair
long vblanks = 0;
int armedQ = 0;             // 2: VBlank started, waiting for the game to acknowledge it (IF bit 0 cleared); 1: waiting for the idle thread
long qkey = -1;
long curframe = 0;
u32 modlo = 0x02D80000, modhi = 0x02F80000;
struct Dump { int kind; long key; std::string path; };
std::vector<Dump> dumps;
std::vector<u32> k9, k7; u32 k9n = 0, k7n = 0;
struct IoLast { int cpu = -1; u32 pc = 0, addr = 0, val = 0; int size = 0; bool w = false; u64 n = 0; long frame = 0; u64 ts = 0; } last;
u64 warned = 0;

enum { R_HOOK, R_STK9, R_IDLE9, R_CTX9, R_IRQ9, R_SVC9, R_STK7, R_CTX7, R_IRQ7, R_SVC7, NRULES };
const char* rulenames[NRULES] = { "hook", "stk9", "idle9", "ctx9", "irq9", "svc9", "stk7", "ctx7", "irq7", "svc7" };

struct Region { std::string name; u32 base; u32 size; u32 blk; u32 first; };
std::vector<Region> regions;
u32 nblocks = 0;
std::vector<std::vector<u8>> bufs;

static u64 mix(u64 h, const u8* p, size_t n)
{
    size_t i = 0;
    for (; i + 8 <= n; i += 8) { u64 w; std::memcpy(&w, p + i, 8); h ^= w; h *= 0x100000001b3ull; h ^= h >> 29; }
    for (; i < n; i++) { h ^= p[i]; h *= 0x100000001b3ull; }
    return h;
}

static u32 rd9(u32 a)
{
    auto& c = nds->ARM9; u32 v = 0;
    if ((a & c.DTCMMask) == c.DTCMBase) std::memcpy(&v, &c.DTCM[a & 0x3FFC], 4);
    else if (a < 0x02000000 && a < c.ITCMSize) std::memcpy(&v, &c.ITCM[a & 0x7FFC], 4);
    else if ((a >> 24) == 2) std::memcpy(&v, &nds->MainRAM[a & nds->MainRAMMask & ~3u], 4);
    return v;
}
static u32 rd7(u32 a)
{
    u32 v = 0;
    if ((a >> 24) == 2) std::memcpy(&v, &nds->MainRAM[a & nds->MainRAMMask & ~3u], 4);
    else if (a >= 0x03800000 && a < 0x04000000) std::memcpy(&v, &nds->ARM7WRAM[a & 0xFFFC], 4);
    else if ((a >> 24) == 3) v = nds->ARM7Read32(a & ~3u);
    return v;
}
static u32 rd(int c, u32 a) { return c == 9 ? rd9(a) : rd7(a); }
static bool ram9(u32 a) { auto& c = nds->ARM9; return (a >> 24) == 2 || (a & c.DTCMMask) == c.DTCMBase; }
static bool ram7(u32 a) { return (a >> 24) == 2 || (a >> 24) == 3; }
static bool ramc(int c, u32 a) { return a && !(a & 3) && (c == 9 ? ram9(a) : ram7(a)); }

// banked stack pointers (melonDS keeps the inactive banks in R_xxx)
static u32 sp_usr(ARM& c)
{
    switch (c.CPSR & 0x1F) { case 0x11: return c.R_FIQ[5]; case 0x12: return c.R_IRQ[0]; case 0x13: return c.R_SVC[0];
        case 0x17: return c.R_ABT[0]; case 0x1B: return c.R_UND[0]; default: return c.R[13]; }
}
static u32 sp_mode(ARM& c, u32 m)
{
    if ((c.CPSR & 0x1F) == m) return c.R[13];
    return m == 0x12 ? c.R_IRQ[0] : c.R_SVC[0];
}

struct Thr { u32 t, sp, lo, hi; };
// The SDK thread list of one CPU. OSThreadInfo: +4 current, +8 list; OSThread: context first (sp at +0x38),
// then state, next, ... and a stack-bounds pair. Offsets differ between CPUs/SDK versions: found once by checking
// that the list walk ends, contains the current thread, and that every thread's saved SP lies in its bounds.
static bool threads(int c, u32 info, u32& cur, std::vector<Thr>& out)
{
    out.clear(); cur = 0;
    if (!info) return false;
    cur = rd(c, info + 4);
    u32 list = rd(c, info + 8);
    int& nx = c == 9 ? next9 : next7; int& so = c == 9 ? stk9 : stk7;
    auto walk = [&](int off, std::vector<u32>& v) {
        v.clear(); bool hascur = false;
        for (u32 t = list; t; t = rd(c, t + off)) { if (!ramc(c, t) || v.size() > 64) return false; v.push_back(t); hascur |= t == cur; }
        return hascur && !v.empty();
    };
    std::vector<u32> ts;
    if (nx < 0) { for (int off : { 0x68, 0x4C, 0x6C, 0x48 }) if (walk(off, ts)) { nx = off; break; } if (nx < 0) return false; }
    else if (!walk(nx, ts)) return false;
    if (so < 0)
    {
        for (int o = nx + 0x10; o <= nx + 0x80 && so < 0; o += 4)
        {
            bool ok = true;
            for (u32 t : ts)
            {
                if (c == 9 && t == idle9) continue;   // the idle thread's saved SP may be our module's stack
                u32 a = rd(c, t + o), b = rd(c, t + o + 4), lo = std::min(a, b), hi = std::max(a, b), sp = rd(c, t + 0x38);
                if (hi - lo < 0x40 || hi - lo > 0x200000 || !ramc(c, lo) || sp < lo || sp > hi) { ok = false; break; }
            }
            if (ok) so = o;
        }
        if (so < 0) return false;
    }
    for (u32 t : ts)
    {
        u32 a = rd(c, t + so), b = rd(c, t + so + 4);
        out.push_back({ t, rd(c, t + 0x38), std::min(a, b), std::max(a, b) });
    }
    return true;
}

static void find_info()
{
    for (u32 b : { 0x027FFFA0u, 0x02FFFFA0u })
    {
        u32 i9 = rd9(b), i7 = rd9(b + 4);
        if (!info9 && ramc(9, i9) && ramc(9, rd9(i9 + 4))) info9 = i9;
        if (!info7 && ramc(7, i7) && ramc(7, rd7(i7 + 4))) info7 = i7;
    }
}

struct Ex { int rule; int cpu; u32 lo, hi; };

static void exclusions(std::vector<Ex>& ex, u32& flags)
{
    ex.clear();
    if (haltaddr) ex.push_back({ R_HOOK, 9, haltaddr, haltaddr + 8 });
    for (int c : { 9, 7 })
    {
        ARM& cpu = c == 9 ? (ARM&)nds->ARM9 : (ARM&)nds->ARM7;
        const int rSTK = c == 9 ? R_STK9 : R_STK7, rCTX = c == 9 ? R_CTX9 : R_CTX7, rIRQ = c == 9 ? R_IRQ9 : R_IRQ7, rSVC = c == 9 ? R_SVC9 : R_SVC7;
        std::vector<Thr> ts; u32 cur = 0;
        bool have = threads(c, c == 9 ? info9 : info7, cur, ts);
        if (!have) { flags |= c == 9 ? 4 : 8; }
        const bool inirq = (cpu.CPSR & 0x1F) == 0x12;
        u32 spirq = sp_mode(cpu, 0x12), spsvc = sp_mode(cpu, 0x13);
        u32& irqtop = c == 9 ? irqtop9 : irqtop7;
        if (!inirq) irqtop = std::max(irqtop, spirq);
        u32 ctxsize = c == 9 ? 0x64 : 0x48;
        if (c == 9 && (nds->ARM9.CPSR & 0x1F) == 0x12) flags |= 2;
        if (c == 9 && cur && cur == idle9) flags |= 1;
        for (auto& t : ts)
        {
            if (c == 9 && t.t == idle9)
            {
                ex.push_back({ R_IDLE9, 9, t.t, t.t + ctxsize });
                ex.push_back({ R_IDLE9, 9, t.lo, t.hi });
                continue;
            }
            u32 sp = t.t == cur ? sp_usr(cpu) : t.sp;
            if (t.t == cur) ex.push_back({ rCTX, c, t.t, t.t + ctxsize });
            if (sp >= t.lo && sp <= t.hi) ex.push_back({ rSTK, c, t.lo, sp });
            else flags |= 16;
        }
        // IRQ stack: from the next stack below it (the highest thread stack top under SP_irq, within 8 KB) up to SP_irq
        u32 irqlo = 0;
        for (auto& t : ts) if (t.hi <= spirq && spirq - t.hi <= 0x2000) irqlo = std::max(irqlo, t.hi);
        if (irqlo) ex.push_back({ rIRQ, c, irqlo, spirq }); else flags |= 32;
        // SVC stack: from the IRQ stack's top up to SP_svc
        if (irqtop && irqtop <= spsvc && spsvc - irqtop <= 0x400) ex.push_back({ rSVC, c, irqtop, spsvc }); else flags |= 64;
    }
}

static void add_region(const std::string& name, u32 base, u32 size, u32 blk)
{
    regions.push_back({ name, base, size, blk, nblocks });
    nblocks += (size + blk - 1) / blk;
}

static void setup_regions()
{
    auto& g = nds->GPU;
    const u32 ms = nds->MainRAMMask + 1;
    add_region("main", 0x02000000, 0x400000, 0x1000);
    if (ms > 0x800000) add_region("sys27", 0x027FF000, 0x1000, 0x1000);
    if (ms > 0x1000000 - 1) add_region("sys2F", 0x02FFF000, 0x1000, 0x1000);
    add_region("dtcm", nds->ARM9.DTCMBase, 0x4000, 0x1000);
    add_region("itcm", 0, 0x8000, 0x1000);
    add_region("swram", 0x03000000, 0x8000, 0x1000);
    add_region("wram7", 0x03800000, 0x10000, 0x1000);
    add_region("vram", 0x06800000, sizeof g.VRAM_A * 4 + sizeof g.VRAM_E + sizeof g.VRAM_F + sizeof g.VRAM_G + sizeof g.VRAM_H + sizeof g.VRAM_I, 0x1000);
    add_region("pal", 0x05000000, 0x800, 0x800);
    add_region("oam", 0x07000000, 0x800, 0x800);
    add_region("io", 0x04000000, 0x80, 4);   // one block per register word (see fill)
    add_region("other", 0x02000000, ms, 0x10000);   // main RAM outside the game's areas and the module: nds-bootstrap
    bufs.resize(regions.size());
}

static void fill(size_t r, std::vector<u8>& b)
{
    auto& g = nds->GPU; const auto& R = regions[r];
    b.assign(R.size, 0);
    auto cp = [&](u32 off, const u8* p, u32 n) { std::memcpy(&b[off], p, n); };
    if (R.name == "main") cp(0, nds->MainRAM, 0x400000);
    else if (R.name == "sys27") cp(0, &nds->MainRAM[0x7FF000 & nds->MainRAMMask], 0x1000);
    else if (R.name == "sys2F") cp(0, &nds->MainRAM[0xFFF000 & nds->MainRAMMask], 0x1000);
    else if (R.name == "dtcm") cp(0, nds->ARM9.DTCM, 0x4000);
    else if (R.name == "itcm") cp(0, nds->ARM9.ITCM, 0x8000);
    else if (R.name == "swram") cp(0, nds->SharedWRAM, 0x8000);
    else if (R.name == "wram7") cp(0, nds->ARM7WRAM, 0x10000);
    else if (R.name == "vram")
    {
        u32 o = 0;
        for (auto [p, n] : std::initializer_list<std::pair<const u8*, u32>>{ { g.VRAM_A, sizeof g.VRAM_A }, { g.VRAM_B, sizeof g.VRAM_B },
                 { g.VRAM_C, sizeof g.VRAM_C }, { g.VRAM_D, sizeof g.VRAM_D }, { g.VRAM_E, sizeof g.VRAM_E }, { g.VRAM_F, sizeof g.VRAM_F },
                 { g.VRAM_G, sizeof g.VRAM_G }, { g.VRAM_H, sizeof g.VRAM_H }, { g.VRAM_I, sizeof g.VRAM_I } })
        { cp(o, p, n); o += n; }
    }
    else if (R.name == "pal") cp(0, g.Palette, 0x800);
    else if (R.name == "oam") cp(0, g.OAM, 0x800);
    else if (R.name == "io")
    {
        // configuration registers only (no counters, no VCOUNT/IF: those depend on the instant, not on behaviour)
        u32 w[32] = {}; int k = 0;
        w[k++] = nds->IME[0]; w[k++] = nds->IE[0]; w[k++] = nds->IME[1]; w[k++] = nds->IE[1];
        w[k++] = nds->PowerControl9 | (u32)nds->ARM7Read16(0x04000304) << 16; w[k++] = nds->ARM9Read8(0x04000247);
        for (int i = 0; i < 9; i++) w[k + i / 4] |= (u32)g.VRAMCNT[i] << (8 * (i % 4));
        k += 3;
        w[k++] = g.GPU2D_A.DispCnt; w[k++] = g.GPU2D_B.DispCnt;
        for (int i = 0; i < 8; i++) w[k++] = nds->Timers[i].Cnt | (u32)nds->Timers[i].Reload << 16;
        for (int i = 0; i < 4; i++) { w[k++] = nds->ARM9Read32(0x040000B8 + 12 * i); w[k++] = nds->ARM7Read32(0x040000B8 + 12 * i); }
        cp(0, (const u8*)w, sizeof w);   // words: IME9 IE9 IME7 IE7 POWCNT WRAMCNT VRAMCNT*3 DISPCNT_A DISPCNT_B TM9_0-3 TM7_0-3 DMA9_0/DMA7_0 ... DMA9_3/DMA7_3
    }
    else if (R.name == "other")
    {
        const u32 ms = nds->MainRAMMask + 1;
        cp(0, nds->MainRAM, ms);
        std::memset(&b[0], 0, 0x400000);
        if (ms > 0x800000) std::memset(&b[0x7FF000], 0, 0x1000);
        if (ms > 0xFFFFFF) std::memset(&b[0xFFF000], 0, 0x1000);
        u32 mlo = (modlo - 0x02000000) & nds->MainRAMMask, mhi = std::min(ms, mlo + (modhi - modlo));
        if (mlo < ms) std::memset(&b[mlo], 0, mhi - mlo);
    }
}

// where a CPU address lands in the hashed regions: (region index, offset); -1 if nowhere
static std::pair<int, u32> locate(int c, u32 a)
{
    auto idx = [&](const char* n) { for (size_t i = 0; i < regions.size(); i++) if (regions[i].name == n) return (int)i; return -1; };
    if (c == 9 && (a & nds->ARM9.DTCMMask) == nds->ARM9.DTCMBase) return { idx("dtcm"), a & 0x3FFF };
    if ((a >> 24) == 2)
    {
        u32 o = a & nds->MainRAMMask;
        if (o < 0x400000) return { idx("main"), o };
        if ((o & ~0xFFFu) == 0x7FF000) return { idx("sys27"), o & 0xFFF };
        if ((o & ~0xFFFu) == 0xFFF000) return { idx("sys2F"), o & 0xFFF };
        return { idx("other"), o };
    }
    if (c == 7 && a >= 0x03800000 && a < 0x04000000) return { idx("wram7"), a & 0xFFFF };
    if ((a >> 24) == 3) return { idx("swram"), a & 0x7FFF };
    return { -1, 0 };
}

static void write_meta()
{
    if (!meta) return;
    std::fprintf(meta, "version 1\nblocks %u\n", nblocks);
    for (auto& r : regions) std::fprintf(meta, "region %s %08x %x %x %u\n", r.name.c_str(), r.base, r.size, r.blk, r.first);
    for (int i = 0; i < NRULES; i++) std::fprintf(meta, "rule %d %s\n", i, rulenames[i]);
    std::fprintf(meta, "module %08x %08x\n", modlo, modhi);
    std::fflush(meta);
}

static void snapshot(int kind, long key)
{
    if (!snap) return;
    if (regions.empty()) { setup_regions(); write_meta(); }
    if (!info9 || !info7) find_info();
    std::vector<Ex> ex; u32 flags = 0;
    exclusions(ex, flags);
    if (!idle9) flags |= 128;
    static int shown[2] = { 0, 0 };
    if (meta && shown[kind] < 3)
    {
        shown[kind]++;
        std::fprintf(meta, "record %c %ld frame %ld vcount %u flags %x cpsr9 %08x sp_usr9 %08x sp_irq9 %08x sp_svc9 %08x sp_irq7 %08x sp_svc7 %08x:", kind ? 'Q' : 'A', key, curframe,
                     nds->GPU.VCount, flags, nds->ARM9.CPSR, sp_usr(nds->ARM9), sp_mode(nds->ARM9, 0x12), sp_mode(nds->ARM9, 0x13), sp_mode(nds->ARM7, 0x12), sp_mode(nds->ARM7, 0x13));
        for (auto& e : ex) { auto [r, off] = locate(e.cpu, e.lo); std::fprintf(meta, " %s:%08x-%08x@%s", rulenames[e.rule], e.lo, e.hi, r < 0 ? "-" : regions[r].name.c_str()); }
        std::fprintf(meta, "\n");
    }
    u64 rh[NRULES], rr[NRULES]; u32 rb[NRULES];
    for (int i = 0; i < NRULES; i++) { rh[i] = rr[i] = 1469598103934665603ull; rb[i] = 0; }
    auto hook = nds->BgIoHook; nds->BgIoHook = nullptr;
    for (size_t r = 0; r < regions.size(); r++) fill(r, bufs[r]);
    nds->BgIoHook = hook;
    for (auto& e : ex)
    {
        u32 t[3] = { (u32)e.cpu, e.lo, e.hi }; rr[e.rule] = mix(rr[e.rule], (const u8*)t, sizeof t);
        if (e.hi <= e.lo) continue;
        auto [r, off] = locate(e.cpu, e.lo);
        if (r < 0) continue;
        u32 n = std::min<u32>(e.hi - e.lo, regions[r].size - off);
        rh[e.rule] = mix(rh[e.rule], &bufs[r][off], n);
        std::memset(&bufs[r][off], 0, n); rb[e.rule] += n;
    }
    std::vector<u64> blk(nblocks);
    for (size_t r = 0; r < regions.size(); r++)
    {
        auto& R = regions[r];
        for (u32 o = 0, i = R.first; o < R.size; o += R.blk, i++) blk[i] = mix(1469598103934665603ull, &bufs[r][o], std::min(R.blk, R.size - o));
    }
    u32 hdr[6] = { 0x31524742u, (u32)kind, (u32)key, (u32)curframe, nds->GPU.VCount, flags };
    std::fwrite(hdr, 4, 6, snap);
    std::fwrite(rh, 8, NRULES, snap); std::fwrite(rr, 8, NRULES, snap); std::fwrite(rb, 4, NRULES, snap);
    std::fwrite(blk.data(), 8, nblocks, snap);
    for (auto& d : dumps)
        if (d.kind == kind && d.key == key)
            if (FILE* f = std::fopen(d.path.c_str(), "wb")) { for (auto& b : bufs) std::fwrite(b.data(), 1, b.size(), f); std::fclose(f); }
}

static void flush_io()
{
    if (io && last.cpu >= 0)
        std::fprintf(io, "%ld,%d,%llu,%08x,%c,%08x,%d,%x,%llu\n", last.frame, last.cpu ? 7 : 9, (unsigned long long)last.ts, last.pc,
                     last.w ? 'w' : 'r', last.addr, last.size, last.val, (unsigned long long)last.n);
    last.cpu = -1;
}

static void io_hook(NDS& n, int cpu, u32 addr, u32 val, int size, bool write)
{
    const u32 a = addr & ~1u;
    bool keep;
    if (write) keep = a == 0x04000180 || (addr & ~3u) == 0x04000188;
    else
    {
        if (a == 0x04000130 || (cpu && a == 0x04000136))
        {
            auto& v = cpu ? k7 : k9; u32& cnt = cpu ? k7n : k9n; cnt++;
            u32 x = (val & 0xFFFF) | (a == 0x04000136 ? 0x10000 : 0);
            if (v.empty() || v.back() != x) { if (v.size() < 16) v.push_back(x); }
            return;
        }
        keep = a == 0x04000004 || a == 0x04000006 || (a >= 0x04000100 && a < 0x04000110 && !(a & 2));
    }
    if (!keep || !io) return;
    ARM& c = cpu ? (ARM&)n.ARM7 : (ARM&)n.ARM9;
    u32 pc = c.R[15] - ((c.CPSR & 0x20) ? 4 : 8);
    if (!cpu && pc >= modlo && pc < modhi) return;   // our own reads (the module) are not the game's
    if (last.cpu == cpu && last.pc == pc && last.addr == addr && last.val == val && last.w == write && last.frame == curframe) { last.n++; return; }
    flush_io();
    last = { cpu, pc, addr, val, size, write, 1, curframe, cpu ? n.ARM7Timestamp : (n.ARM9Timestamp >> n.ARM9ClockShift) };
}

static void insn_hook(ARMv5& cpu, u32 addr, u32 cpsr)
{
    const bool irq = (cpsr & 0x1F) == 0x12;
    if (!info9) find_info();
    if (!idle9 && haltaddr && addr == haltaddr && !irq && info9)
    {
        idle9 = rd9(info9 + 4);
        if (meta) { std::fprintf(meta, "idle9 %08x (first OS_Halt call at frame %ld)\n", idle9, curframe); std::fflush(meta); }
    }
    if (armedQ == 2 && !(nds->IF[0] & 1)) armedQ = 1;
    if (armedQ == 1 && idle9 && !irq && rd9(info9 + 4) == idle9 && (rd9(info9) >> 16) == 0)
    {
        armedQ = 0;
        snapshot(1, qkey);
    }
    cpu.BgArmed = armedQ || !idle9;
}

static void on_vblank(ARMv5& cpu)
{
    if (!snap) return;
    vblanks++;
    qkey = vblanks; armedQ = 2;
    cpu.BgArmed = true;
}

static void frame_begin(long f)
{
    curframe = f;
    k9.clear(); k7.clear(); k9n = k7n = 0;
    if (frames) std::fprintf(frames, "%ld,%llu", f, (unsigned long long)nds->ARM7Timestamp);
}

static void frame_end(long f)
{
    if (!frames && !snap) return;
    if (frames)
    {
        auto fnv = [](u64 h, const void* p, size_t n) { const u8* b = (const u8*)p; for (size_t i = 0; i < n; i++) { h ^= b[i]; h *= 1099511628211ull; } return h; };
        void* top; void* bot; u64 ht = 0, hb = 0;
        if (nds->GPU.GetFramebuffers(&top, &bot)) { ht = fnv(1469598103934665603ull, top, 256 * 192 * 4); hb = fnv(1469598103934665603ull, bot, 256 * 192 * 4); }
        std::fprintf(frames, ",%016llx,%016llx,%d,%u,", (unsigned long long)ht, (unsigned long long)hb, nds->LagFrameFlag ? 1 : 0, k9n);
        for (size_t i = 0; i < k9.size(); i++) std::fprintf(frames, "%s%x", i ? "/" : "", k9[i]);
        std::fprintf(frames, ",%u,", k7n);
        for (size_t i = 0; i < k7.size(); i++) std::fprintf(frames, "%s%x", i ? "/" : "", k7[i]);
        std::fprintf(frames, "\n");
    }
    snapshot(0, f);
}

static bool open(const std::string& p)
{
    prefix = p;
    snap = std::fopen((p + ".snap").c_str(), "wb");
    frames = std::fopen((p + ".frames.csv").c_str(), "w");
    io = std::fopen((p + ".io.csv").c_str(), "w");
    meta = std::fopen((p + ".meta").c_str(), "w");
    if (!snap || !frames || !io || !meta) return false;
    std::fprintf(frames, "frame,ts7,screen_top,screen_bottom,lagflag,keyreads9,keys9,keyreads7,keys7\n");
    std::fprintf(io, "frame,cpu,ts,pc,rw,addr,size,value,count\n");
    return true;
}

static void close()
{
    flush_io();
    if (meta)
    {
        std::fprintf(meta, "info9 %08x info7 %08x next9 %x next7 %x stack9 %x stack7 %x irqtop9 %08x irqtop7 %08x halt %08x\n",
                     info9, info7, next9, next7, stk9, stk7, irqtop9, irqtop7, haltaddr);
        for (int c : { 9, 7 })
        {
            std::vector<Thr> ts; u32 cur = 0;
            if (threads(c, c == 9 ? info9 : info7, cur, ts))
                for (auto& t : ts) std::fprintf(meta, "thread%d %08x stack %08x-%08x sp %08x%s%s\n", c, t.t, t.lo, t.hi, t.sp,
                                                t.t == cur ? " current" : "", c == 9 && t.t == idle9 ? " idle" : "");
        }
    }
    for (FILE** f : { &snap, &frames, &io, &meta }) if (*f) { std::fclose(*f); *f = nullptr; }
}
}
