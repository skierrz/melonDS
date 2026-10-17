/*
    Copyright 2016-2026 melonDS team

    This file is part of melonDS.

    melonDS is free software: you can redistribute it and/or modify it under
    the terms of the GNU General Public License as published by the Free
    Software Foundation, either version 3 of the License, or (at your option)
    any later version.

    melonDS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with melonDS. If not, see http://www.gnu.org/licenses/.
*/

#ifndef ARM_H
#define ARM_H

#include <algorithm>
#include <cstdio>
#include <optional>
#include <map>
#include <array>
#include <tuple>
#include <vector>
#include <unordered_map>

#include "types.h"
#include "MemRegion.h"
#include "MemConstants.h"

#ifdef GDBSTUB_ENABLED
#include "debug/GdbStub.h"
#endif

namespace melonDS
{
inline u32 ROR(u32 x, u32 n)
{
    return (x >> (n&0x1F)) | (x << ((32-n)&0x1F));
}

enum
{
    RWFlags_Nonseq = (1<<5),
    RWFlags_ForceUser = (1<<21),
};

enum class CPUExecuteMode : u32
{
    Interpreter,
    InterpreterGDB,
#ifdef JIT_ENABLED
    JIT
#endif
};

struct GDBArgs;
class ARMJIT;
class GPU;
class ARMJIT_Memory;
class NDS;
class Savestate;

class ARM
#ifdef GDBSTUB_ENABLED
    : public Gdb::StubCallbacks
#endif
{
public:
    ARM(u32 num, bool jit, std::optional<GDBArgs> gdb, NDS& nds);
    virtual ~ARM(); // destroy shit

    void SetGdbArgs(std::optional<GDBArgs> gdb);

    virtual void Reset();

    virtual void DoSavestate(Savestate* file);

    virtual void FillPipeline() = 0;

    virtual void JumpTo(u32 addr, bool restorecpsr = false) = 0;
    void RestoreCPSR();

    void Halt(u32 halt)
    {
        if (halt==2 && Halted==1) return;
        Halted = halt;
    }

    void NocashPrint(u32 addr) noexcept;

    bool CheckCondition(u32 code) const
    {
        if (code == 0xE) return true;
        if (ConditionTable[code] & (1 << (CPSR>>28))) return true;
        return false;
    }

    void SetC(bool c)
    {
        if (c) CPSR |= 0x20000000;
        else CPSR &= ~0x20000000;
    }

    void SetNZ(bool n, bool z)
    {
        CPSR &= ~0xC0000000;
        if (n) CPSR |= 0x80000000;
        if (z) CPSR |= 0x40000000;
    }

    void SetNZCV(bool n, bool z, bool c, bool v)
    {
        CPSR &= ~0xF0000000;
        if (n) CPSR |= 0x80000000;
        if (z) CPSR |= 0x40000000;
        if (c) CPSR |= 0x20000000;
        if (v) CPSR |= 0x10000000;
    }

    inline bool ModeIs(u32 mode) const
    {
        u32 cm = CPSR & 0x1f;
        mode &= 0x1f;

        if (mode == cm) return true;
        if (mode == 0x17) return cm >= 0x14 && cm <= 0x17; // abt
        if (mode == 0x1b) return cm >= 0x18 && cm <= 0x1b; // und

        return false;
    }

    void UpdateMode(u32 oldmode, u32 newmode, bool phony = false);

    void TriggerIRQ();

    void SetupCodeMem(u32 addr);


    virtual void DataRead8(u32 addr, u32* val) = 0;
    virtual void DataRead16(u32 addr, u32* val) = 0;
    virtual void DataRead32(u32 addr, u32* val) = 0;
    virtual void DataRead32S(u32 addr, u32* val) = 0;
    virtual void DataWrite8(u32 addr, u8 val) = 0;
    virtual void DataWrite16(u32 addr, u16 val) = 0;
    virtual void DataWrite32(u32 addr, u32 val) = 0;
    virtual void DataWrite32S(u32 addr, u32 val) = 0;

    virtual void AddCycles_C() = 0;
    virtual void AddCycles_CI(s32 numI) = 0;
    virtual void AddCycles_CDI() = 0;
    virtual void AddCycles_CD() = 0;

    void CheckGdbIncoming();

    u32 Num;

    s32 Cycles;
    union
    {
        struct
        {
            u8 Halted;
            u8 IRQ; // nonzero to trigger IRQ
            u8 IdleLoop;
        };
        u32 StopExecution;
    };

    u32 CodeRegion;
    s32 CodeCycles;

    u32 DataRegion;
    s32 DataCycles;

    u32 R[16]; // heh
    u32 CPSR;
    u32 R_FIQ[8]; // holding SPSR too
    u32 R_SVC[3];
    u32 R_ABT[3];
    u32 R_IRQ[3];
    u32 R_UND[3];
    u32 CurInstr;
    u32 NextInstr[2];

    u32 ExceptionBase;

    MemRegion CodeMem;

#ifdef JIT_ENABLED
    u32 FastBlockLookupStart, FastBlockLookupSize;
    u64* FastBlockLookup;
#endif

    static const u32 ConditionTable[16];
#ifdef GDBSTUB_ENABLED
    Gdb::GdbStub GdbStub;
#endif

    melonDS::NDS& NDS;
protected:
    virtual u8 BusRead8(u32 addr) = 0;
    virtual u16 BusRead16(u32 addr) = 0;
    virtual u32 BusRead32(u32 addr) = 0;
    virtual void BusWrite8(u32 addr, u8 val) = 0;
    virtual void BusWrite16(u32 addr, u16 val) = 0;
    virtual void BusWrite32(u32 addr, u32 val) = 0;

#ifdef GDBSTUB_ENABLED
    bool IsSingleStep;
    bool BreakReq;
    bool BreakOnStartup;
    u16 Port;

public:
    int GetCPU() const override { return Num ? 7 : 9; }

    u32 ReadReg(Gdb::Register reg) override;
    void WriteReg(Gdb::Register reg, u32 v) override;
    u32 ReadMem(u32 addr, int size) override;
    void WriteMem(u32 addr, int size, u32 v) override;

    void ResetGdb() override;
    int RemoteCmd(const u8* cmd, size_t len) override;

protected:
#endif

    void GdbCheckA();
    void GdbCheckB();
    void GdbCheckC();
};

class ARMv5 : public ARM
{
public:
    ARMv5(melonDS::NDS& nds, std::optional<GDBArgs> gdb, bool jit);
    ~ARMv5();

    void Reset() override;

    void DoSavestate(Savestate* file) override;

    void UpdateRegionTimings(u32 addrstart, u32 addrend);

    void FillPipeline() override;

    void JumpTo(u32 addr, bool restorecpsr = false) override;

    void PrefetchAbort();
    void DataAbort();

    template <CPUExecuteMode mode>
    void Execute();

    // all code accesses are forced nonseq 32bit
    u32 CodeRead32(u32 addr, bool branch);

    void DataRead8(u32 addr, u32* val) override;
    void DataRead16(u32 addr, u32* val) override;
    void DataRead32(u32 addr, u32* val) override;
    void DataRead32S(u32 addr, u32* val) override;
    void DataWrite8(u32 addr, u8 val) override;
    void DataWrite16(u32 addr, u16 val) override;
    void DataWrite32(u32 addr, u32 val) override;
    void DataWrite32S(u32 addr, u32 val) override;

    void AddCycles_C() override
    {
        // code only. always nonseq 32-bit for ARM9.
        s32 numC = (R[15] & 0x2) ? ThumbOddCycle : CodeCycles;
        Cycles += numC;
    }

    void AddCycles_CI(s32 numI) override
    {
        // code+internal
        s32 numC = (R[15] & 0x2) ? ThumbOddCycle : CodeCycles;
        Cycles += numC + numI;
    }

    void AddCycles_CDI() override
    {
        // LDR/LDM cycles. ARM9 seems to skip the internal cycle there.
        // TODO: ITCM data fetches shouldn't be parallelized, they say
        s32 numC = (R[15] & 0x2) ? ThumbOddCycle : CodeCycles;
        s32 numD = DataCycles;

        //if (DataRegion != CodeRegion)
            Cycles += std::max(numC + numD - 6, std::max(numC, numD));
        //else
        //    Cycles += numC + numD;
    }

    void AddCycles_CD() override
    {
        // TODO: ITCM data fetches shouldn't be parallelized, they say
        s32 numC = (R[15] & 0x2) ? ThumbOddCycle : CodeCycles;
        s32 numD = DataCycles;

        //if (DataRegion != CodeRegion)
            Cycles += std::max(numC + numD - 6, std::max(numC, numD));
        //else
        //    Cycles += numC + numD;
    }

    void GetCodeMemRegion(u32 addr, MemRegion* region);

    void CP15Reset();
    void CP15DoSavestate(Savestate* file);

    void UpdateDTCMSetting();
    void UpdateITCMSetting();

    void UpdatePURegion(u32 n);
    void UpdatePURegions(bool update_all);

    u32 RandomLineIndex();

    void ICacheLookup(u32 addr);
    void ICacheInvalidateByAddr(u32 addr);
    void ICacheInvalidateAll();

    // Optional timing-only cache model (off by default). Tracks which lines
    // each cache holds so hits and misses cost what they would on hardware;
    // it never changes the data the CPU sees.
    struct CacheModelStats
    {
        u64 IHit, IMiss, DReadHit, DReadMiss, DWriteHit, DWriteMiss, DWriteBack;
    };
    bool CacheModel = false;
    s32 ThumbOddCycle = 0;               // cycles for a Thumb instruction in a fetched word's upper half
    CacheModelStats CacheStats {};
    void CacheModelReset();
    s32 CacheModelCode(u32 addr);
    s32 CacheModelData(u32 addr, bool write, bool writeback, bool seq, s32 uncached);
    s32 CacheModelCodeReal(u32 addr);
    s32 CacheModelDataReal(u32 addr, bool write, bool writeback, bool seq, s32 uncached);
    void CacheModelDInvalidateLine(u32 addr, bool clean);
    void CacheModelDInvalidateSetWay(u32 val, bool clean);
    void CacheModelDInvalidateAll();
    s32 DataCacheCycles(u32 addr, bool write, bool seq, s32 table);

    // Optional profile (off by default; a frontend sets ProfOn and the
    // module's addresses). Per VBlank interval: ARM9 cycles with the PC in
    // the module [ProfLo, ProfHi), split by context: the VBlank call (PC in
    // [ProfVBlankLo, ProfVBlankHi), the entry stub, or SP on the module's IRQ
    // stack [ProfIrqStackLo, ProfIrqStackHi]) and everything else (the idle
    // thread); module cycles in CPSR IRQ mode, whatever the context; and how
    // often the game's idle thread arrived at idle. An arrival is a halt
    // outside IRQ mode and outside the VBlank call, or reaching module idle
    // code from anywhere else (OS_Halt's jump, or an IRQ returning into
    // preempted idle work). Cycles are in 133 MHz units, whatever the clock.
    struct ProfileStats
    {
        u64 VBlank, Idle, IRQMode, Busy;   // this interval
        u32 IdleArrivals;                  // this interval
        u32 Halts;                         // game OS_Halt calls seen (all time)
        u64 Halted, HaltedBios;            // this interval: cycles halted (Halted == 1), and the part
                                           // where the halt came from the BIOS (PC >= 0xFFFF0000)
        // the GAME (PC outside the module), this interval: cycles outside / in CPSR IRQ mode,
        // instructions, and with the cache model its own cache misses (code fetches outside the
        // module, data accesses by instructions outside it)
        u64 GameCycles, GameIRQCycles, GameInstr;
        u64 GameIMiss, GameDReadMiss, GameDWriteMiss, GameDWriteBack;
        // shadow cache (see CacheShadow): what the game's accesses would have cost had the module
        // never touched the caches. Extra = real cost - shadow cost (cycles, 133 MHz units);
        // Poll = real miss where the shadow hit, Rev = real hit where the shadow missed
        s64 GameIExtra, GameDExtra;
        u64 PollI, PollD, PollWB, RevI, RevD;
        s64 GameExtraIRQ;                  // the part of GameIExtra + GameDExtra in CPSR IRQ mode
        u32 Resumes;                       // the game ran right after module code (IRQ or return)
        u32 IRQs;                          // stretches of CPSR IRQ mode (IRQ handler runs)
        u64 IRQMax;                        // the longest one (cycles, 133 MHz units, module included)
        // module code outside IRQ mode with CPSR.I set: all of it, the part with an IRQ waiting
        // (IME on, IE & IF: the game's IRQ is held off), and the longest such wait
        u64 Masked, MaskedPend, MaskedPendMax;
        u32 Swi5;                          // SWI 5 (VBlankIntrWait) calls this interval: the BIOS-wait game's frames
    };
    u64 ProfMaskRun = 0;                   // the current wait so far
    // --trace-wait (LAB coverage): one CSV line per event of a BIOS-wait game's frame: G gate entry
    // (PC = ProfGateAddr), S SWI 5, R its return, I VBlank IRQ taken, H OS_Halt; V (runner) each VBlank
    FILE* ProfWaitLog = nullptr;
    u32 ProfGateAddr = 0, ProfWaitRet = 0, ProfFrame = 0;
    void ProfWaitEvent(char kind, u32 addr);
    u64 ProfIRQRun = 0;                    // the current IRQ-mode stretch so far
    bool ProfPrevIRQ = false;
    bool ProfHaltBios = false;             // the current halt started in the BIOS
    void ProfHalted(u64 cycles);           // ARM9 cycles skipped while halted
    bool ProfOn = false;
    u32 ProfLo = 0, ProfHi = 0;            // module image
    u32 ProfVBlankLo = 0, ProfVBlankHi = 0;
    u32 ProfIrqStackLo = 0, ProfIrqStackHi = 0;
    u32 ProfHaltEntry = 0;                 // module entry that OS_Halt jumps to (0: none)
    ProfileStats Prof {};
    bool ProfInIdle = false;               // last instruction outside IRQ mode was module idle code
    u32 ProfPrevInstr = 0;
    void (*ProfVBlank)(ARMv5& cpu) = nullptr;  // called at the start of each VBlank
    // behaviour gate (runner --bgate): while BgArmed, BgHook is called before each ARM9 instruction (address, CPSR)
    bool BgArmed = false;
    void (*BgHook)(ARMv5& cpu, u32 addr, u32 cpsr) = nullptr;
    void ProfileInstr(u32 addr, u32 cpsr);
    // Optional, with the cache model: the VBlank call's instructions by PC
    // (cycles, runs, I-cache misses, D-cache read/write misses) and its data
    // misses by address (bit 0: write). A frontend sets the maps.
    struct ProfPCStat { u64 Cycles, Count, IMiss, DReadMiss, DWriteMiss, MaskedPend; };
    std::map<u32, ProfPCStat>* ProfPCs = nullptr;
    std::map<u32, u64>* ProfDMiss = nullptr;
    std::map<u32, u64>* ProfIdleDMiss = nullptr;   // --profile-idle-dmiss: module idle-path misses per address
    // Optional: the module's idle work (PC in the module, not the VBlank
    // call) by PC, indexed by (addr - ProfLo) / 2: what CPI the evaluation
    // gets (cycles per instruction, cache misses). A frontend sizes it.
    std::vector<ProfPCStat>* ProfIdlePCs = nullptr;
    // Shadow caches (--profile with the cache model): the same caches fed only the game's
    // accesses (code fetches outside [ProfLo, ProfHi), data accesses and cache maintenance by
    // instructions outside it), with their own replacement counter. Without a module the
    // shadow equals the real model; with one, the difference is the game's cost of sharing
    // the caches with the module. Optional: the game's extra cycles by PC (code: the fetched
    // line; data: the instruction) for attribution.
    struct CacheShadow
    {
        u32 ITags[64*4]; u8 ICount[64];
        u32 DTags[32*4]; u8 DCount[32];
        u32 LastDLine; u32 Seed;
    };
    CacheShadow Shadow {};
    bool ProfCurMod = false;               // the instruction executing is module code
    bool ProfPrevIn = false;               // the last instruction was module code
    // what-if (timing study only): module code fetches / module data accesses use only the cache
    // sets in this mask (0: all), as if the module's hot code/data were laid out in those sets
    u32 WhatIfISets = 0, WhatIfDSets = 0;
    u32 ProfCurPC = 0;
    struct ProfGamePC { s64 IExtra, DExtra; u64 PollI, PollD; };
    std::map<u32, ProfGamePC>* ProfGamePCs = nullptr;
    // busyloop lab: --hist9, ARM9 cycles by PC (outside / in CPSR IRQ mode) while set; --range9, per
    // interval, non-IRQ cycles with the PC in [Lo, Hi) or, once there, in [SubLo, SubHi) until the PC
    // leaves both (a spin loop and the poll it calls); First/Last: VCOUNT at the first entry / last exit.
    std::map<u32, std::array<u64, 2>>* Hist9 = nullptr;
    struct Range9 { u32 Lo, Hi, SubLo, SubHi; bool In; u64 Cyc, Entries; u32 First, Last; };
    std::vector<Range9> Ranges9;
    // gsdd lab: --pass9 ENTRY:RET:WORK,... classifies each call of a function (entered at ENTRY, back at
    // RET on the same thread, judged by SP) as work (a WORK pc ran inside it) or empty; per interval:
    // calls, empty calls, non-IRQ cycles in empty / work calls, the longest stretch of back-to-back empty
    // calls on that thread (cycles, broken by a work call or another thread), VCOUNT of the first / last
    // empty call's end. --count9 PC: per interval, non-IRQ executions of PC.
    struct Pass9 { u32 Entry, Ret; std::vector<u32> Work; bool In, Worked, StretchOn; u32 SP; u64 CurCyc;
                   u64 Calls, Empty, CycEmpty, CycWork, StretchRun, StretchDone, MaxStretch; u32 FirstV, LastV; };
    std::vector<Pass9> Passes9;
    std::vector<std::pair<u32, u64>> Counts9;
    // gsdd lab: --sp9 LO:HI: per interval, non-IRQ cycles with SP in [LO, HI) (one thread's stack)
    struct Sp9 { u32 Lo, Hi; u64 Cyc; };
    std::vector<Sp9> Sps9;
    // gsdd lab: --access9 LO:SIZE (repeatable): ARM9 data reads / writes into the ranges by pc (key: pc,
    // range, write, IRQ mode); for main-RAM writes also how many changed the stored value.
    struct Acc9Key { u32 PC; u8 Range, Write, IRQ; bool operator<(const Acc9Key& o) const
        { return std::tie(PC, Range, Write, IRQ) < std::tie(o.PC, o.Range, o.Write, o.IRQ); } };
    struct Acc9Val { u64 N, Changed; u32 Lo, Hi; u32 FirstF, LastF, LastVal; };
    std::vector<std::pair<u32, u32>> Acc9Ranges;
    u32 Acc9Lo = 0, Acc9Span = 0;
    std::map<Acc9Key, Acc9Val> Acc9;
    void Acc9Hit(u32 addr, int size, bool write, u32 val);
    // gsdd lab: --stall9 PC:UNIT:PERFRAME: emulate a preemptible gate at PC (non-IRQ mode): each arrival at
    // PC, while the interval's budget lasts and every --stall9-zero word reads 0, the ARM9 burns UNIT cycles
    // there in 64-cycle steps (IRQs are taken between steps, so threads switch as they would) before it
    // runs the instruction. It executes nothing and writes nothing: only time passes.
    u32 StallPC = 0; u64 StallUnit = 0, StallPerFrame = 0, StallLeft = 0, StallUsed = 0, StallUnits = 0, StallTotal = 0;
    bool StallBusy = false;
    u64 StallCheck = 0, StallSinceCheck = 0, StallAborts = 0;   // --stall9-check N: re-test the zero words every N cycles; abort the unit if one is set
    std::vector<u32> StallZero;
    std::vector<std::pair<u32, u32>> StallEq;   // dkr lab: --stall9-eq A:B, only while word A == word B
    bool StallOK();
    bool Irq9On = false; u64 Irq9[32] {}; u64 Irq9Last = ~0ull;   // --irq9-csv: IRQ vector entries per interval by IF&IE bit
    u64 StallStep = 64;   // --stall9-step N: cycles per burn step (IRQs are taken between steps)
    // audioid lab (research only): what the module's presence does to the game's timeline.
    // DscCache 1: module code fetches and data accesses (PC in the module) never look up or fill the cache
    //   model and cost 1 cycle (an idealised cache-neutral module); 2: they bypass the cache and cost the bus
    //   (what running the module uncached would cost it; the game's cache is untouched either way).
    // DscIdeal 1: while the PC is in the module (not IRQ mode), an instruction never runs past the slice
    //   target (the excess is dropped, so the system and the ARM7 see the same slice ends as with a halted
    //   ARM9), and a pending enabled IRQ is taken before the next module instruction, as a halted ARM9 takes it;
    //   2: also when the module has IRQs masked (CPSR.I), returning to the masked state.
    int DscCache = 0, DscIdeal = 0, DscIdealLat = 0; bool DscNull = false;   // DscNull: the module never runs: its halt entry halts and returns at once   // DscIdealLat: cycles added at such an IRQ (a stock halting MCR leaves its own cost pending until the wake)
    u64 DscIrqIdeal = 0, DscIrqMasked = 0, DscClampCyc = 0, DscClampN = 0;
    u64 StallVbl = 0, StallVblUsed = 0, StallVblN = 0;   // dkr lab: --stall9-vbl N: burn N cycles at the IRQ vector when a VBlank IRQ is taken (IRQ time, like a VBlank hook)
    // dkr lab: --thr9 INFO: SDK thread accounting. The current thread is the word at INFO+4 (OSThreadInfo.current);
    // a change of it is a thread switch (counted as an IRQ switch when it happens in IRQ mode). Non-IRQ cycles are
    // charged to the current thread.
    struct Thr9 { u64 Cyc = 0, Ins = 0, IrqIns = 0; };
    u32 Thr9Info = 0, Thr9Cur = 0; u64 Thr9Sw = 0, Thr9SwIrq = 0, Thr9IrqCyc = 0;
    std::map<u32, Thr9> Thr9Stats;
    FILE* Thr9Log = nullptr; u32 Thr9LastPC = 0, Thr9LastLR = 0;   // --thr9-log FILE: one line per switch
    // dkr lab: --callers9 PC: executions of PC by (lr, current thread), summary at exit
    std::vector<u32> Call9PCs;
    std::map<std::tuple<u32, u32, u32>, u64> Call9;
    // dkr lab: --ev9 FILE --ev9-pc PC (repeatable) --ev9-hash LO:LEN (repeatable): one line per execution of PC
    // (any mode): frame, ARM9 and ARM7 timestamps, pc, thread, r0-r3, lr, SPU channel 1/3 and capture 0 positions,
    // hashes of the main-RAM ranges
    FILE* Ev9Log = nullptr; std::vector<u32> Ev9PCs; std::vector<std::pair<u32, u32>> Ev9Hash;
    std::vector<u32> Ev9Words; std::vector<std::tuple<u32, u32, u32>> Ev9HashP;   // --ev9-word ADDR, --ev9-hashp PTR:OFF:LEN
    void Ev9Event(u32 addr, u32 cpsr);
    void ShadowReset();
    bool ProfInModule(u32 addr) const { return addr >= ProfLo && addr < ProfHi; }
    // Optional: the idle work by role (--profile-idle-roles). ProfRoleOf maps a
    // function entry ((addr - ProfLo) / 2) to a role (0: none); a call to one
    // pushes a shadow frame (its return address) and its cycles, and those of
    // whatever it calls that has no role of its own, go to that role until the
    // return address runs. Only on the module's idle stack (one pass at a time):
    // other module code in idle (halt_entry before the stack switch) is role 1.
    // ProfRoles[r]: cycles, instructions, cache misses; Count: calls.
    std::vector<u16>* ProfRoleOf = nullptr;
    std::vector<ProfPCStat>* ProfRoles = nullptr;
    std::vector<u64>* ProfRoleCalls = nullptr;
    u32 ProfIdleStackLo = 0, ProfIdleStackHi = 0;
    struct ProfRoleFrame { u32 Ret, Fn; u16 Role; };
    ProfRoleFrame ProfRoleStack[64] {};
    u32 ProfRoleDepth = 0;
    // --profile-footprint: the distinct cache lines (and sets) module idle code touches per
    // stretch (module code between two game instructions, i.e. per wake or per resumed part of a
    // pass): code lines fetched, data lines read or written-hit through the D-cache (a write miss
    // allocates nothing). Purely observational. FpHot: a stretch that runs code at or above it
    // (the hot region: rcheevos, the core's pass) is a "pass" stretch, else "light".
    struct FpStat { u64 Stretches, ILines, ISets, DOwnLines, DOwnSets, DGameLines, DGameSets, DSets; };
    bool FpOn = false, FpActive = false, FpPass = false;
    u32 FpHot = 0, FpStretch = 1;
    u32 FpLightLo = 0, FpLightHi = 0;               // code in the hot region that light wakes run (not a pass)
    FpStat Fp[2] {};
    std::unordered_map<u32, u32> FpSeen;              // key -> last stretch that touched it
    std::vector<u32> FpCur;                           // keys this stretch (line | 1 for data)
    std::map<u32, std::pair<u64, u64>>* FpLines = nullptr;  // key -> stretches (light, pass)
    void FpTouch(u32 key)
    {
        u32& s = FpSeen[key];
        if (s != FpStretch) { s = FpStretch; FpCur.push_back(key); }
    }
    void FpEnd();
    u32 ProfPend[8] {};
    u32 ProfPendN = 0;
    CacheModelStats ProfPrevStats {};

    void CP15Write(u32 id, u32 val);
    u32 CP15Read(u32 id) const;

    u32 CP15Control;

    u32 RNGSeed;
    u32 TraceProcessID;

    u32 DTCMSetting, ITCMSetting;

    // for aarch64 JIT they need to go up here
    // to be addressable by a 12-bit immediate
    u32 ITCMSize;
    u32 DTCMBase, DTCMMask;
    s32 RegionCodeCycles;

    u8 ITCM[ITCMPhysicalSize];
    u8* DTCM;

    u8 ICache[0x2000];
    u32 ICacheTags[64*4];
    u8 ICacheCount[64];

    // cache model state: line address | 1 (valid) | 2 (dirty, data cache only)
    u32 CMITags[64*4];
    u8 CMICount[64];
    u32 CMDTags[32*4];
    u8 CMDCount[32];
    u32 CMLastDLine;
    // ARM946E-S cache lockdown (CP15 c9,c0,0 D / c9,c0,1 I): bits 1:0 lockdown base (ways below it are
    // never refilled; hits still occur), bit 31 load (refills go to the base way). Footprint lab.
    u32 CMDLockdown = 0, CMILockdown = 0;
    u64 LockedGameFetch = 0, LockedGameIRQFetch = 0;
    bool LockModOnly = false;
    // --dsc-unlock-on-irq (approach A model): every IRQ entry clears the lockdown, as a stub at the
    // game's IRQ handler entry would; it is put back only when the IRQ returns straight to module code
    // (our code relocks when it runs again); a game thread switched in first runs unlocked
    bool UnlockOnIRQ = false;
    // --dsc-unlock-on-switch (approach A at the thread switch): the lockdown stays through IRQ handlers and is
    // cleared when NitroSDK's current thread (OSThreadInfo.current, found from the pointer at 0x02FFFFA0 or
    // 0x027FFFA0) stops being the thread that set it, as a patch in the SDK's thread switch would; our code
    // relocks when it runs again outside IRQ mode
    bool UnlockOnSwitch = false;
    u32 LockThrInfo = 0, LockThread = 0;
    u64 SwitchUnlocks = 0;
    u32 UnlockIRQMask = 0;   // --dsc-unlock-irq-mask M: with the switch model, an IRQ entry with (IE & IF & M) also unlocks
    u64 IRQLockedTaken[32] = {}, IRQMaskUnlocks = 0;   // IRQs taken under a lockdown, by IE & IF bit
    u32 LockCurThread();
    inline void LockSwitchCheck()
    {
        if (UnlockOnSwitch && (CMDLockdown | CMILockdown) && LockCurThread() != LockThread)
        {
            SavedDLock = CMDLockdown; SavedILock = CMILockdown;
            CMDLockdown = CMILockdown = 0;
            SwitchUnlocks++;
        }
    }
    bool LockIgnore = false;   // --dsc-lock-ignore: the cache model ignores every lockdown (same-binary control)
    u32 SavedDLock = 0, SavedILock = 0;
    u64 IRQUnlocks = 0, Relocks = 0, LockDiscards = 0;   // --dsc-lock-modonly (oracle): a lockdown applies to refills by module code only   // game code fetches with a lockdown set (outside the module; part in IRQ mode)

    u32 PU_CodeCacheable;
    u32 PU_DataCacheable;
    u32 PU_DataCacheWrite;

    u32 PU_CodeRW;
    u32 PU_DataRW;

    u32 PU_Region[8];

    // 0=dataR 1=dataW 2=codeR 4=datacache 5=datawrite 6=codecache
    u8 PU_PrivMap[0x100000];
    u8 PU_UserMap[0x100000];

    // games operate under system mode, generally
    //#define PU_Map PU_PrivMap
    u8* PU_Map;

    // code/16N/32N/32S
    u8 MemTimings[0x100000][4];

    u8* CurICacheLine;

    bool (*GetMemRegion)(u32 addr, bool write, MemRegion* region);

#ifdef GDBSTUB_ENABLED
    u32 ReadMem(u32 addr, int size) override;
    void WriteMem(u32 addr, int size, u32 v) override;
#endif

protected:
    u8 BusRead8(u32 addr) override;
    u16 BusRead16(u32 addr) override;
    u32 BusRead32(u32 addr) override;
    void BusWrite8(u32 addr, u8 val) override;
    void BusWrite16(u32 addr, u16 val) override;
    void BusWrite32(u32 addr, u32 val) override;
};

class ARMv4 : public ARM
{
public:
    ARMv4(melonDS::NDS& nds, std::optional<GDBArgs> gdb, bool jit);

    void FillPipeline() override;

    void JumpTo(u32 addr, bool restorecpsr = false) override;

    template <CPUExecuteMode mode>
    void Execute();

    u16 CodeRead16(u32 addr)
    {
        return BusRead16(addr);
    }

    u32 CodeRead32(u32 addr)
    {
        return BusRead32(addr);
    }

    void DataRead8(u32 addr, u32* val) override;
    void DataRead16(u32 addr, u32* val) override;
    void DataRead32(u32 addr, u32* val) override;
    void DataRead32S(u32 addr, u32* val) override;
    void DataWrite8(u32 addr, u8 val) override;
    void DataWrite16(u32 addr, u16 val) override;
    void DataWrite32(u32 addr, u32 val) override;
    void DataWrite32S(u32 addr, u32 val) override;
    void AddCycles_C() override;
    // Optional ARM7 profile (a frontend sets Prof7On): cycles per interval in total and in
    // [Prof7Lo, Prof7Hi) (e.g. nds-bootstrap's ARM7 code), and cycles by 256-byte PC block
    // (Prof7Blocks, indexed by (addr >> 8) & 0x3FFFF) over the whole run.
    bool Prof7On = false;
    u32 Prof7Lo = 0, Prof7Hi = 0;
    u64 Prof7Busy = 0, Prof7Range = 0;
    u64* Prof7Blocks = nullptr;
    void AddCycles_CI(s32 num) override;
    void AddCycles_CDI() override;
    void AddCycles_CD() override;
protected:
    u8 BusRead8(u32 addr) override;
    u16 BusRead16(u32 addr) override;
    u32 BusRead32(u32 addr) override;
    void BusWrite8(u32 addr, u8 val) override;
    void BusWrite16(u32 addr, u16 val) override;
    void BusWrite32(u32 addr, u32 val) override;
};

namespace ARMInterpreter
{

void A_UNK(ARM* cpu);
void T_UNK(ARM* cpu);

}
}
#endif // ARM_H
