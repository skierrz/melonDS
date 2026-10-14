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

#include <stdio.h>
#include <string.h>
#include "NDS.h"
#include "DSi.h"
#include "ARM.h"
#include "Platform.h"
#include "ARMJIT_Memory.h"
#include "ARMJIT.h"

namespace melonDS
{
using Platform::Log;
using Platform::LogLevel;

// access timing for cached regions
// this would be an average between cache hits and cache misses
// this was measured to be close to hardware average
// a value of 1 would represent a perfect cache, but that causes
// games to run too fast, causing a number of issues
const int kDataCacheTiming = 3;//2;
const int kCodeCacheTiming = 3;//5;


void ARMv5::CP15Reset()
{
    CP15Control = 0x2078; // dunno

    RNGSeed = 44203;
    TraceProcessID = 0;

    DTCMSetting = 0;
    ITCMSetting = 0;

    memset(ITCM, 0, ITCMPhysicalSize);
    memset(DTCM, 0, DTCMPhysicalSize);

    ITCMSize = 0;
    DTCMBase = 0xFFFFFFFF;
    DTCMMask = 0;

    memset(ICache, 0, 0x2000);
    ICacheInvalidateAll();
    memset(ICacheCount, 0, 64);
    CacheModelReset();

    PU_CodeCacheable = 0;
    PU_DataCacheable = 0;
    PU_DataCacheWrite = 0;

    PU_CodeRW = 0;
    PU_DataRW = 0;

    memset(PU_Region, 0, 8*sizeof(u32));
    UpdatePURegions(true);

    CurICacheLine = NULL;
}

void ARMv5::CP15DoSavestate(Savestate* file)
{
    file->Section("CP15");

    file->Var32(&CP15Control);

    file->Var32(&DTCMSetting);
    file->Var32(&ITCMSetting);

    file->VarArray(ITCM, ITCMPhysicalSize);
    file->VarArray(DTCM, DTCMPhysicalSize);

    file->Var32(&PU_CodeCacheable);
    file->Var32(&PU_DataCacheable);
    file->Var32(&PU_DataCacheWrite);

    file->Var32(&PU_CodeRW);
    file->Var32(&PU_DataRW);

    file->VarArray(PU_Region, 8*sizeof(u32));

    if (!file->Saving)
    {
        UpdateDTCMSetting();
        UpdateITCMSetting();
        UpdatePURegions(true);
    }
}


void ARMv5::UpdateDTCMSetting()
{
    u32 newDTCMBase;
    u32 newDTCMMask;
    u32 newDTCMSize;

    if (CP15Control & (1<<16))
    {
        newDTCMSize = 0x200 << ((DTCMSetting >> 1) & 0x1F);
        if (newDTCMSize < 0x1000) newDTCMSize = 0x1000;
        newDTCMMask = 0xFFFFF000 & ~(newDTCMSize-1);
        newDTCMBase = DTCMSetting & newDTCMMask;
    }
    else
    {
        newDTCMSize = 0;
        newDTCMBase = 0xFFFFFFFF;
        newDTCMMask = 0;
    }

    if (newDTCMBase != DTCMBase || newDTCMMask != DTCMMask)
    {
        NDS.JIT.Memory.RemapDTCM(newDTCMBase, newDTCMSize);
        DTCMBase = newDTCMBase;
        DTCMMask = newDTCMMask;
    }
}

void ARMv5::UpdateITCMSetting()
{
    if (CP15Control & (1<<18))
    {
        ITCMSize = 0x200 << ((ITCMSetting >> 1) & 0x1F);
#ifdef JIT_ENABLED
        FastBlockLookupSize = 0;
#endif
    }
    else
    {
        ITCMSize = 0;
    }
}


// covers updates to a specific PU region's cache/etc settings
// (not to the region range/enabled status)
void ARMv5::UpdatePURegion(u32 n)
{
    if (!(CP15Control & (1<<0)))
        return;

    u32 coderw = (PU_CodeRW >> (4*n)) & 0xF;
    u32 datarw = (PU_DataRW >> (4*n)) & 0xF;

    u32 codecache, datacache, datawrite;

    // datacache/datawrite
    // 0/0: goes to memory
    // 0/1: goes to memory
    // 1/0: goes to memory and cache
    // 1/1: goes to cache

    if (CP15Control & (1<<12))
        codecache = (PU_CodeCacheable >> n) & 0x1;
    else
        codecache = 0;

    if (CP15Control & (1<<2))
    {
        datacache = (PU_DataCacheable >> n) & 0x1;
        datawrite = (PU_DataCacheWrite >> n) & 0x1;
    }
    else
    {
        datacache = 0;
        datawrite = 0;
    }

    u32 rgn = PU_Region[n];
    if (!(rgn & (1<<0)))
    {
        return;
    }

    // notes:
    // * min size of a pu region is 4KiB (12 bits)
    // * size is calculated as size + 1, but the 12 lsb of address space are ignored, therefore we need it as size + 1 - 12, or size - 11
    // * pu regions are aligned based on their size
    u32 size = std::max((int)((rgn>>1) & 0x1F) - 11, 0); // obtain the size, subtract 11 and clamp to a min of 0.
    u32 start = ((rgn >> 12) >> size) << size; // determine the start offset, and use shifts to force alignment with a multiple of the size.
    u32 end = start + (1<<size); // add 1 left shifted by size to start to determine end point
    // dont need to bounds check the end point because the force alignment inherently prevents it from breaking

    u8 usermask = 0;
    u8 privmask = 0;

    switch (datarw)
    {
    case 0: break;
    case 1: privmask |= 0x03; break;
    case 2: privmask |= 0x03; usermask |= 0x01; break;
    case 3: privmask |= 0x03; usermask |= 0x03; break;
    case 5: privmask |= 0x01; break;
    case 6: privmask |= 0x01; usermask |= 0x01; break;
    default: Log(LogLevel::Warn, "!! BAD DATARW VALUE %d\n", datarw&0xF);
    }

    switch (coderw)
    {
    case 0: break;
    case 1: privmask |= 0x04; break;
    case 2: privmask |= 0x04; usermask |= 0x04; break;
    case 3: privmask |= 0x04; usermask |= 0x04; break;
    case 5: privmask |= 0x04; break;
    case 6: privmask |= 0x04; usermask |= 0x04; break;
    default: Log(LogLevel::Warn, "!! BAD CODERW VALUE %d\n", datarw&0xF);
    }

    if (datacache & 0x1)
    {
        privmask |= 0x10;
        usermask |= 0x10;

        if (datawrite & 0x1)
        {
            privmask |= 0x20;
            usermask |= 0x20;
        }
    }

    if (codecache & 0x1)
    {
        privmask |= 0x40;
        usermask |= 0x40;
    }

    Log(
        LogLevel::Debug,
        "PU region %d: %08X-%08X, user=%02X priv=%02X, %08X/%08X\n",
        n,
        start << 12,
        (end << 12) - 1,
        usermask,
        privmask,
        PU_DataRW,
        PU_CodeRW
    );

    for (u32 i = start; i < end; i++)
    {
        PU_UserMap[i] = usermask;
        PU_PrivMap[i] = privmask;
    }

    UpdateRegionTimings(start, end);
}

void ARMv5::UpdatePURegions(bool update_all)
{
    if (!(CP15Control & (1<<0)))
    {
        // PU disabled

        u8 mask = 0x07;
        if (CP15Control & (1<<2))  mask |= 0x30;
        if (CP15Control & (1<<12)) mask |= 0x40;

        memset(PU_UserMap, mask, 0x100000);
        memset(PU_PrivMap, mask, 0x100000);

        UpdateRegionTimings(0x00000, 0x100000);
        return;
    }

    if (update_all)
    {
        memset(PU_UserMap, 0, 0x100000);
        memset(PU_PrivMap, 0, 0x100000);
    }

    for (int n = 0; n < 8; n++)
    {
        UpdatePURegion(n);
    }

    // TODO: this is way unoptimized
    // should be okay unless the game keeps changing shit, tho
    if (update_all) UpdateRegionTimings(0x00000, 0x100000);

    // TODO: throw exception if the region we're running in has become non-executable, I guess
}

void ARMv5::UpdateRegionTimings(u32 addrstart, u32 addrend)
{
    for (u32 i = addrstart; i < addrend; i++)
    {
        u8 pu = PU_Map[i];
        u8* bustimings = NDS.ARM9MemTimings[i >> 2];

        if (pu & 0x40)
        {
            MemTimings[i][0] = 0xFF;//kCodeCacheTiming;
        }
        else
        {
            MemTimings[i][0] = bustimings[2] << NDS.ARM9ClockShift;
        }

        if (pu & 0x10)
        {
            MemTimings[i][1] = kDataCacheTiming;
            MemTimings[i][2] = kDataCacheTiming;
            MemTimings[i][3] = 1;
        }
        else
        {
            MemTimings[i][1] = bustimings[0] << NDS.ARM9ClockShift;
            MemTimings[i][2] = bustimings[2] << NDS.ARM9ClockShift;
            MemTimings[i][3] = bustimings[3] << NDS.ARM9ClockShift;
        }
    }
}


u32 ARMv5::RandomLineIndex()
{
    // lame RNG, but good enough for this purpose
    u32 s = RNGSeed;
    RNGSeed ^= (s*17);
    RNGSeed ^= (s*7);

    return (RNGSeed >> 17) & 0x3;
}

void ARMv5::ICacheLookup(u32 addr)
{
    u32 tag = addr & 0xFFFFF800;
    u32 id = (addr >> 5) & 0x3F;

    id <<= 2;
    if (ICacheTags[id+0] == tag)
    {
        CodeCycles = 1;
        CurICacheLine = &ICache[(id+0) << 5];
        return;
    }
    if (ICacheTags[id+1] == tag)
    {
        CodeCycles = 1;
        CurICacheLine = &ICache[(id+1) << 5];
        return;
    }
    if (ICacheTags[id+2] == tag)
    {
        CodeCycles = 1;
        CurICacheLine = &ICache[(id+2) << 5];
        return;
    }
    if (ICacheTags[id+3] == tag)
    {
        CodeCycles = 1;
        CurICacheLine = &ICache[(id+3) << 5];
        return;
    }

    // cache miss

    u32 line;
    if (CP15Control & (1<<14))
    {
        line = ICacheCount[id>>2];
        ICacheCount[id>>2] = (line+1) & 0x3;
    }
    else
    {
        line = RandomLineIndex();
    }

    line += id;

    addr &= ~0x1F;
    u8* ptr = &ICache[line << 5];

    if (CodeMem.Mem)
    {
        memcpy(ptr, &CodeMem.Mem[addr & CodeMem.Mask], 32);
    }
    else
    {
        for (int i = 0; i < 32; i+=4)
            *(u32*)&ptr[i] = NDS.ARM9Read32(addr+i);
    }

    ICacheTags[line] = tag;

    // ouch :/
    //printf("cache miss %08X: %d/%d\n", addr, NDS::ARM9MemTimings[addr >> 14][2], NDS::ARM9MemTimings[addr >> 14][3]);
    CodeCycles = (NDS.ARM9MemTimings[addr >> 14][2] + (NDS.ARM9MemTimings[addr >> 14][3] * 7)) << NDS.ARM9ClockShift;
    CurICacheLine = ptr;
}

void ARMv5::ICacheInvalidateByAddr(u32 addr)
{
    for (int w = 0; w < 4; w++)
    {
        u32& t = CMITags[(((addr >> 5) & 0x3F) << 2) + w];
        if ((t & ~0x1F) == (addr & ~0x1F)) t = 0;
    }

    u32 tag = addr & 0xFFFFF800;
    u32 id = (addr >> 5) & 0x3F;

    id <<= 2;
    if (ICacheTags[id+0] == tag)
    {
        ICacheTags[id+0] = 1;
        return;
    }
    if (ICacheTags[id+1] == tag)
    {
        ICacheTags[id+1] = 1;
        return;
    }
    if (ICacheTags[id+2] == tag)
    {
        ICacheTags[id+2] = 1;
        return;
    }
    if (ICacheTags[id+3] == tag)
    {
        ICacheTags[id+3] = 1;
        return;
    }
}

void ARMv5::ICacheInvalidateAll()
{
    for (int i = 0; i < 64*4; i++)
        ICacheTags[i] = 1;
    memset(CMITags, 0, sizeof(CMITags));
    if (ProfOn && !ProfCurMod) memset(Shadow.ITags, 0, sizeof(Shadow.ITags));
}


// ===== timing-only cache model =====
// ARM946E-S as used in the DS: 8 KB I-cache (64 sets) and 4 KB D-cache
// (32 sets), both 4-way with 32-byte lines. A miss costs a line fill: one
// nonsequential then seven sequential 32-bit reads at the bus timing of the
// address. Stores never allocate. A dirty victim costs a line write.

void ARMv5::CacheModelReset()
{
    memset(CMITags, 0, sizeof(CMITags));
    memset(CMICount, 0, sizeof(CMICount));
    memset(CMDTags, 0, sizeof(CMDTags));
    memset(CMDCount, 0, sizeof(CMDCount));
    CMLastDLine = 0xFFFFFFFF;
    CacheStats = {};
    ShadowReset();
}

void ARMv5::ShadowReset()
{
    memset(&Shadow, 0, sizeof(Shadow));
    Shadow.LastDLine = 0xFFFFFFFF;
    Shadow.Seed = RNGSeed;
}

// the shadow's victim choice: the same rule as the real one, on its own counter/seed
static inline u32 ShadowVictim(ARMv5::CacheShadow& sh, u8* count, u32 set, bool roundrobin)
{
    if (roundrobin)
    {
        u32 way = count[set];
        count[set] = (way + 1) & 3;
        return way;
    }
    u32 s = sh.Seed;
    sh.Seed ^= (s*17);
    sh.Seed ^= (s*7);
    return (sh.Seed >> 17) & 0x3;
}

static inline s32 CacheModelLineCost(NDS& nds, u32 addr)
{
    u8* t = nds.ARM9MemTimings[addr >> 14];
    return (t[2] + t[3] * 7) << nds.ARM9ClockShift;
}

static inline u32 CacheModelVictim(ARMv5& cpu, u8* count, u32 set, bool roundrobin)
{
    if (roundrobin)
    {
        u32 way = count[set];
        count[set] = (way + 1) & 3;
        return way;
    }
    return cpu.RandomLineIndex();
}

s32 ARMv5::CacheModelCode(u32 addr)
{
    s32 cost = CacheModelCodeReal(addr);
    if (!ProfOn || ProfInModule(addr)) return cost;
    // a game fetch: the shadow I-cache
    bool realmiss = cost != 1;
    if (realmiss) Prof.GameIMiss++;
    u32 line = addr & ~0x1F;
    u32 set = (addr >> 5) & 0x3F;
    u32* tags = &Shadow.ITags[set << 2];
    s32 scost = 1;
    bool hit = false;
    for (int w = 0; w < 4; w++)
        if ((tags[w] & ~0x1F) == line && (tags[w] & 1)) { hit = true; break; }
    if (!hit)
    {
        tags[ShadowVictim(Shadow, Shadow.ICount, set, CP15Control & (1<<14))] = line | 1;
        scost = CacheModelLineCost(NDS, addr);
    }
    s64 d = (s64)(cost - scost) << (2 - NDS.ARM9ClockShift);
    Prof.GameIExtra += d;
    if ((CPSR & 0x1F) == 0x12) Prof.GameExtraIRQ += d;
    if (realmiss && hit) Prof.PollI++;
    if (!realmiss && !hit) Prof.RevI++;
    if (ProfGamePCs && d) { auto& e = (*ProfGamePCs)[line]; e.IExtra += d; if (realmiss && hit) e.PollI++; }
    return cost;
}

s32 ARMv5::CacheModelCodeReal(u32 addr)
{
    u32 line = addr & ~0x1F;
    u32 set = (addr >> 5) & 0x3F;
    if (WhatIfISets && ProfOn && ProfInModule(addr)) set &= WhatIfISets; // what-if: module code in fewer sets
    u32* tags = &CMITags[set << 2];
    for (int w = 0; w < 4; w++)
    {
        if ((tags[w] & ~0x1F) == line && (tags[w] & 1))
        {
            CacheStats.IHit++;
            return 1;
        }
    }
    CacheStats.IMiss++;
    tags[CacheModelVictim(*this, CMICount, set, CP15Control & (1<<14))] = line | 1;
    return CacheModelLineCost(NDS, addr);
}

// the shadow D-cache for a game access: returns its cost and whether it missed (a read
// line fill, or a write that went to memory); dirty write-backs counted in *wb
static s32 ShadowData(ARMv5& cpu, NDS& nds, u32 addr, bool write, bool writeback, bool seq, s32 uncached, bool* miss, bool* wb)
{
    auto& sh = cpu.Shadow;
    *miss = false; *wb = false;
    u32 line = addr & ~0x1F;
    if (seq && line == sh.LastDLine)
        return 1;
    sh.LastDLine = line;
    u32 set = (addr >> 5) & 0x1F;
    u32* tags = &sh.DTags[set << 2];
    for (int w = 0; w < 4; w++)
    {
        if ((tags[w] & ~0x1F) == line && (tags[w] & 1))
        {
            if (!write) return 1;
            if (writeback) { tags[w] |= 2; return 1; }
            return uncached;
        }
    }
    *miss = true;
    if (write) return uncached;
    u32 way = ShadowVictim(sh, sh.DCount, set, cpu.CP15Control & (1<<14));
    s32 cost = CacheModelLineCost(nds, addr);
    if ((tags[way] & 3) == 3)
    {
        *wb = true;
        cost += CacheModelLineCost(nds, tags[way] & ~0x1F);
    }
    tags[way] = line | 1;
    return cost;
}

s32 ARMv5::CacheModelData(u32 addr, bool write, bool writeback, bool seq, s32 uncached)
{
    if (!ProfOn || ProfCurMod) return CacheModelDataReal(addr, write, writeback, seq, uncached);
    CacheModelStats before = CacheStats;
    s32 cost = CacheModelDataReal(addr, write, writeback, seq, uncached);
    bool realmiss = CacheStats.DReadMiss != before.DReadMiss || CacheStats.DWriteMiss != before.DWriteMiss;
    Prof.GameDReadMiss += CacheStats.DReadMiss - before.DReadMiss;
    Prof.GameDWriteMiss += CacheStats.DWriteMiss - before.DWriteMiss;
    bool realwb = CacheStats.DWriteBack != before.DWriteBack;
    if (realwb) Prof.GameDWriteBack++;
    bool smiss, swb;
    s32 scost = ShadowData(*this, NDS, addr, write, writeback, seq, uncached, &smiss, &swb);
    s64 d = (s64)(cost - scost) << (2 - NDS.ARM9ClockShift);
    Prof.GameDExtra += d;
    if ((CPSR & 0x1F) == 0x12) Prof.GameExtraIRQ += d;
    if (realmiss && !smiss) Prof.PollD++;
    if (!realmiss && smiss) Prof.RevD++;
    if (realwb && !swb) Prof.PollWB++;
    if (ProfGamePCs && d) { auto& e = (*ProfGamePCs)[ProfCurPC]; e.DExtra += d; if (realmiss && !smiss) e.PollD++; }
    return cost;
}

s32 ARMv5::CacheModelDataReal(u32 addr, bool write, bool writeback, bool seq, s32 uncached)
{
    u32 line = addr & ~0x1F;
    if (seq && line == CMLastDLine)
        return 1; // LDM/STM continuing within the line just accessed
    CMLastDLine = line;

    u32 set = (addr >> 5) & 0x1F;
    if (WhatIfDSets && ProfOn && ProfCurMod) set &= WhatIfDSets;   // what-if: module data in fewer sets
    u32* tags = &CMDTags[set << 2];
    for (int w = 0; w < 4; w++)
    {
        if ((tags[w] & ~0x1F) == line && (tags[w] & 1))
        {
            if (!write)
            {
                CacheStats.DReadHit++;
                return 1;
            }
            CacheStats.DWriteHit++;
            if (writeback)
            {
                tags[w] |= 2;
                return 1;
            }
            return uncached; // write-through: also goes to memory
        }
    }

    if ((ProfDMiss || ProfIdleDMiss) && ProfPendN < 8) ProfPend[ProfPendN++] = (addr & ~3u) | (write ? 1 : 0);
    if (write)
    {
        CacheStats.DWriteMiss++;
        return uncached; // no write-allocate
    }

    CacheStats.DReadMiss++;
    u32 way = CacheModelVictim(*this, CMDCount, set, CP15Control & (1<<14));
    s32 cost = CacheModelLineCost(NDS, addr);
    if ((tags[way] & 3) == 3)
    {
        CacheStats.DWriteBack++;
        cost += CacheModelLineCost(NDS, tags[way] & ~0x1F);
    }
    tags[way] = line | 1;
    return cost;
}

void ARMv5::CacheModelDInvalidateLine(u32 addr, bool clean)
{
    if (ProfOn && !ProfCurMod)
    {
        u32* st = &Shadow.DTags[((addr >> 5) & 0x1F) << 2];
        for (int w = 0; w < 4; w++)
            if ((st[w] & ~0x1F) == (addr & ~0x1F)) st[w] = 0;
        Shadow.LastDLine = 0xFFFFFFFF;
    }
    u32 line = addr & ~0x1F;
    u32* tags = &CMDTags[((addr >> 5) & 0x1F) << 2];
    for (int w = 0; w < 4; w++)
        if ((tags[w] & ~0x1F) == line)
        {
            if (clean && (tags[w] & 3) == 3) CacheStats.DWriteBack++;
            tags[w] = 0;
        }
    CMLastDLine = 0xFFFFFFFF;
}

void ARMv5::CacheModelDInvalidateSetWay(u32 val, bool clean)
{
    // set index in bits [9:5] for a 4 KB cache, way in bits [31:30]
    u32 idx = (((val >> 5) & 0x1F) << 2) | (val >> 30);
    if (ProfOn && !ProfCurMod) { Shadow.DTags[idx] = 0; Shadow.LastDLine = 0xFFFFFFFF; }
    if (clean && (CMDTags[idx] & 3) == 3) CacheStats.DWriteBack++;
    CMDTags[idx] = 0;
    CMLastDLine = 0xFFFFFFFF;
}

void ARMv5::CacheModelDInvalidateAll()
{
    if (ProfOn && !ProfCurMod) { memset(Shadow.DTags, 0, sizeof(Shadow.DTags)); Shadow.LastDLine = 0xFFFFFFFF; }
    memset(CMDTags, 0, sizeof(CMDTags));
    CMLastDLine = 0xFFFFFFFF;
}


void ARMv5::CP15Write(u32 id, u32 val)
{
    //if(id!=0x704)printf("CP15 write op %03X %08X %08X\n", id, val, R[15]);

    switch (id)
    {
    case 0x100:
        {
            u32 old = CP15Control;
            val &= 0x000FF085;
            CP15Control &= ~0x000FF085;
            CP15Control |= val;
            //printf("CP15Control = %08X (%08X->%08X)\n", CP15Control, old, val);
            UpdateDTCMSetting();
            UpdateITCMSetting();
            if ((old & 0x1005) != (val & 0x1005))
            {
                UpdatePURegions((old & 0x1) != (val & 0x1));
            }
            if (val & (1<<7)) Log(LogLevel::Warn, "!!!! ARM9 BIG ENDIAN MODE. VERY BAD. SHIT GONNA ASPLODE NOW\n");
            if (val & (1<<13)) ExceptionBase = 0xFFFF0000;
            else               ExceptionBase = 0x00000000;
        }
        return;


    case 0x200: // data cacheable
        {
            u32 diff = PU_DataCacheable ^ val;
            PU_DataCacheable = val;
            for (u32 i = 0; i < 8; i++)
            {
                if (diff & (1<<i)) UpdatePURegion(i);
            }
        }
        return;

    case 0x201: // code cacheable
        {
            u32 diff = PU_CodeCacheable ^ val;
            PU_CodeCacheable = val;
            for (u32 i = 0; i < 8; i++)
            {
                if (diff & (1<<i)) UpdatePURegion(i);
            }
        }
        return;


    case 0x300: // data cache write-buffer
        {
            u32 diff = PU_DataCacheWrite ^ val;
            PU_DataCacheWrite = val;
            for (u32 i = 0; i < 8; i++)
            {
                if (diff & (1<<i)) UpdatePURegion(i);
            }
        }
        return;


    case 0x500: // legacy data permissions
        {
            u32 old = PU_DataRW;
            PU_DataRW = 0;
            PU_DataRW |= (val & 0x0003);
            PU_DataRW |= ((val & 0x000C) << 2);
            PU_DataRW |= ((val & 0x0030) << 4);
            PU_DataRW |= ((val & 0x00C0) << 6);
            PU_DataRW |= ((val & 0x0300) << 8);
            PU_DataRW |= ((val & 0x0C00) << 10);
            PU_DataRW |= ((val & 0x3000) << 12);
            PU_DataRW |= ((val & 0xC000) << 14);
            u32 diff = old ^ PU_DataRW;
            for (u32 i = 0; i < 8; i++)
            {
                if (diff & (0xF<<(i*4))) UpdatePURegion(i);
            }
        }
        return;

    case 0x501: // legacy code permissions
        {
            u32 old = PU_CodeRW;
            PU_CodeRW = 0;
            PU_CodeRW |= (val & 0x0003);
            PU_CodeRW |= ((val & 0x000C) << 2);
            PU_CodeRW |= ((val & 0x0030) << 4);
            PU_CodeRW |= ((val & 0x00C0) << 6);
            PU_CodeRW |= ((val & 0x0300) << 8);
            PU_CodeRW |= ((val & 0x0C00) << 10);
            PU_CodeRW |= ((val & 0x3000) << 12);
            PU_CodeRW |= ((val & 0xC000) << 14);
            u32 diff = old ^ PU_CodeRW;
            for (u32 i = 0; i < 8; i++)
            {
                if (diff & (0xF<<(i*4))) UpdatePURegion(i);
            }
        }
        return;

    case 0x502: // data permissions
        {
            u32 diff = PU_DataRW ^ val;
            PU_DataRW = val;
            for (u32 i = 0; i < 8; i++)
            {
                if (diff & (0xF<<(i*4))) UpdatePURegion(i);
            }
        }
        return;

    case 0x503: // code permissions
        {
            u32 diff = PU_CodeRW ^ val;
            PU_CodeRW = val;
            for (u32 i = 0; i < 8; i++)
            {
                if (diff & (0xF<<(i*4))) UpdatePURegion(i);
            }
        }
        return;


    case 0x600:
    case 0x601:
    case 0x610:
    case 0x611:
    case 0x620:
    case 0x621:
    case 0x630:
    case 0x631:
    case 0x640:
    case 0x641:
    case 0x650:
    case 0x651:
    case 0x660:
    case 0x661:
    case 0x670:
    case 0x671:
        char log_output[1024];
        PU_Region[(id >> 4) & 0xF] = val;

        std::snprintf(log_output,
                 sizeof(log_output),
                 "PU: region %d = %08X : %s, start: %08X size: %02X\n",
                 (id >> 4) & 0xF,
                 val,
                 val & 1 ? "enabled" : "disabled",
                 val & 0xFFFFF000,
                 (val & 0x3E) >> 1
        );
        Log(LogLevel::Debug, "%s", log_output);
        // Some implementations of Log imply a newline, so we build up the line before printing it

        // TODO: smarter region update for this?
        UpdatePURegions(true);
        return;


    case 0x704:
    case 0x782:
        Halt(1);
        return;


    case 0x750:
        ICacheInvalidateAll();
        //Halt(255);
        return;
    case 0x751:
        ICacheInvalidateByAddr(val);
        //Halt(255);
        return;
    case 0x752:
        Log(LogLevel::Warn, "CP15: ICACHE INVALIDATE WEIRD. %08X\n", val);
        //Halt(255);
        return;


    case 0x760:
        CacheModelDInvalidateAll();
        return;
    case 0x761:
        //printf("inval data cache %08X\n", val);
        CacheModelDInvalidateLine(val, false);
        return;
    case 0x762:
        //printf("inval data cache SI\n");
        CacheModelDInvalidateSetWay(val, false);
        return;

    case 0x7A1:
        //printf("flush data cache %08X\n", val);
        // clean: write back but keep the line
        for (int w = 0; w < 4; w++)
        {
            u32& t = CMDTags[(((val >> 5) & 0x1F) << 2) + w];
            if ((t & ~0x1F) == (val & ~0x1F) && (t & 3) == 3) { t &= ~2; CacheStats.DWriteBack++; }
            u32& st = Shadow.DTags[(((val >> 5) & 0x1F) << 2) + w];
            if (ProfOn && !ProfCurMod && (st & ~0x1F) == (val & ~0x1F)) st &= ~2;
        }
        return;
    case 0x7A2:
        //printf("flush data cache SI\n");
        {
            u32& t = CMDTags[(((val >> 5) & 0x1F) << 2) | (val >> 30)];
            if ((t & 3) == 3) { t &= ~2; CacheStats.DWriteBack++; }
            if (ProfOn && !ProfCurMod) Shadow.DTags[(((val >> 5) & 0x1F) << 2) | (val >> 30)] &= ~2;
        }
        return;

    case 0x7E1:
        CacheModelDInvalidateLine(val, true);
        return;
    case 0x7E2:
        CacheModelDInvalidateSetWay(val, true);
        return;


    case 0x910:
        DTCMSetting = val & 0xFFFFF03E;
        UpdateDTCMSetting();
        return;

    case 0x911:
        ITCMSetting = val & 0x0000003E;
        UpdateITCMSetting();
        return;

    case 0xD01:
        TraceProcessID = val;
        return;

    case 0xF00:
        //printf("cache debug index register %08X\n", val);
        return;

    case 0xF10:
        //printf("cache debug instruction tag %08X\n", val);
        return;

    case 0xF20:
        //printf("cache debug data tag %08X\n", val);
        return;

    case 0xF30:
        //printf("cache debug instruction cache %08X\n", val);
        return;

    case 0xF40:
        //printf("cache debug data cache %08X\n", val);
        return;

    }

    if ((id & 0xF00) == 0xF00) // test/debug shit?
        return;

    if ((id & 0xF00) != 0x700)
        Log(LogLevel::Debug, "unknown CP15 write op %03X %08X\n", id, val);
}

u32 ARMv5::CP15Read(u32 id) const
{
    //printf("CP15 read op %03X %08X\n", id, NDS::ARM9->R[15]);

    switch (id)
    {
    case 0x000: // CPU ID
    case 0x003:
    case 0x004:
    case 0x005:
    case 0x006:
    case 0x007:
        return 0x41059461;

    case 0x001: // cache type
        return 0x0F0D2112;

    case 0x002: // TCM size
        return (6 << 6) | (5 << 18);


    case 0x100: // control reg
        return CP15Control;


    case 0x200:
        return PU_DataCacheable;
    case 0x201:
        return PU_CodeCacheable;
    case 0x300:
        return PU_DataCacheWrite;


    case 0x500:
        {
            u32 ret = 0;
            ret |=  (PU_DataRW & 0x00000003);
            ret |= ((PU_DataRW & 0x00000030) >> 2);
            ret |= ((PU_DataRW & 0x00000300) >> 4);
            ret |= ((PU_DataRW & 0x00003000) >> 6);
            ret |= ((PU_DataRW & 0x00030000) >> 8);
            ret |= ((PU_DataRW & 0x00300000) >> 10);
            ret |= ((PU_DataRW & 0x03000000) >> 12);
            ret |= ((PU_DataRW & 0x30000000) >> 14);
            return ret;
        }
    case 0x501:
        {
            u32 ret = 0;
            ret |=  (PU_CodeRW & 0x00000003);
            ret |= ((PU_CodeRW & 0x00000030) >> 2);
            ret |= ((PU_CodeRW & 0x00000300) >> 4);
            ret |= ((PU_CodeRW & 0x00003000) >> 6);
            ret |= ((PU_CodeRW & 0x00030000) >> 8);
            ret |= ((PU_CodeRW & 0x00300000) >> 10);
            ret |= ((PU_CodeRW & 0x03000000) >> 12);
            ret |= ((PU_CodeRW & 0x30000000) >> 14);
            return ret;
        }
    case 0x502:
        return PU_DataRW;
    case 0x503:
        return PU_CodeRW;


    case 0x600:
    case 0x601:
    case 0x610:
    case 0x611:
    case 0x620:
    case 0x621:
    case 0x630:
    case 0x631:
    case 0x640:
    case 0x641:
    case 0x650:
    case 0x651:
    case 0x660:
    case 0x661:
    case 0x670:
    case 0x671:
        return PU_Region[(id >> 4) & 0xF];


    case 0x910:
        return DTCMSetting;
    case 0x911:
        return ITCMSetting;

    case 0xD01:
        return TraceProcessID;
    }

    if ((id & 0xF00) == 0xF00) // test/debug shit?
        return 0;

    Log(LogLevel::Debug, "unknown CP15 read op %03X\n", id);
    return 0;
}


// TCM are handled here.
// TODO: later on, handle PU, and maybe caches

u32 ARMv5::CodeRead32(u32 addr, bool branch)
{
    /*if (branch || (!(addr & 0xFFF)))
    {
        if (!(PU_Map[addr>>12] & 0x04))
        {
            PrefetchAbort();
            return 0;
        }
    }*/

    if (addr < ITCMSize)
    {
        CodeCycles = 1;
        return *(u32*)&ITCM[addr & (ITCMPhysicalSize - 1)];
    }

    CodeCycles = RegionCodeCycles;
    if (CodeCycles == 0xFF) // cached memory. hax
    {
        if (branch || !(addr & 0x1F))
            CodeCycles = CacheModel ? CacheModelCode(addr) : kCodeCacheTiming;//ICacheLookup(addr);
        else
            CodeCycles = 1;

        //return *(u32*)&CurICacheLine[addr & 0x1C];
    }

    if (CodeMem.Mem) return *(u32*)&CodeMem.Mem[addr & CodeMem.Mask];

    return BusRead32(addr);
}



// Data access cost. With the cache model off this is the stock table value.
// With it on, cacheable pages cost a hit or a line fill; others use bus timing
// (the uncached value melonDS would charge for the page).
inline s32 ARMv5::DataCacheCycles(u32 addr, bool write, bool seq, s32 table)
{
    if (!CacheModel) return table;
    u8 pu = PU_Map[addr >> 12];
    if (!(pu & 0x10))
    {
        if ((ProfDMiss || ProfIdleDMiss) && ProfPendN < 8) ProfPend[ProfPendN++] = (addr & ~3u) | 2 | (write ? 1 : 0);
        return table;
    }
    u8* bus = NDS.ARM9MemTimings[addr >> 14];
    s32 uncached = (write ? (seq ? bus[3] : bus[2]) : bus[2]) << NDS.ARM9ClockShift;
    return CacheModelData(addr, write, pu & 0x20, seq, uncached);
}

void ARMv5::DataRead8(u32 addr, u32* val)
{
    if (!(PU_Map[addr>>12] & 0x01))
    {
        DataAbort();
        return;
    }

    DataRegion = addr;

    if (addr < ITCMSize)
    {
        DataCycles = 1;
        *val = *(u8*)&ITCM[addr & (ITCMPhysicalSize - 1)];
        return;
    }
    if ((addr & DTCMMask) == DTCMBase)
    {
        DataCycles = 1;
        *val = *(u8*)&DTCM[addr & (DTCMPhysicalSize - 1)];
        return;
    }

    *val = BusRead8(addr);
    DataCycles = DataCacheCycles(addr, false, false, MemTimings[addr >> 12][1]);
}

void ARMv5::DataRead16(u32 addr, u32* val)
{
    if (!(PU_Map[addr>>12] & 0x01))
    {
        DataAbort();
        return;
    }

    DataRegion = addr;

    addr &= ~1;

    if (addr < ITCMSize)
    {
        DataCycles = 1;
        *val = *(u16*)&ITCM[addr & (ITCMPhysicalSize - 1)];
        return;
    }
    if ((addr & DTCMMask) == DTCMBase)
    {
        DataCycles = 1;
        *val = *(u16*)&DTCM[addr & (DTCMPhysicalSize - 1)];
        return;
    }

    *val = BusRead16(addr);
    DataCycles = DataCacheCycles(addr, false, false, MemTimings[addr >> 12][1]);
}

void ARMv5::DataRead32(u32 addr, u32* val)
{
    if (!(PU_Map[addr>>12] & 0x01))
    {
        DataAbort();
        return;
    }

    DataRegion = addr;

    addr &= ~3;

    if (addr < ITCMSize)
    {
        DataCycles = 1;
        *val = *(u32*)&ITCM[addr & (ITCMPhysicalSize - 1)];
        return;
    }
    if ((addr & DTCMMask) == DTCMBase)
    {
        DataCycles = 1;
        *val = *(u32*)&DTCM[addr & (DTCMPhysicalSize - 1)];
        return;
    }

    *val = BusRead32(addr);
    DataCycles = DataCacheCycles(addr, false, false, MemTimings[addr >> 12][2]);
}

void ARMv5::DataRead32S(u32 addr, u32* val)
{
    addr &= ~3;

    if (addr < ITCMSize)
    {
        DataCycles += 1;
        *val = *(u32*)&ITCM[addr & (ITCMPhysicalSize - 1)];
        return;
    }
    if ((addr & DTCMMask) == DTCMBase)
    {
        DataCycles += 1;
        *val = *(u32*)&DTCM[addr & (DTCMPhysicalSize - 1)];
        return;
    }

    *val = BusRead32(addr);
    DataCycles += DataCacheCycles(addr, false, true, MemTimings[addr >> 12][3]);
}

void ARMv5::DataWrite8(u32 addr, u8 val)
{
    if (!(PU_Map[addr>>12] & 0x02))
    {
        DataAbort();
        return;
    }

    DataRegion = addr;

    if (addr < ITCMSize)
    {
        DataCycles = 1;
        *(u8*)&ITCM[addr & (ITCMPhysicalSize - 1)] = val;
        NDS.JIT.CheckAndInvalidate<0, ARMJIT_Memory::memregion_ITCM>(addr);
        return;
    }
    if ((addr & DTCMMask) == DTCMBase)
    {
        DataCycles = 1;
        *(u8*)&DTCM[addr & (DTCMPhysicalSize - 1)] = val;
        return;
    }

    BusWrite8(addr, val);
    DataCycles = DataCacheCycles(addr, true, false, MemTimings[addr >> 12][1]);
}

void ARMv5::DataWrite16(u32 addr, u16 val)
{
    if (!(PU_Map[addr>>12] & 0x02))
    {
        DataAbort();
        return;
    }

    DataRegion = addr;

    addr &= ~1;

    if (addr < ITCMSize)
    {
        DataCycles = 1;
        *(u16*)&ITCM[addr & (ITCMPhysicalSize - 1)] = val;
        NDS.JIT.CheckAndInvalidate<0, ARMJIT_Memory::memregion_ITCM>(addr);
        return;
    }
    if ((addr & DTCMMask) == DTCMBase)
    {
        DataCycles = 1;
        *(u16*)&DTCM[addr & (DTCMPhysicalSize - 1)] = val;
        return;
    }

    BusWrite16(addr, val);
    DataCycles = DataCacheCycles(addr, true, false, MemTimings[addr >> 12][1]);
}

void ARMv5::DataWrite32(u32 addr, u32 val)
{
    if (!(PU_Map[addr>>12] & 0x02))
    {
        DataAbort();
        return;
    }

    DataRegion = addr;

    addr &= ~3;

    if (addr < ITCMSize)
    {
        DataCycles = 1;
        *(u32*)&ITCM[addr & (ITCMPhysicalSize - 1)] = val;
        NDS.JIT.CheckAndInvalidate<0, ARMJIT_Memory::memregion_ITCM>(addr);
        return;
    }
    if ((addr & DTCMMask) == DTCMBase)
    {
        DataCycles = 1;
        *(u32*)&DTCM[addr & (DTCMPhysicalSize - 1)] = val;
        return;
    }

    BusWrite32(addr, val);
    DataCycles = DataCacheCycles(addr, true, false, MemTimings[addr >> 12][2]);
}

void ARMv5::DataWrite32S(u32 addr, u32 val)
{
    addr &= ~3;

    if (addr < ITCMSize)
    {
        DataCycles += 1;
        *(u32*)&ITCM[addr & (ITCMPhysicalSize - 1)] = val;
#ifdef JIT_ENABLED
        NDS.JIT.CheckAndInvalidate<0, ARMJIT_Memory::memregion_ITCM>(addr);
#endif
        return;
    }
    if ((addr & DTCMMask) == DTCMBase)
    {
        DataCycles += 1;
        *(u32*)&DTCM[addr & (DTCMPhysicalSize - 1)] = val;
        return;
    }

    BusWrite32(addr, val);
    DataCycles += DataCacheCycles(addr, true, true, MemTimings[addr >> 12][3]);
}

void ARMv5::GetCodeMemRegion(u32 addr, MemRegion* region)
{
    /*if (addr < ITCMSize)
    {
        region->Mem = ITCM;
        region->Mask = 0x7FFF;
        return;
    }*/

    NDS.ARM9GetMemRegion(addr, false, &CodeMem);
}

}
