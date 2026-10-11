// Minimal melonDS Platform implementation for the headless runner.
// Files via stdio, threads via std::thread, everything else a no-op.

#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <filesystem>

#include "Platform.h"
#include "SPI_Firmware.h"

void hl_log(melonDS::Platform::LogLevel level, const char* msg);
void hl_signal_stop();
int hl_net_send(melonDS::u8* data, int len);
int hl_net_recv(melonDS::u8* data);

namespace melonDS::Platform
{

void SignalStop(StopReason reason, void*)
{
    std::fprintf(stderr, "[runner] emulator stop, reason %d\n", (int)reason);
    hl_signal_stop();
}

static const char* fmode(FileMode m)
{
    bool r = m & Read, w = m & Write;
    if (m & Append) return r ? "a+b" : "ab";
    if (r && w) return (m & Preserve) ? "r+b" : "w+b";
    if (w) return "wb";
    return "rb";
}

FileHandle* OpenFile(const std::string& path, FileMode mode)
{
    if ((mode & NoCreate) && !std::filesystem::exists(path)) return nullptr;
    if ((mode & Preserve) && !std::filesystem::exists(path))
    {
        FILE* c = std::fopen(path.c_str(), "wb");
        if (c) std::fclose(c);
    }
    return (FileHandle*)std::fopen(path.c_str(), fmode(mode));
}

std::string GetLocalFilePath(const std::string& filename) { return filename; }
FileHandle* OpenLocalFile(const std::string& path, FileMode mode) { return OpenFile(path, mode); }
bool FileExists(const std::string& name) { return std::filesystem::exists(name); }
bool LocalFileExists(const std::string& name) { return FileExists(name); }
bool CheckFileWritable(const std::string& p)
{
    FILE* f = std::fopen(p.c_str(), "ab");
    if (!f) return false;
    std::fclose(f);
    return true;
}
bool CheckLocalFileWritable(const std::string& p) { return CheckFileWritable(p); }
bool CloseFile(FileHandle* f) { return std::fclose((FILE*)f) == 0; }
bool IsEndOfFile(FileHandle* f) { return std::feof((FILE*)f) != 0; }
bool FileReadLine(char* s, int n, FileHandle* f) { return std::fgets(s, n, (FILE*)f) != nullptr; }
u64 FilePosition(FileHandle* f) { return (u64)std::ftell((FILE*)f); }
bool FileSeek(FileHandle* f, s64 off, FileSeekOrigin o)
{
    int w = o == FileSeekOrigin::Start ? SEEK_SET : o == FileSeekOrigin::Current ? SEEK_CUR : SEEK_END;
    return fseeko((FILE*)f, off, w) == 0;
}
void FileRewind(FileHandle* f) { std::rewind((FILE*)f); }
u64 FileRead(void* d, u64 s, u64 c, FileHandle* f) { return std::fread(d, s, c, (FILE*)f); }
bool FileFlush(FileHandle* f) { return std::fflush((FILE*)f) == 0; }
u64 FileWrite(const void* d, u64 s, u64 c, FileHandle* f) { return std::fwrite(d, s, c, (FILE*)f); }
u64 FileWriteFormatted(FileHandle* f, const char* fmt, ...)
{
    va_list a; va_start(a, fmt);
    int r = std::vfprintf((FILE*)f, fmt, a);
    va_end(a);
    return r < 0 ? 0 : (u64)r;
}
u64 FileLength(FileHandle* f)
{
    FILE* fp = (FILE*)f;
    long pos = std::ftell(fp);
    std::fseek(fp, 0, SEEK_END);
    long len = std::ftell(fp);
    std::fseek(fp, pos, SEEK_SET);
    return (u64)len;
}

void Log(LogLevel level, const char* fmt, ...)
{
    char buf[4096];
    va_list a; va_start(a, fmt);
    std::vsnprintf(buf, sizeof buf, fmt, a);
    va_end(a);
    hl_log(level, buf);
}

// ---- threads ----
struct Thread { std::thread t; };
Thread* Thread_Create(std::function<void()> fn) { return new Thread{std::thread(std::move(fn))}; }
void Thread_Free(Thread* t) { if (t->t.joinable()) t->t.detach(); delete t; }
void Thread_Wait(Thread* t) { if (t->t.joinable()) t->t.join(); }

struct Semaphore { std::mutex m; std::condition_variable cv; int count = 0; };
Semaphore* Semaphore_Create() { return new Semaphore; }
void Semaphore_Free(Semaphore* s) { delete s; }
void Semaphore_Reset(Semaphore* s) { std::lock_guard l(s->m); s->count = 0; }
void Semaphore_Wait(Semaphore* s) { std::unique_lock l(s->m); s->cv.wait(l, [&]{ return s->count > 0; }); s->count--; }
bool Semaphore_TryWait(Semaphore* s, int ms)
{
    std::unique_lock l(s->m);
    if (!s->cv.wait_for(l, std::chrono::milliseconds(ms), [&]{ return s->count > 0; })) return false;
    s->count--;
    return true;
}
void Semaphore_Post(Semaphore* s, int n) { { std::lock_guard l(s->m); s->count += n; } s->cv.notify_all(); }

struct Mutex { std::mutex m; };
Mutex* Mutex_Create() { return new Mutex; }
void Mutex_Free(Mutex* m) { delete m; }
void Mutex_Lock(Mutex* m) { m->m.lock(); }
void Mutex_Unlock(Mutex* m) { m->m.unlock(); }
bool Mutex_TryLock(Mutex* m) { return m->m.try_lock(); }

void Sleep(u64 us) { std::this_thread::sleep_for(std::chrono::microseconds(us)); }
u64 GetMSCount() { return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
u64 GetUSCount() { return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

// ---- saves/firmware/RTC: kept in memory only ----
void WriteNDSSave(const u8*, u32, u32, u32, void*) {}
void WriteGBASave(const u8*, u32, u32, u32, void*) {}
void WriteFirmware(const Firmware&, u32, u32, void*) {}
void WriteDateTime(int, int, int, int, int, int, void*) {}

// ---- local multiplayer / network: not connected yet ----
void MP_Begin(void*) {}
void MP_End(void*) {}
int MP_SendPacket(u8*, int len, u64, void*) { return len; }
int MP_RecvPacket(u8*, u64*, void*) { return 0; }
int MP_SendCmd(u8*, int len, u64, void*) { return len; }
int MP_SendReply(u8*, int len, u64, u16, void*) { return len; }
int MP_SendAck(u8*, int len, u64, void*) { return len; }
int MP_RecvHostPacket(u8*, u64*, void*) { return 0; }
u16 MP_RecvReplies(u8*, u64, u16, void*) { return 0; }
int Net_SendPacket(u8* data, int len, void*) { return hl_net_send(data, len); }
int Net_RecvPacket(u8* data, void*) { return hl_net_recv(data); }

// ---- peripherals ----
void Camera_Start(int, void*) {}
void Camera_Stop(int, void*) {}
void Camera_CaptureFrame(int, u32* frame, int w, int h, bool, void*) { std::memset(frame, 0, (size_t)w * h * 4); }
void Mic_Start(void*) {}
void Mic_Stop(void*) {}
int Mic_ReadInput(s16*, int, void*) { return 0; }
AACDecoder* AAC_Init() { return nullptr; }
void AAC_DeInit(AACDecoder*) {}
bool AAC_Configure(AACDecoder*, int, int) { return false; }
bool AAC_DecodeFrame(AACDecoder*, const void*, int, void*, int) { return false; }
bool Addon_KeyDown(KeyType, void*) { return false; }
void Addon_RumbleStart(u32, void*) {}
void Addon_RumbleStop(void*) {}
float Addon_MotionQuery(MotionQueryType, void*) { return 0.0f; }

DynamicLibrary* DynamicLibrary_Load(const char*) { return nullptr; }
void DynamicLibrary_Unload(DynamicLibrary*) {}
void* DynamicLibrary_LoadFunction(DynamicLibrary*, const char*) { return nullptr; }

}
