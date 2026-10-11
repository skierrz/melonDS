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

#ifndef DSI_NWIFI_H
#define DSI_NWIFI_H

#include "DSi_SD.h"
#include "FIFO.h"
#include "Savestate.h"

namespace melonDS
{
class DSi_NWifi : public DSi_SDDevice
{
public:
    DSi_NWifi(melonDS::DSi& dsi, DSi_SDHost* host);
    ~DSi_NWifi();

    void Reset();

    void DoSavestate(Savestate* file);

    void SendCMD(MMCCommand cmd, u32 param);
    void SendACMD(MMCAppCommand cmd, u32 param);

    void ContinueTransfer();

    void SetIRQ_F1_Counter(u32 n);

    void MSTimer(u32 param);

    // Test hooks for the emulated access point: it drops us (deauthentication),
    // switches to WPA2-PSK (CCMP) with this PMK, starts a group key rekey.
    void DropConnection();
    void EnableWPA2(const u8* pmk);
    void StartGroupRekey();
    struct WPAStats
    {
        u32 Handshakes;     // 4-way handshakes completed
        u32 GroupRekeys;    // group rekeys answered
        u32 PairwiseKeysOK; // WMI_ADD_CIPHER_KEY with the PTK's TK
        u32 GroupKeysOK;    // WMI_ADD_CIPHER_KEY with the current GTK and index
        u32 KeyMismatches;  // keys the host loaded that aren't ours
        u32 BadFrames;      // EAPOL with a bad MIC, or out of turn
        u32 Timeouts;       // handshakes the host didn't finish (we drop it)
    } WpaStats {};

private:
    melonDS::DSi& DSi;
    u32 TransferCmd;
    u32 TransferAddr;
    u32 RemSize;

    void UpdateIRQ();
    void UpdateIRQ_F1();
    //void SetIRQ_F1_Counter(u32 n);
    void ClearIRQ_F1_Counter(u32 n);
    void SetIRQ_F1_CPU(u32 n);

    u8 F0_Read(u32 addr);
    void F0_Write(u32 addr, u8 val);

    u8 F1_Read(u32 addr);
    void F1_Write(u32 addr, u8 val);

    u8 SDIO_Read(u32 func, u32 addr);
    void SDIO_Write(u32 func, u32 addr, u8 val);

    void ReadBlock();
    void WriteBlock();

    void HandleCommand();
    void BMI_Command();
    void HTC_Command();
    void WMI_Command();

    void WMI_ConnectToNetwork();
    void WMI_SendPacket(u16 len);

    void SendWMIEvent(u8 ep, u16 id, u8* data, u32 len);
    void SendWMIAck(u8 ep);
    void SendWMIBSSInfo(u8 type, u8* data, u32 len);

    void CheckRX();
    void DrainRXBuffer();

    u32 WindowRead(u32 addr);
    void WindowWrite(u32 addr, u32 val);

    u16 MB_Read16(int n)
    {
        u16 ret = Mailbox[n].Read();
        ret |= (Mailbox[n].Read() << 8);
        return ret;
    }

    void MB_Write16(int n, u16 val)
    {
        Mailbox[n].Write(val & 0xFF); val >>= 8;
        Mailbox[n].Write(val & 0xFF);
    }

    u32 MB_Read32(int n)
    {
        u32 ret = Mailbox[n].Read();
        ret |= (Mailbox[n].Read() << 8);
        ret |= (Mailbox[n].Read() << 16);
        ret |= (Mailbox[n].Read() << 24);
        return ret;
    }

    void MB_Write32(int n, u32 val)
    {
        Mailbox[n].Write(val & 0xFF); val >>= 8;
        Mailbox[n].Write(val & 0xFF); val >>= 8;
        Mailbox[n].Write(val & 0xFF); val >>= 8;
        Mailbox[n].Write(val & 0xFF);
    }

    void MB_Drain(int n)
    {
        while (!Mailbox[n].IsEmpty()) Mailbox[n].Read();
    }

    DynamicFIFO<u8> Mailbox[9];

    u8 F0_IRQEnable;
    u8 F0_IRQStatus;

    u8 F1_IRQEnable, F1_IRQEnable_CPU, F1_IRQEnable_Error, F1_IRQEnable_Counter;
    u8 F1_IRQStatus, F1_IRQStatus_CPU, F1_IRQStatus_Error, F1_IRQStatus_Counter;

    u32 WindowData, WindowReadAddr, WindowWriteAddr;

    u32 ROMID;
    u32 ChipID;
    u32 HostIntAddr;

    u8 EEPROM[0x400];
    u32 EEPROMReady;

    u32 BootPhase;

    u32 ErrorMask;
    u32 ScanTimer;

    u64 BeaconTimer;
    u32 ConnectionStatus;

    u8 LANBuffer[2048];

    void WPA_Send(u16 info, const u8* nonce, const u8* keydata, u32 kdlen);
    void WPA_Receive(const u8* frame, u32 len);
    void WPA_AddKey();
    bool WpaEnabled = false;
    bool WpaPortOpen = false;   // a 4-way handshake finished on this association
    int WpaState = 0;           // 0 idle, 1 sent message 1, 2 sent message 3, 3 keyed, 4 sent group message 1
    u32 WpaTimer = 0;
    u64 WpaReplay = 0;
    u8 WpaPMK[32], WpaANonce[32], WpaKCK[16], WpaKEK[16], WpaTK[16], WpaGTK[16];
    u8 WpaGTKIdx = 1;

    bool SendBSSInfo;
};

}
#endif // DSI_NWIFI_H
