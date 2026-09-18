using System.Runtime.InteropServices;

namespace ViperCore
{
    /// <summary>
    /// P/Invoke wrapper around the ROBOTIS Dynamixel SDK **C** library
    /// (`dxl_x86_c.dll`). The SDK ships in two flavours: a C++ API built on
    /// classes (PortHandler, PacketHandler, GroupSyncWrite…) and this flat C
    /// API, meant to be wrapped from other languages. The signatures below are
    /// the C ones.
    ///
    /// None of the read/write calls takes an error argument: the outcome of the
    /// last transaction is read separately with <see cref="GetLastTxRxResult"/>
    /// and <see cref="GetLastRxPacketError"/>. An earlier version of this file
    /// declared a trailing `ref byte dxlError` on ping/read/write, which the C
    /// functions do not have. Under Cdecl that does not crash — the caller
    /// cleans the stack — but the argument was never written, so every error
    /// check that read it was dead code.
    ///
    /// Note also that `ping` and `pingGetModelNum` are the same symbol in the
    /// DLL (verified with dumpbin: both export at address 0x2E50). `ping` takes
    /// three arguments and returns the model number.
    ///
    /// SDK: https://github.com/ROBOTIS-GIT/DynamixelSDK
    /// </summary>
    internal static class DynamixelSDK
    {
        private const string Dll = "dxl_x86_c";
        private const CallingConvention CC = CallingConvention.Cdecl;

        // ── Port ─────────────────────────────────────────────────────────────

        [DllImport(Dll, EntryPoint = "portHandler", CallingConvention = CC,
                   CharSet = CharSet.Ansi)]
        public static extern int PortHandler(string portName);

        [DllImport(Dll, EntryPoint = "openPort", CallingConvention = CC)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool OpenPort(int portNum);

        [DllImport(Dll, EntryPoint = "closePort", CallingConvention = CC)]
        public static extern void ClosePort(int portNum);

        [DllImport(Dll, EntryPoint = "setBaudRate", CallingConvention = CC)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool SetBaudRate(int portNum, int baudRate);

        [DllImport(Dll, EntryPoint = "packetHandler", CallingConvention = CC)]
        public static extern void PacketHandler();

        // ── Single-motor transactions ────────────────────────────────────────

        [DllImport(Dll, EntryPoint = "ping", CallingConvention = CC)]
        public static extern ushort Ping(int portNum, int protocol, byte id);

        [DllImport(Dll, EntryPoint = "read1ByteTxRx", CallingConvention = CC)]
        public static extern byte Read1ByteTxRx(int portNum, int protocol, byte id, ushort address);

        [DllImport(Dll, EntryPoint = "read2ByteTxRx", CallingConvention = CC)]
        public static extern ushort Read2ByteTxRx(int portNum, int protocol, byte id, ushort address);

        [DllImport(Dll, EntryPoint = "read4ByteTxRx", CallingConvention = CC)]
        public static extern uint Read4ByteTxRx(int portNum, int protocol, byte id, ushort address);

        [DllImport(Dll, EntryPoint = "write1ByteTxRx", CallingConvention = CC)]
        public static extern void Write1ByteTxRx(int portNum, int protocol, byte id, ushort address, byte data);

        [DllImport(Dll, EntryPoint = "write2ByteTxRx", CallingConvention = CC)]
        public static extern void Write2ByteTxRx(int portNum, int protocol, byte id, ushort address, ushort data);

        [DllImport(Dll, EntryPoint = "write4ByteTxRx", CallingConvention = CC)]
        public static extern void Write4ByteTxRx(int portNum, int protocol, byte id, ushort address, uint data);

        // ── Outcome of the last transaction ──────────────────────────────────

        [DllImport(Dll, EntryPoint = "getLastTxRxResult", CallingConvention = CC)]
        public static extern int GetLastTxRxResult(int portNum, int protocol);

        [DllImport(Dll, EntryPoint = "getLastRxPacketError", CallingConvention = CC)]
        public static extern byte GetLastRxPacketError(int portNum, int protocol);

        // ── Group sync write: one packet writes the same register on many IDs

        [DllImport(Dll, EntryPoint = "groupSyncWrite", CallingConvention = CC)]
        public static extern int GroupSyncWrite(int portNum, int protocol,
            ushort startAddress, ushort dataLength);

        [DllImport(Dll, EntryPoint = "groupSyncWriteAddParam", CallingConvention = CC)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool GroupSyncWriteAddParam(int groupNum, byte id,
            uint data, ushort dataLength);

        [DllImport(Dll, EntryPoint = "groupSyncWriteTxPacket", CallingConvention = CC)]
        public static extern void GroupSyncWriteTxPacket(int groupNum);

        [DllImport(Dll, EntryPoint = "groupSyncWriteClearParam", CallingConvention = CC)]
        public static extern void GroupSyncWriteClearParam(int groupNum);

        // ── Group sync read: one packet reads the same block from many IDs ───

        [DllImport(Dll, EntryPoint = "groupSyncRead", CallingConvention = CC)]
        public static extern int GroupSyncRead(int portNum, int protocol,
            ushort startAddress, ushort dataLength);

        [DllImport(Dll, EntryPoint = "groupSyncReadAddParam", CallingConvention = CC)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool GroupSyncReadAddParam(int groupNum, byte id);

        [DllImport(Dll, EntryPoint = "groupSyncReadTxRxPacket", CallingConvention = CC)]
        public static extern void GroupSyncReadTxRxPacket(int groupNum);

        [DllImport(Dll, EntryPoint = "groupSyncReadIsAvailable", CallingConvention = CC)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool GroupSyncReadIsAvailable(int groupNum, byte id,
            ushort address, ushort dataLength);

        [DllImport(Dll, EntryPoint = "groupSyncReadGetData", CallingConvention = CC)]
        public static extern uint GroupSyncReadGetData(int groupNum, byte id,
            ushort address, ushort dataLength);

        [DllImport(Dll, EntryPoint = "groupSyncReadClearParam", CallingConvention = CC)]
        public static extern void GroupSyncReadClearParam(int groupNum);

        // ── Constants ────────────────────────────────────────────────────────

        public const int COMM_SUCCESS = 0;
        public const int PROTOCOL     = 2;   // Protocol 2.0 — XM/XH series
    }
}
