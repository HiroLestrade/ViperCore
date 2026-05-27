using System.Runtime.InteropServices;

namespace ViperCore
{
    /// <summary>
    /// P/Invoke wrapper around the ROBOTIS Dynamixel SDK C library (dxl_x86_c.dll).
    /// Signatures match the official dynamixel_sdk.cs from the ROBOTIS GitHub repo.
    /// Download SDK: https://github.com/ROBOTIS-GIT/DynamixelSDK
    /// </summary>
    internal static class DynamixelSDK
    {
        private const string Dll = "dxl_x86_c";
        private const CallingConvention CC = CallingConvention.Cdecl;

        // ── Port ─────────────────────────────────────────────────────────────────

        [DllImport(Dll, EntryPoint = "portHandler",   CallingConvention = CC)]
        public static extern int  PortHandler(string portName);

        [DllImport(Dll, EntryPoint = "openPort",      CallingConvention = CC)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool OpenPort(int portNum);

        [DllImport(Dll, EntryPoint = "closePort",     CallingConvention = CC)]
        public static extern void ClosePort(int portNum);

        [DllImport(Dll, EntryPoint = "setBaudRate",   CallingConvention = CC)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool SetBaudRate(int portNum, int baudRate);

        // ── Packet handler ───────────────────────────────────────────────────────
        // Call once to initialize internal state. Returns void — the protocol
        // version (1 or 2) is passed explicitly to every Tx/Rx function.

        [DllImport(Dll, EntryPoint = "packetHandler", CallingConvention = CC)]
        public static extern void PacketHandler();

        // ── Communication ────────────────────────────────────────────────────────
        // Second argument is always the protocol version integer (use PROTOCOL = 2).

        [DllImport(Dll, EntryPoint = "ping",           CallingConvention = CC)]
        public static extern int  Ping(int portNum, int protocolVersion, int id,
            ref ushort modelNumber, ref byte dxlError);

        // write functions return void
        [DllImport(Dll, EntryPoint = "write1ByteTxRx", CallingConvention = CC)]
        public static extern void Write1ByteTxRx(int portNum, int protocolVersion, int id,
            ushort address, byte data, ref byte dxlError);

        [DllImport(Dll, EntryPoint = "write4ByteTxRx", CallingConvention = CC)]
        public static extern void Write4ByteTxRx(int portNum, int protocolVersion, int id,
            ushort address, uint data, ref byte dxlError);

        // read4ByteTxRx returns the value directly as uint (NOT a result code).
        [DllImport(Dll, EntryPoint = "read4ByteTxRx",  CallingConvention = CC)]
        public static extern uint Read4ByteTxRx(int portNum, int protocolVersion, int id,
            ushort address, ref byte dxlError);

        // Returns the comm result of the last Tx/Rx operation on this port.
        [DllImport(Dll, EntryPoint = "getLastTxRxResult", CallingConvention = CC)]
        public static extern int GetLastTxRxResult(int portNum, int protocolVersion);

        // Returns the hardware error byte from the last received status packet.
        [DllImport(Dll, EntryPoint = "getLastRxPacketError", CallingConvention = CC)]
        public static extern byte GetLastRxPacketError(int portNum, int protocolVersion);

        // ── Constants ────────────────────────────────────────────────────────────

        public const int COMM_SUCCESS = 0;
        public const int PROTOCOL     = 2;   // Protocol 2.0 — XM/XH series
    }
}
