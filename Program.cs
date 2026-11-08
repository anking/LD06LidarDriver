using System;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using CommandLine;

namespace LD06_Driver
{
    class Program
    {
        private const int ExitOk = 0;
        private const int ExitFailed = 1;
        private const int ExitInvalidArguments = 2;

        private const int PublishPeriodMs = 100;
        private const int ReadChunkSize = LidarPacketParser.PacketLength * 4;

        public class CommandLineOptions
        {
            [Option('p', "port", Required = true, HelpText = "Lidar port")]
            public string Port { get; set; }

            [Option('b', "baud-rate", Default = 230400, HelpText = "Lidar port speed")]
            public int BaudRate { get; set; }

            [Option('s', "start-angle", Default = 290, HelpText = "Angle where the field of view starts. Angles grow clockwise, 0 is straight ahead.")]
            public int StartAngle { get; set; }

            [Option('e', "end-angle", Default = 70, HelpText = "Angle where the field of view ends")]
            public int EndAngle { get; set; }

            [Option('x', "sync-socket", Default = "/tmp/lidar-sync-data.sock", HelpText = "Unix socket of the owning process")]
            public string SyncSocket { get; set; }
        }

        public static int Main(string[] args)
        {
            var parsed = Parser.Default.ParseArguments<CommandLineOptions>(args);
            if (parsed.Value == null) return ExitInvalidArguments;

            try
            {
                return Run(parsed.Value);
            }
            catch (Exception e)
            {
                Console.Error.WriteLine("Lidar driver failed: " + e.Message);
                return ExitFailed;
            }
        }

        private static int Run(CommandLineOptions options)
        {
            Console.WriteLine($"Lidar on port {options.Port}, baud rate {options.BaudRate}, field of view {options.StartAngle} to {options.EndAngle} degrees");

            using var owner = ConnectOwner(options.SyncSocket);
            using var port = new SerialPort(options.Port, options.BaudRate, Parity.None, 8, StopBits.One)
            {
                Handshake = Handshake.None,
                ReadTimeout = 500,
            };
            port.Open();

            var clock = Stopwatch.StartNew();
            var scan = new ScanAggregator(options.StartAngle, options.EndAngle);
            var crcErrors = 0L;
            var parser = new LidarPacketParser(
                point => scan.Add(point, clock.ElapsedMilliseconds),
                () => Interlocked.Increment(ref crcErrors));

            var buffer = new byte[ReadChunkSize];
            var nextPublish = clock.ElapsedMilliseconds + PublishPeriodMs;
            var nextHealthCheck = clock.ElapsedMilliseconds;
            var lastData = clock.ElapsedMilliseconds;

            while (true)
            {
                try
                {
                    // Blocks until data arrives, so this neither spins a core nor holds readings back
                    var count = port.Read(buffer, 0, buffer.Length);
                    parser.Add(buffer, count);
                    lastData = clock.ElapsedMilliseconds;
                }
                catch (TimeoutException)
                {
                    // The publishing below still has to run, and says nothing new
                }

                var now = clock.ElapsedMilliseconds;

                if (now >= nextPublish)
                {
                    Publish(owner, scan, now);
                    nextPublish = now + PublishPeriodMs;
                }

                if (now >= nextHealthCheck)
                {
                    // Left running, this process would keep the serial port away from its replacement
                    if (IsClosedByPeer(owner))
                    {
                        Console.WriteLine("The owning process went away, exiting");
                        return ExitOk;
                    }

                    if (now - lastData > 5000) Console.Error.WriteLine("No data from the lidar for 5 seconds");
                    if (Interlocked.Exchange(ref crcErrors, 0) is var errors and > 0) Console.Error.WriteLine($"{errors} lidar packets failed their CRC");

                    nextHealthCheck = now + 5000;
                }
            }
        }

        private static Socket ConnectOwner(string path)
        {
            var socket = new Socket(AddressFamily.Unix, SocketType.Stream, ProtocolType.Unspecified) { SendTimeout = 500 };

            for (var attempt = 1; ; attempt++)
            {
                try
                {
                    socket.Connect(new UnixDomainSocketEndPoint(path));
                    return socket;
                }
                catch (SocketException) when (attempt < 10)
                {
                    Thread.Sleep(1000);
                }
            }
        }

        private static bool IsClosedByPeer(Socket socket)
        {
            try
            {
                return socket.Poll(0, SelectMode.SelectRead) && socket.Available == 0;
            }
            catch (Exception e) when (e is SocketException or ObjectDisposedException)
            {
                return true;
            }
        }

        private static void Publish(Socket owner, ScanAggregator scan, long now)
        {
            var zones = scan.ZoneRanges(now);
            var lines = new StringBuilder();
            for (var zone = 0; zone < zones.Length; zone++)
            {
                lines.Append("RANGE_").Append(zone + 1).Append("::").Append(zones[zone].ToString(CultureInfo.InvariantCulture)).Append('\n');
            }

            lines.Append("SCAN::").Append(scan.ScanJson(now)).Append('\n');

            owner.Send(Encoding.ASCII.GetBytes(lines.ToString()));
        }
    }
}
