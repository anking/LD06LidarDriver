using System;
using System.Globalization;
using System.IO.Ports;
using System.Text;
using System.Threading;
using System.Linq;
using System.Collections.Generic;
using System.Threading.Tasks;
using CommandLine;

namespace LD06_Driver
{
    class Program
    {
        static SerialPort _serialPort;
        static bool _continue;
        static Queue<PointData> _pointDatas;
        static int _startAngle = 0;
        static int _endAngle = 360;

        static List<PointData> zone1 = new List<PointData>();
        static List<PointData> zone2 = new List<PointData>();
        static List<PointData> zone3 = new List<PointData>();
        static List<PointData> zone4 = new List<PointData>();
        static int zone1Start;
        static int zone2Start;
        static int zone3Start;
        static int zone4Start;
        static int zone1End;
        static int zone2End;
        static int zone3End;
        static int zone4End;
        static double zone1Distance;
        static double zone2Distance;
        static double zone3Distance;
        static double zone4Distance;


        public class CommandLineOptions
        {
            [Option('p', "port", Required = true, HelpText = "Lidar port")]
            public string Port { get; set; }
            [Option('b', "baud-rate", Required = false, HelpText = "Lidar port speed")]
            public int BaudRate { get; set; }
            [Option('r', "read-buffer-size", Required = false, HelpText = "Read buffer size")]
            public int ReadBufferSize { get; set; }
            [Option('s', "start-angle", Required = false, HelpText = "Start angle at which data will be reported")]
            public int StartAngle { get; set; }
            [Option('e', "end-angle", Required = false, HelpText = "End angle at which stop reporting data")]
            public int EndAngle { get; set; }
        }


        public static void Main(string[] args)
        {
            try
            {
                // Create a new SerialPort object with default settings.
                _serialPort = new SerialPort();

                //set dafault settings
                _serialPort.Parity = Parity.None;
                _serialPort.DataBits = 8;
                _serialPort.StopBits = StopBits.One;
                _serialPort.Handshake = Handshake.None;
                _serialPort.ReadTimeout = 500;
                _serialPort.WriteTimeout = 500;

                //Parse user provided settings
                Parser.Default.ParseArguments<CommandLineOptions>(args)
                    .WithParsed(o =>
                    {
                        _serialPort.PortName = o.Port; //"COM5";//"/dev/ttyUSB0";
                        _serialPort.BaudRate = o.BaudRate == 0 ? 230400 : o.BaudRate;
                        _serialPort.ReadBufferSize = o.ReadBufferSize == 0 ? 4096 : o.ReadBufferSize;
                        _startAngle = o.StartAngle == 0 ? 290 : o.StartAngle;
                        _endAngle = o.EndAngle == 0 ? 70 : o.EndAngle;
                    })
                    .WithNotParsed(o =>
                    {
                        foreach (Error e in o) Console.WriteLine(e.ToString());

                        throw new Exception("Params not set");
                    });

                //calculate zone statring/ending angles
                var areaWidth = (_startAngle > _endAngle ? (360 - _startAngle) + _endAngle : _endAngle - _startAngle) / 4;
                zone1Start = _startAngle;
                zone2Start = _startAngle + areaWidth >= 360 ? _startAngle + areaWidth - 360 : _startAngle + areaWidth;
                zone3Start = _startAngle + areaWidth * 2 >= 360 ? _startAngle + areaWidth * 2 - 360 : _startAngle + areaWidth * 2;
                zone4Start = _startAngle + areaWidth * 3 >= 360 ? _startAngle + areaWidth * 3 - 360 : _startAngle + areaWidth * 3;
                zone1End = zone1Start + areaWidth > 360 ? zone1Start + areaWidth - 360 : zone1Start + areaWidth;
                zone2End = zone2Start + areaWidth > 360 ? zone2Start + areaWidth - 360 : zone2Start + areaWidth;
                zone3End = zone3Start + areaWidth > 360 ? zone3Start + areaWidth - 360 : zone3Start + areaWidth;
                zone4End = zone4Start + areaWidth > 360 ? zone4Start + areaWidth - 360 : zone4Start + areaWidth;

                _serialPort.Open();

                LD06Driver lidarDriver = new LD06Driver(_serialPort);

                _pointDatas = new Queue<PointData>();

                //create a que of fixed size to store data points
                lidarDriver.AddQueue(_pointDatas, 10000);

                //Read lidar thread
                Thread lidarReaderThread = new Thread(lidarDriver.ReadLidarData);

                //Clean Expired data points
                Thread cleanDataPoints = new Thread(calculateZoneRanges);

                _continue = true;

                lidarReaderThread.Start();
                cleanDataPoints.Start();

                while (_continue)
                {
                    while (_pointDatas.Count > 0)
                    {
                        PointData point;

                        lock (_pointDatas) point = _pointDatas.Dequeue();

                        if(point != null)
                        {
                            //select only point within my range
                            if (point.angle <= _endAngle || point.angle >= _startAngle)
                            {
                                if (point.angle >= zone1Start && point.angle < zone1End)
                                {
                                    lock (zone1)
                                    {
                                        zone1.Add(point);
                                    }
                                }
                                else if (point.angle >= zone2Start && point.angle < zone2End)
                                {
                                    lock (zone2)
                                    {
                                        zone2.Add(point);
                                    }
                                }
                                else if (point.angle >= zone3Start && point.angle < zone3End)
                                {
                                    lock (zone3)
                                    {
                                        zone3.Add(point);
                                    }
                                }
                                else if (point.angle >= zone4Start && point.angle < zone4End)
                                {
                                    lock (zone4)
                                    {
                                        zone4.Add(point);
                                    }
                                }
                            }
                        }


                        //if (point != null && (point.angle <= _endAngle || point.angle >= _startAngle)) Console.WriteLine(point.angle.ToString("0.##") + " " + point.distance + " " + point.confidence);
                    }

                    Task.Delay(1).Wait();
                }

                lidarReaderThread.Join();
                cleanDataPoints.Join();

                _serialPort.Close();
            }
            catch(Exception e)
            {
                Console.WriteLine(e.Message);
            }
        }

        public static void calculateZoneRanges()
        {
            int KEEPING_LENGTH = 250;
            int MINIMAL_CONFIDENCE = 200;

            while (true)
            {
                var lastpoint = zone1.LastOrDefault();

                if (lastpoint != null)
                {
                    lock (zone1)
                    {
                        //remove point that sit longer than a second
                        zone1.RemoveAll(x => x.timestamp > lastpoint.timestamp || x.timestamp < lastpoint.timestamp - KEEPING_LENGTH);
                        var goodDistances = zone1.FindAll(x => x.confidence > MINIMAL_CONFIDENCE);
                        zone1Distance = goodDistances.Count() > 0 ? goodDistances.Average(x => x.distance) : zone1Distance / 2; //if no distance data, devide previous oe by 2
                    }
                }

                lastpoint = zone2.LastOrDefault();

                if (lastpoint != null)
                {
                    lock (zone2)
                    {
                        //remove point that sit longer than a second
                        zone2.RemoveAll(x => x.timestamp > lastpoint.timestamp || x.timestamp < lastpoint.timestamp - KEEPING_LENGTH);
                        var goodDistances = zone2.FindAll(x => x.confidence > MINIMAL_CONFIDENCE);
                        zone2Distance = goodDistances.Count() > 0 ? goodDistances.Average(x => x.distance) : zone2Distance / 2; //if no distance data, devide previous oe by 2
                    }
                }

                lastpoint = zone3.LastOrDefault();

                if (lastpoint != null)
                {
                    lock (zone3)
                    {
                        //remove point that sit longer than a second
                        zone3.RemoveAll(x => x.timestamp > lastpoint.timestamp || x.timestamp < lastpoint.timestamp - KEEPING_LENGTH);
                        var goodDistances = zone3.FindAll(x => x.confidence > MINIMAL_CONFIDENCE);
                        zone3Distance = goodDistances.Count() > 0 ? goodDistances.Average(x => x.distance) : zone3Distance / 2; //if no distance data, devide previous oe by 2
                    }
                }

                lastpoint = zone4.LastOrDefault();

                if (lastpoint != null)
                {
                    lock (zone4)
                    {
                        //remove point that sit longer than a second
                        zone4.RemoveAll(x => x.timestamp > lastpoint.timestamp || x.timestamp < lastpoint.timestamp - KEEPING_LENGTH);
                        var goodDistances = zone4.FindAll(x => x.confidence > MINIMAL_CONFIDENCE);
                        zone4Distance = goodDistances.Count() > 0 ? goodDistances.Average(x => x.distance) : zone4Distance / 2; //if no distance data, devide previous oe by 2
                    }
                }


                Console.WriteLine(zone1Distance.ToString("0.##") + " " + zone2Distance.ToString("0.##") + " " + zone3Distance.ToString("0.##") + " " + zone4Distance.ToString("0.##"));

                Task.Delay(300).Wait();
            }
        }
    } 
}
