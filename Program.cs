using System;
using System.Globalization;
using System.IO.Ports;
using System.Text;
using System.Threading;
using System.Linq;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace LD06_Driver
{
    class Program
    {
        static SerialPort _serialPort;
        static bool _continue;
        static Queue<PointData> _pointDatas;

        public static void Main()
        {

            // Create a new SerialPort object with default settings.
            _serialPort = new SerialPort();

            // Allow the user to set the appropriate properties.
            _serialPort.PortName = "COM5";//"/dev/ttyUSB0";
            _serialPort.BaudRate = 230400;
            _serialPort.Parity = Parity.None;
            _serialPort.DataBits = 8;
            _serialPort.StopBits = StopBits.One;
            _serialPort.Handshake = Handshake.None;
            _serialPort.ReadBufferSize = 2000;

            // Set the read/write timeouts
            _serialPort.ReadTimeout = 500;
            _serialPort.WriteTimeout = 500;

            _serialPort.Open();

            LD06Driver lidarDriver = new LD06Driver(_serialPort);

            _pointDatas = new Queue<PointData>();

            lidarDriver.AddQueue(_pointDatas, 5000);
            lidarDriver.StopAngle = 70;
            lidarDriver.StartAngle = 290;

            Thread lidarReaderThread = new Thread(lidarDriver.ReadLidarData);

            _continue = true;

            lidarReaderThread.Start();

            while (_continue)
            {
                //Console.WriteLine("Hello world!");

                //var message = Console.ReadLine();
                while (_pointDatas.Count > 0)
                {
                    PointData point;

                    lock (_pointDatas) point = _pointDatas.Dequeue();

                    if (point != null && (point.angle < 70 || point.angle > 290)) Console.WriteLine(point.angle.ToString("0.##") + " " + point.distance + " " + point.confidence);
                }

                Task.Delay(1).Wait();
            }

            lidarReaderThread.Join();

            _serialPort.Close();
        }      
    } 
}
