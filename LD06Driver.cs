using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Text;

namespace LD06_Driver
{
    class LD06Driver
    {
        public int READ_BUFFER_SIZE = 5000;

        bool _continue = true;
        SerialPort _serialPort;
        private static int frameCounter = 0;
        public static LidarPacket lidarPacket;
        
        /// <summary>
        /// Angle at which processing and logging begins
        /// </summary>
        public int StartAngle { get; set; }

        /// <summary>
        /// Angle at which processing and logging stops
        /// </summary>
        public int StopAngle { get; set; }

        Queue<PointData> outputQueue;
        int outputQueueSize = 360;

        public static byte[] CrcTable = new byte[]{
            0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
            0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
            0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
            0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
            0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
            0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
            0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
            0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
            0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
            0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
            0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
            0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
            0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
            0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
            0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
            0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
            0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
            0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
            0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
            0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
            0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
            0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
        };

        /// <summary>
        /// Constructor. Takes an instance of serial port that is configured for the lidar
        /// </summary>
        public LD06Driver(SerialPort port)
        {
            _serialPort = port;
            READ_BUFFER_SIZE = _serialPort.ReadBufferSize;
        }

        public void AddQueue(Queue<PointData> queue, int size)
        {
            outputQueue = queue;
            outputQueueSize = size;
        }

        public void ReadLidarData()
        {
            byte[] readBuffer = new byte[READ_BUFFER_SIZE];             //read buffer
            Queue<byte> readQueue = new Queue<byte>();      //read queue (used to pull one byte at a time)

            while (_continue)
            {
                try
                {
                    //Read serial only if there's more than 5000 bytes
                    if (_serialPort.BytesToRead < READ_BUFFER_SIZE) continue;

                    int bytesCount = _serialPort.Read(readBuffer, 0, READ_BUFFER_SIZE);

                    foreach (byte readByte in readBuffer) readQueue.Enqueue(readByte);

                    while (readQueue.Count > 100)
                    {

                        //Fetch single byte from a buffer
                        byte dataByte = readQueue.Dequeue();

                        /*
                         * starting character：Length 1 Byte, fixed value 0x54, means the beginning of data packet;
                         */
                        if (dataByte == 0x54 && frameCounter == 0)
                        {
                            //Console.WriteLine("\n\nstart byte received");
                            frameCounter = 1;                                               //go to data length                            

                            lidarPacket = new LidarPacket { header = 0x54 };
                            lidarPacket.packetRaw.Add(0x54);
                        }

                        /*
                         * Data Length: Length 1 Byte, the first three digits reserved, the last five digits represent the number of measured points in a packet, currently fixed value 12;
                         */
                        //data length
                        else if (frameCounter == 1)
                        {
                            lidarPacket.dataLength = dataByte & 0x0F;
                            lidarPacket.packetRaw.Add(dataByte);
                            frameCounter++;
                        }

                        /*
                         * Radar speed：Length 2 Byte, in degrees per second;
                         */
                        //Radar Speed /LSB
                        else if (frameCounter == 2)
                        {
                            lidarPacket.radarSpeed |= dataByte;
                            lidarPacket.packetRaw.Add(dataByte);
                            frameCounter++;
                        }

                        //Radar Speed /MSB
                        else if (frameCounter == 3)
                        {
                            lidarPacket.radarSpeed |= dataByte << 8;
                            lidarPacket.packetRaw.Add(dataByte);
                            frameCounter++;
                        }

                        /*
                         * Start angle: Length: 2 Byte; unit: 0.01 degree;
                         */
                        //Start angle /LSB
                        else if (frameCounter == 4)
                        {
                            lidarPacket.startAngle |= dataByte;
                            lidarPacket.packetRaw.Add(dataByte);
                            frameCounter++;
                        }

                        //Start angle /MSB
                        else if (frameCounter == 5)
                        {
                            lidarPacket.startAngle |= dataByte << 8;
                            lidarPacket.packetRaw.Add(dataByte);
                            frameCounter++;
                        }

                        /*
                         * A measurement data length is 3 bytes
                         */
                        //Data
                        else if (frameCounter == 6)
                        {
                            //Add raw data to the lidar packet
                            for (int j = 0; j < lidarPacket.dataLength * 3; j++) // 3 bytes in each data bin
                            {
                                lidarPacket.data.Add(dataByte);                           //read angle data character
                                lidarPacket.packetRaw.Add(dataByte);

                                if (j != lidarPacket.dataLength * 3 - 1)                //do not extract byte from the string on the last iteration
                                {
                                    dataByte = readQueue.Dequeue();
                                }
                            }

                            frameCounter++;
                        }

                        /*
                         * End Angle: Length: 2 Byte; unit: 0.01 degree
                         */
                        //End angle /LSB
                        else if (frameCounter == 7)
                        {
                            lidarPacket.endAngle |= dataByte;
                            lidarPacket.packetRaw.Add(dataByte);
                            frameCounter++;
                        }

                        //End angle /MSB
                        else if (frameCounter == 8)
                        {
                            lidarPacket.endAngle |= dataByte << 8;
                            lidarPacket.packetRaw.Add(dataByte);
                            frameCounter++;
                        }

                        /*
                         * Timestamp: Length 2 Bytes in ms, recount if reaching to MAX 30000
                         */
                        //Timesamp /LSB
                        else if (frameCounter == 9)
                        {
                            lidarPacket.timestamp |= dataByte;
                            lidarPacket.packetRaw.Add(dataByte);
                            frameCounter++;
                        }

                        //Timestamp /MSB
                        else if (frameCounter == 10)
                        {
                            lidarPacket.timestamp |= dataByte << 8;
                            lidarPacket.packetRaw.Add(dataByte);
                            frameCounter++;
                        }

                        /*
                         * CRC check：Checksum of all previous data
                         */
                        //CRC Check
                        else if (frameCounter == 11)
                        {
                            lidarPacket.crcCheck = dataByte;
                            lidarPacket.packetRaw.Add(dataByte);

                            frameCounter = 0;                               //Reset frame counter

                            var crcCalculated = lidarPacket.CalCRC8();


                            //Debug output
                            if (lidarPacket != null && crcCalculated == lidarPacket.crcCheck)
                            {
                                lidarPacket.CalculateAngles(StartAngle, StopAngle);

                                //Console.WriteLine("Data Length: " + lidarPacket.dataLength);
                                //Console.WriteLine("Speed deg/sec: " + lidarPacket.radarSpeed);
                                //Console.WriteLine("Start Angle: " + (double)lidarPacket.startAngle / 100);
                                //Console.WriteLine("End Angle: " + (double)lidarPacket.endAngle / 100);
                                //Console.WriteLine("Timestamp: " + lidarPacket.timestamp);
                                //Console.WriteLine("CRC Check: " + lidarPacket.crcCheck);
                                //Console.WriteLine("CRC Calc: " + crcCalculated);

                                lidarPacket.pointData.ForEach(point =>
                                {
                                    //if (point.angle < 2) Console.WriteLine(point.angle.ToString("0.##") + " " + point.distance + " " + point.confidence);

                                    //Add point to the output queue
                                    lock (outputQueue)
                                    {
                                        outputQueue?.Enqueue(point);

                                        if (outputQueue != null && outputQueue.Count > outputQueueSize) { outputQueue.Dequeue(); Console.WriteLine("queue overflow"); }
                                    }
                                });


                            }
                            else
                            {
                                Console.WriteLine("CRC Error!");
                            }
                        }
                    }

                }
                catch (TimeoutException) { }
            }
        }
    }

    class LidarPacket
    {
        public byte header;
        public int dataLength;
        public int radarSpeed;
        public int startAngle;
        public List<byte> data;     //data bytes(distance and confidence data in raw format)
        public int endAngle;
        public int timestamp;
        public byte crcCheck;

        public List<PointData> pointData;

        //Raw packet bytes received (used to calculate CRC)
        public List<byte> packetRaw;

        public LidarPacket()
        {
            packetRaw = new List<byte>();
            data = new List<byte>();
        }

        public byte CalCRC8() //string packet, byte len)
        {
            //convert buffer queue to array
            byte[] packet = packetRaw.ToArray();

            //debug output
            //var hexString = BitConverter.ToString(packet);
            //Console.WriteLine("Packet: " + hexString);
            //Console.WriteLine("Packet len: " + packet.Length);

            byte crc = 0;

            for (int i = 0; i < packet.Length - 1; i++)
            {
                crc = LD06Driver.CrcTable[(crc ^ packet[i]) & 0xff];
            }

            return crc;
        }

        public void CalculateAngles(int startAngleSetting, int stopAngleSetting)
        {
            pointData = new List<PointData>();

            var step = (((float)endAngle - startAngle) / 100) / (dataLength - 1);


            //calculate angle values for every 3 bytes
            for (int i = 0; i < dataLength; i++)
            {
                var newPoint = new PointData();

                newPoint.distance |= data[i * 3];
                newPoint.distance |= data[i * 3 + 1] << 8;
                newPoint.confidence = data[i * 3 + 2];
                newPoint.angle = ((float)startAngle / 100) + step * i;

                pointData.Add(newPoint);
            }
        }
    }

    public class PointData
    {
        public double angle;
        public int distance;
        public int confidence;
    }
}
