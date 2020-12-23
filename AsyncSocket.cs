/*

socket implementation description used for local sockets can be found here:
https://medium.com/@goelhardik/http-connection-to-unix-socket-from-dotnet-core-in-c-21d19ef08f8a

*/

using System;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;

namespace LD06_Driver.Communication
{
    class AsyncSocket
    {
        string _socketLocation;     //location of the interprocess socket(used to communicate with nodejs server)
        Socket _socket;             //instance of .net core socket

        public int Available
        {
            get => _socket.Available;
        }

        public AsyncSocket(string location)
        {
            _socketLocation = location;
        }

        /// <summary>
        /// connect to f9p socket for interprocess communication
        /// </summary>
        public AsyncSocket Connect()
        {
            try
            {
                Console.WriteLine("trying to connect to interprocess socket " + _socketLocation);

                //Check if file exists for connecting socket
                if (!File.Exists(_socketLocation))
                {
                    Console.WriteLine("Socket location file does not exist (" + _socketLocation + ")");

                    return this;
                    //throw new Exception("Socket file(" + _socketLocation + ") does not exist");
                }

                //set socket params for interprocess socket
                _socket = new Socket(AddressFamily.Unix, SocketType.Stream, ProtocolType.IP);

                //must be blocking
                //_socket.Blocking = false;
                _socket.SendTimeout = 500;

                //Unix endpoint creates the address where the socket exists
                _socket.Connect(new UnixEndPoint(_socketLocation)); // connect
            }
            catch (SocketException e)
            {
                //unable to connect to the socket, spit out error in a console
                Console.WriteLine("Socket connection error: " + e.Message + "\r\nStack Trace: " + e.StackTrace);
            }

            return this;
        }

        /// <summary>
        /// Disconnect the socket
        /// </summary>
        public void Disconnect()
        {
            if (_socket == null) return;

            _socket.Disconnect(false);      //false iddicates this connection cannot be reused
            _socket.Close();
        }

        /// <summary>
        /// Check if socket is connected and can be used
        /// </summary>
        /// <returns>true if socket is connected</returns>
        public bool isConnected()
        {
            if (_socket != null && _socket.Connected) return true;
            return false;
        }

        /// <summary>
        /// Sends ASCII text into socket
        /// </summary>
        /// <param name="text">Text string</param>
        /// <returns>Number of bytes sent to socket, or null if socket is not connected</returns>
        public int? Send(string text)
        {
            try
            {
                return _socket?.Send(Encoding.ASCII.GetBytes(text));
            }
            catch (SocketException e)
            {
                Console.WriteLine(e.Message);
            }

            return null;
        }

        /// <summary>
        /// Sends ASCII text string into socket terminated by \r\n
        /// </summary>
        /// <param name="text">Text string</param>
        /// <returns>Number of bytes sent to socket, or null if socket is not connected</returns>
        public int? SendLine(string text) => Send(text + "\r\n");

        /// <summary>
        /// Sends byte into socket
        /// </summary>
        /// <param name="text">Text string</param>
        /// <returns>Number of bytes sent to socket, or null if socket is not connected</returns>
        public int? Send(byte incoming)
        {
            try
            {
                return _socket?.Send(new byte[1] { incoming });
            }
            catch (SocketException e)
            {
                Console.WriteLine(e.Message);
            }

            return null;
        }

        /// <summary>
        /// Receive from socket buffer
        /// </summary>
        /// <param name="text">Text string</param>
        /// <returns>Number of bytes sent to socket</returns>
        public int Receive(byte[] buffer) => _socket.Receive(buffer);

    }
}
