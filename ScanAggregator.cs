using System;
using System.Globalization;
using System.Text;

namespace LD06_Driver
{
    /// <summary>
    /// Reduces the lidar's 4500 points a second to the nearest distance per degree of the field of view,
    /// in a fixed amount of memory. Each degree remembers its reading for a few revolutions.
    /// </summary>
    internal sealed class ScanAggregator
    {
        public const int ZoneCount = 4;

        /// <summary>Reported for a zone or degree with nothing in range. Beyond the LD06's 12 m reach.</summary>
        public const int NoObstacle = 65535;

        private const int MinimumConfidence = 200;
        private const long KeepMs = 250;        // 2.5 revolutions at the usual 10 Hz
        private const long RevolutionMs = 100;

        private readonly int _start;
        private readonly int _width;
        private readonly int[] _distance;
        private readonly long[] _readAt;
        private readonly object _lock = new();

        public ScanAggregator(int startAngle, int endAngle)
        {
            _start = Normalize(startAngle);
            var end = Normalize(endAngle);
            _width = end > _start ? end - _start : end + 360 - _start;

            _distance = new int[_width];
            _readAt = new long[_width];
            Array.Fill(_readAt, long.MinValue / 2);
        }

        public int DegreeCount => _width;

        private static int Normalize(int angle) => ((angle % 360) + 360) % 360;

        public void Add(LidarPoint point, long nowMs)
        {
            if (point.Confidence <= MinimumConfidence || point.Distance <= 0) return;

            var offset = (int)Math.Floor(point.Angle - _start);
            if (offset < 0) offset += 360;
            if (offset >= _width) return;

            lock (_lock)
            {
                // Within one revolution keep the nearest reading, after that start over
                if (nowMs - _readAt[offset] > RevolutionMs || point.Distance < _distance[offset])
                {
                    _distance[offset] = point.Distance;
                }

                _readAt[offset] = nowMs;
            }
        }

        /// <summary>The nearest distance in each of the four equal zones across the field of view, left to right.</summary>
        public int[] ZoneRanges(long nowMs)
        {
            var zones = new int[ZoneCount];
            Array.Fill(zones, NoObstacle);

            lock (_lock)
            {
                for (var degree = 0; degree < _width; degree++)
                {
                    if (nowMs - _readAt[degree] > KeepMs) continue;

                    var zone = degree * ZoneCount / _width;
                    if (_distance[degree] < zones[zone]) zones[zone] = _distance[degree];
                }
            }

            return zones;
        }

        /// <summary>
        /// The whole field of view as one compact JSON line, for drawing. Ranges are in mm per degree from start, 0 for none.
        /// </summary>
        public string ScanJson(long nowMs)
        {
            var json = new StringBuilder(32 + _width * 5);
            json.Append("{\"start\":").Append(_start.ToString(CultureInfo.InvariantCulture)).Append(",\"ranges\":[");

            lock (_lock)
            {
                for (var degree = 0; degree < _width; degree++)
                {
                    if (degree > 0) json.Append(',');
                    json.Append(nowMs - _readAt[degree] > KeepMs ? 0 : _distance[degree]);
                }
            }

            return json.Append("]}").ToString();
        }
    }
}
