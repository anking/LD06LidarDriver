using System.Collections.Generic;
using System.Linq;
using Xunit;

namespace LD06_Driver.Tests;

public class LidarPacketParserTests
{
    /// <summary>Builds a valid packet with the same distance and confidence at all 12 points.</summary>
    private static byte[] Packet(int startCentiDegrees, int endCentiDegrees, int distance, byte confidence = 250)
    {
        var packet = new byte[LidarPacketParser.PacketLength];
        packet[0] = 0x54;
        packet[1] = 0x2C;
        packet[4] = (byte)startCentiDegrees;
        packet[5] = (byte)(startCentiDegrees >> 8);
        for (var i = 0; i < LidarPacketParser.PointsPerPacket; i++)
        {
            packet[6 + i * 3] = (byte)distance;
            packet[7 + i * 3] = (byte)(distance >> 8);
            packet[8 + i * 3] = confidence;
        }
        packet[42] = (byte)endCentiDegrees;
        packet[43] = (byte)(endCentiDegrees >> 8);
        packet[46] = LidarPacketParser.Crc8(packet, 46);
        return packet;
    }

    private static List<LidarPoint> Parse(byte[] bytes, out int crcErrors)
    {
        var points = new List<LidarPoint>();
        var errors = 0;
        new LidarPacketParser(points.Add, () => errors++).Add(bytes, bytes.Length);
        crcErrors = errors;
        return points;
    }

    [Fact]
    public void The_crc_matches_the_manual_example_table()
    {
        Assert.Equal(0x00, LidarPacketParser.Crc8([0x00], 1));
        Assert.Equal(0x4d, LidarPacketParser.Crc8([0x01], 1));
    }

    [Fact]
    public void A_packet_gives_twelve_points_spread_over_its_angles()
    {
        var points = Parse(Packet(1000, 2100, 1234), out var errors);

        Assert.Equal(0, errors);
        Assert.Equal(12, points.Count);
        Assert.Equal(10.0, points[0].Angle, 6);
        Assert.Equal(11.0, points[1].Angle, 6);
        Assert.Equal(21.0, points[11].Angle, 6);
        Assert.Equal(1234, points[0].Distance);
        Assert.Equal(250, points[0].Confidence);
    }

    [Fact]
    public void A_packet_crossing_zero_keeps_its_angles_in_order()
    {
        // The old code computed a large negative step here, placing these points across the whole circle
        var points = Parse(Packet(35900, 100, 500), out _);

        Assert.All(points, p => Assert.True(p.Angle >= 359.0 || p.Angle <= 1.0, $"angle {p.Angle}"));
        Assert.Equal(359.0, points[0].Angle, 6);
        Assert.Equal(1.0, points[11].Angle, 6);
    }

    [Fact]
    public void A_bad_crc_drops_the_packet_and_is_counted()
    {
        var packet = Packet(1000, 2100, 1234);
        packet[10] ^= 0xFF;

        Assert.Empty(Parse(packet, out var errors));
        Assert.Equal(1, errors);
    }

    [Fact]
    public void The_stream_is_found_again_after_noise()
    {
        var noise = new byte[] { 0x54, 0x01, 0x54, 0x54, 0xFF };
        var bytes = noise.Concat(Packet(1000, 2100, 700)).Concat(Packet(2100, 3200, 800)).ToArray();

        var points = Parse(bytes, out var errors);

        Assert.Equal(24, points.Count);
        Assert.Equal(0, errors);
    }

    [Fact]
    public void A_packet_split_across_reads_still_parses()
    {
        var points = new List<LidarPoint>();
        var parser = new LidarPacketParser(points.Add);
        var packet = Packet(1000, 2100, 1234);

        parser.Add(packet.Take(20).ToArray(), 20);
        Assert.Empty(points);
        parser.Add(packet.Skip(20).ToArray(), 27);

        Assert.Equal(12, points.Count);
    }
}

public class ScanAggregatorTests
{
    private static LidarPoint Point(double angle, int distance, int confidence = 250) => new(angle, distance, confidence);

    [Fact]
    public void Zones_split_a_field_of_view_that_crosses_zero()
    {
        var scan = new ScanAggregator(285, 75); // 150 degrees, 37.5 each

        scan.Add(Point(290, 1000), 0);   // zone 1
        scan.Add(Point(330, 2000), 0);   // zone 2
        scan.Add(Point(5, 3000), 0);     // zone 3
        scan.Add(Point(70, 4000), 0);    // zone 4

        Assert.Equal([1000, 2000, 3000, 4000], scan.ZoneRanges(10));
    }

    [Fact]
    public void Nothing_seen_is_reported_as_no_obstacle_instead_of_crashing()
    {
        // The old driver called Min on an empty list here, which killed the process
        var scan = new ScanAggregator(285, 75);

        Assert.All(scan.ZoneRanges(1000), range => Assert.Equal(ScanAggregator.NoObstacle, range));
    }

    [Fact]
    public void Points_outside_the_field_of_view_are_ignored()
    {
        var scan = new ScanAggregator(285, 75);

        scan.Add(Point(180, 500), 0);

        Assert.All(scan.ZoneRanges(10), range => Assert.Equal(ScanAggregator.NoObstacle, range));
    }

    [Fact]
    public void Low_confidence_and_zero_distances_are_ignored()
    {
        var scan = new ScanAggregator(0, 90);

        scan.Add(Point(10, 500, confidence: 100), 0);
        scan.Add(Point(10, 0), 0);

        Assert.All(scan.ZoneRanges(10), range => Assert.Equal(ScanAggregator.NoObstacle, range));
    }

    [Fact]
    public void Readings_expire_so_a_removed_obstacle_clears()
    {
        var scan = new ScanAggregator(0, 90);

        scan.Add(Point(10, 500), 0);

        Assert.Equal(500, scan.ZoneRanges(250)[0]);
        Assert.Equal(ScanAggregator.NoObstacle, scan.ZoneRanges(251)[0]);
    }

    [Fact]
    public void A_degree_keeps_its_nearest_reading_within_a_revolution_then_starts_over()
    {
        var scan = new ScanAggregator(0, 90);

        scan.Add(Point(10.1, 500), 0);
        scan.Add(Point(10.9, 900), 10);
        Assert.Equal(500, scan.ZoneRanges(20)[0]);

        scan.Add(Point(10.5, 900), 200);
        Assert.Equal(900, scan.ZoneRanges(210)[0]);
    }

    [Fact]
    public void The_scan_line_has_one_range_per_degree()
    {
        var scan = new ScanAggregator(350, 10);

        scan.Add(Point(350.5, 1000), 0);
        scan.Add(Point(5.5, 2000), 0);

        var json = scan.ScanJson(10);

        Assert.StartsWith("{\"start\":350,\"ranges\":[1000,", json);
        var ranges = json[(json.IndexOf('[') + 1)..json.IndexOf(']')].Split(',');
        Assert.Equal(20, ranges.Length);
        Assert.Equal("2000", ranges[15]);
    }
}
