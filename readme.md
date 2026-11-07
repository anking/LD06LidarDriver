# LD06 lidar driver

Reads an [LDROBOT LD06](./docs/LDROBOT_LD06_Datasheet.pdf) lidar over serial and reports what is in front of it on a Unix socket. It is started by [gps-rider](https://github.com/anking/gps-rider).

It turns about 4500 points a second into a compact summary, 10 times a second, in a fixed amount of memory.

## Build

.NET 10 SDK. Builds are self-contained, so a Raspberry Pi needs nothing installed.

```sh
dotnet publish LD06_Driver.csproj -c Release --self-contained -r linux-arm64
```

Tagging a commit `v*` builds `linux-arm64.zip` and `linux-arm.zip` and attaches them to a GitHub release.

## Parameters

| Parameter | Meaning |
| --- | --- |
| `-p`, `--port` | Serial port, such as `/dev/ttyUSB0`. Required. |
| `-b`, `--baud-rate` | Default `230400`. |
| `-s`, `--start-angle` | Where the field of view starts. Default `290`. Angles grow clockwise, 0 is straight ahead. |
| `-e`, `--end-angle` | Where it ends. Default `70`. |
| `-x`, `--sync-socket` | Unix socket of the owner, which must be listening. Default `/tmp/lidar-sync-data.sock`. |

Exit codes: 0 when the owner goes away, 1 on a failure such as the lidar being unplugged, 2 on invalid arguments.

## Output

Lines on the socket, every 100 ms:

```
RANGE_1::1520
RANGE_2::65535
RANGE_3::800
RANGE_4::2300
SCAN::{"start":285,"ranges":[1520,1533,0,...]}
```

- `RANGE_1` to `RANGE_4`: the nearest distance in mm in four equal zones across the field of view, left to right. `65535` means nothing in range.
- `SCAN`: the nearest distance in mm per degree of the field of view, starting at `start`. `0` means no reading.

A reading must have a confidence above 200, and is kept for 250 ms.

## Tests

```sh
dotnet test --project LD06_Driver.Tests/LD06_Driver.Tests.csproj
```
