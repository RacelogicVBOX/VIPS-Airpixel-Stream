# Fake Position Simulator

A Windows tool that generates the same position stream a real VIPS rover or Airpixel VCU, over a serial port or UDP. Use it to develop and test your software or parser before you have hardware, or to reproduce a problem on a bench.

## Requirements

- Windows 10 or later
- [.NET 8 Desktop Runtime](https://dotnet.microsoft.com/download/dotnet/8.0) (x64). Windows will prompt you to install it if it is missing.

If installing the runtime is awkward on your machine, use the self-contained build attached to the [latest release](../../releases/latest) instead. It is a larger download but has no prerequisites; the first launch takes a few seconds longer while it unpacks.

Both builds are signed by Racelogic Ltd, so you can check the publisher in the file's Properties before running it. The SHA256 of the v1.3.0 self-contained zip is:

```
C508BE7887A62603EE4101C690096962E4B55ADD2743F509B87CFEE9F580A114  FakePositionSimulator-1.3.0-win-x64-selfcontained.zip
```

Each release publishes the checksum of its own asset; verify with `Get-FileHash <file> -Algorithm SHA256` in PowerShell.

## Quick start for Airpixel users

1. Run `Fake Position Output.exe`.
2. Open the **Profiles** menu and choose **Airpixel VCU Outdoor** (an XT tracking outdoors) or **Airpixel VCU Indoor** (VIPS beacons indoors).
3. That is it. The simulator starts broadcasting UDP on port 6580 at 50 Hz, which is what a VCU does out of the box. The packets are the binary format described in [docs/README.md](../docs/README.md), with status, orientation, accuracy, VCU status and extended FIZ data enabled.

Point your parser at UDP port 6580 and you should see a camera moving in a 2 m circle with a gentle vertical wave.

## Quick start for VIPS users

1. Run `Fake Position Output.exe`.
2. Open the **Profiles** menu and choose **Automotive**.
3. Pick your COM port at the top of the window and press **Open**.

You will get 100 Hz serial output in the VIPS binary format with global position, velocity, heading, vertical velocity, uncertainty and accuracy fields, and no camera-specific data. UDP is left off in this profile; tick Enable UDP if you want it as well.

## Using the simulator

The window is a single scrolling page. From the top:

**Position source.** Shows whether positions are being generated or replayed. Drop a VBOX `.vbo` file anywhere on this box to replay it instead of the generated motion; Stop replay returns to generated data.

**Serial output.** COM port, baud rate and the Open button, followed by the format for the serial stream (VIPS, FreeD, APXT or NMEA) and an optional log file that captures exactly what is sent.

**UDP output.** Enable UDP, choose the port, and broadcast to the whole subnet or send to one IP address. Has its own format selection and log file. Serial and UDP can run different formats at the same time.

**NMEA options.** Only shown when NMEA is selected somewhere. Adds `$GPVTG` and `$GPRMC` sentences to the standard `$GPGGA`.

**Timing.** Output rate in Hz, whether the timestamp starts at midnight or at the current time, and two ways to make the timing imperfect: random jitter in milliseconds and a clock rate error in parts per million.

**VIPS option mask.** Which optional blocks appear in the VIPS binary packet. Each checkbox shows its bit value and the packet size updates as you go. This box is only shown when VIPS is the selected format. It also holds the Test forwards compatibility tick box, described below.

**Location.** The origin in latitude, longitude and altitude, the starting position in metres, and the motion pattern: static, circle, polygon or star, plus a vertical circle or wave. Random position noise can be added here.

**Orientation, FIZ data, VCU status, System status.** Roll, pitch and yaw, which can instead face the direction of travel, face the centre of the pattern, or pan back and forth between a set angle. Then focus, iris and zoom values in calibrated or raw encoder form, the VCU flags a receiver might react to, and the beacon count, solution type and Kalman filter state.

Everything on the page can be saved with File > Save Settings and reloaded later. Save the file into the `Profiles` folder next to the executable and it will appear in the Profiles menu the next time the simulator starts.

## Testing your parser

Tick **Test forwards compatibility** in the VIPS option mask panel. The simulator then appends one to three fake fields to every packet, on mask bits that this protocol has not defined, which is exactly what a future firmware release will do. The panel shows which bits and sizes it picked and the full mask going out on the wire. Untick and retick for a different set.

A parser written to the rules in the Forwards compatibility section of [docs/README.md](../docs/README.md) carries on decoding every real field and validating the checksum with no change at all. If yours starts reporting checksum errors or garbage values while the box is ticked, it is working out the message length from its own table of fields instead of reading the length field, and it will break the day a real field is added.

## Command line

The first argument is either a profile name or a path to a settings file. Either way the settings are applied on startup and UDP begins if the profile has it enabled. Serial output also starts if the file names a COM port that exists on the machine.

```
"Fake Position Output.exe" "Airpixel VCU Outdoor"
"Fake Position Output.exe" "Automotive"
"Fake Position Output.exe" "C:\test\my-settings.json"
```

Profile names are matched against the `Profiles` folder next to the executable, with or without the `.json` extension.

## Profiles supplied

| Profile | Output | Rate | Notes |
| --- | --- | --- | --- |
| Airpixel VCU Indoor | VIPS binary over UDP 6580 | 50 Hz | VIPS solution, 10 beacons, Canon lens |
| Airpixel VCU Outdoor | VIPS binary over UDP 6580 | 50 Hz | RTK fixed solution, 24 satellites, Fujinon lens |
| Automotive | VIPS binary over serial | 100 Hz | Global position, velocity, heading, uncertainty |

## Notes

This is an internal Racelogic test tool that we make available as a convenience. It is provided as-is, without warranty or support. If you find a case where its output disagrees with the documentation or with real hardware, please raise a github issue, or if its urgent, let us know at <support@racelogic.co.uk>.
