# VIPS/Airpixel output format and advanced configuration options

- [VIPS/Airpixel output format and advanced configuration options](#vipsairpixel-output-format-and-advanced-configuration-options)
  - [Revision History](#revision-history)
  - [Intro](#intro)
  - [Output formats](#output-formats)
  - [Racelogic/Airpixel Binary output format](#racelogicairpixel-binary-output-format)
    - [Introduction](#introduction)
    - [Header](#header)
    - [Message length](#message-length)
    - [Options mask](#options-mask)
    - [Details of output fields](#details-of-output-fields)
    - [Kalman Filter Status](#kalman-filter-status)
    - [Checksum](#checksum)
    - [Example data](#example-data)

## Revision History

| Revision | Date | Change notes | Initials |
| --- | --- | --- | --- |
| 1 | 01/01/2023 | First Issue | AA |
| 1.1 | 23/04/2023 | Addition of Quaternions and Basic VCU Status Parameters | JB |
| 1.2 | 01/06/2023 | BREAKING CHANGE: Update to FIZ Format to add support for calibrated and uncalibrated lenses, as well as increase precision on Zoom parameter | JB |
| 1.3 | 01/06/2023 | Removed RS232 Title header and added Fizbox/VCU description | JB |
| 1.4 | 01/09/2024 | Undo breaking change from 2023, add new message field to support improved FIZ format, Clarify Airpixel sections, added KF status description | JB |

## Intro

This document provides the same information as the Microsoft Word document in this folder, in a friendlier format that can be displayed on web browsers and devices that can;t access Microsoft Word.

If you have any questions or issues, please do not hesitate to get in contact with racelogic at <support@racelogic.co.uk>.

## Output formats

VIPS outputs data natively via RS232 (as well as CAN, which is outside the scope of this document). The default rate is at 115200 baud, 8Bit, No Parity, 1 Stop Bit, however 460800 is also available in special modes. Talk to Racelogic before considering this.

With the use of additional Hardware (such as a Fizbox or VCU), the same format message can be output over Ethernet via UDP broadcast. If output over Ethernet, the same header bytes and checksum will be included in the data portion of the UDP Packet.

## Racelogic/Airpixel Binary output format

### Introduction

The Racelogic binary format output from VIPS/Airpixel is intended to provide a configurable output that can be set to only output the information of interest. With VIPS, the format of this message will usually be set from within the VIPS site setup software. Section 3.1 gives details of how to manually configure this option if you wish to force a specific message structure.

When connected to a Fizbox/VCU, Airpixel XT also outputs in this format, which is now the unified format for all Airpixel Output. Throughout this document, references to 'VIPS' refer to the indoor tracking solution, and references to 'Airpixel XT' refer to the outdoor GNSS tracking solution.

The general message structure is:

| Header | Message length | Options mask | Time | Location | Optional fields | Checksum |
| --- | --- | --- | --- | --- | --- | --- |
| 2 bytes | 2 bytes | 4 bytes | 4 bytes | 20 bytes | Variable length | 2 bytes |

All multi-byte values are sent in little endian order (least significant byte first).

### Header

This is a fixed 2 byte pattern of `0x24` followed by `0xD9`

### Message length

This is a 16 bit unsigned integer containing the length of the entire message including header bytes and checksum. This allows the message checksum to be verified even if the meaning of some of the options mask bits are unknown.

***Important Note:*** *Parsers should use this length to calculate the checksum without needing to know all future field options. This makes future version of the protocol backwards compatible, by only parsing the sections that are known at the time, and support for new sections can be added later if needed.*

### Options mask

This is a 32 bit long bit field, each bit indicates whether a specific data field is present in the output message or not. The enabled data fields will always be in the order of least significant bit to most significant bit (Order shown below).

Note that not all times apply to all products. Some are VIPS automotive specific, some Airpixel specific, and some are only applicable when used with an outdoor system, such as **Airpixel XT**

| Bit | Name | Usage | Size increase |
| --- | --- | --- | --- |
| 0 (0x0000\_0001) | GLOBAL | If set position and velocity are in global coordinates. If clear they are in local coordinates. | None |
| 1 (0x0000\_0002) | STATUS | Output basic status information | 4 |
| 2 (0x0000\_0004) | ORIENTATION | Output orientation information | 12 |
| 3 (0x0000\_0008) | VELOCITY | Output velocity information | 8 |
| 4 (0x0000\_0010) | VERT-VELOCITY | Output vertical velocity | 4 |
| 5 (0x0000\_0020) | UNCERTAINTY | Output detailed uncertainty values for all enabled outputs | 12 + 12 if ORIENTATION + 12 if VELOCITY |
| 6 (0x0000\_0040) | ACCURACY | Output basic output accuracy metric and rover ID | 4 |
| 7 (0x0000\_0080) | Reserved1 | Used for Racelogic diagnostics. | 24 |
| 8 (0x0000\_0100) | Reserved2 | Used for Racelogic diagnostics. | 24 |
| 9 (0x0000\_0200) | UNIT\_ID | Rover unit information | 4 |
| 10 (0x0000\_0400) | FIZ\_DATA | Output Basic FIZ data | 8 |
| 11 (0x0000\_0800) | ORIGIN | Output site origin information | 24 |
| 12 (0x0000\_1000) | BEACON\_USED | Beacons used for calculations | 12 |
| 13 (0x0000\_2000) | VCU\_STATUS | Output VCU Status | 4 |
| 14 (0x0000\_4000) | QUATERNION | Output Kalman filter orientation in Quaternions | 16 |
| 15 (0x0000\_8000) | FIZ\_EXTENDED | Output Extended FIZ data | 12 |
| 16-31 | Reserved |  |  |

### Details of output fields

Items in red are automotive specific

Items in Blue are Airpixel specific, and require the use of additional hardware, such as **VCU** or **Fizbox**.

Items in Green are only applicable when used with an outdoor system, such as **Airpixel XT**

Table 2 VIPS Output field details

| **Name** | **Size** | **Format** | **Notes** | **Included if** |
| --- | --- | --- | --- | --- |
| Header 1 | 1 | uint8\_t | 0x24 | Always |
| Header 2 | 1 | uint8\_t | 0xd9 | |
| Length | 2 | uint16\_t | Min 34. Length is full packet including header and checksum. | |
| Mask | 4 | uint32\_t | Sets which fields are included in message. See options mask section for details | |
| Time | 4 | uint32\_t | Time since midnight UTC in ms. | |
| Latitude / X location | 8 | 64 bit double precision float | If GLOBAL bit is set in options mask then these fields contain the Latitude and Longitude in degrees.  If the GLOBAL bit is clear then they contain the location in meters to the local grid. | |
| Longitude / Y location | 8 | 64 bit double precision float | If GLOBAL bit is set in options mask then these fields contain the Latitude and Longitude in degrees.  If the GLOBAL bit is clear then they contain the location in meters to the local grid.| |
| Altitude / Z location | 4 | 32 bit float | if GLOBAL bit set om options mask, then this is height above the ellipsoid in meters to WGS 84. Otherwise height in metres from local grid origin. | |
| Beacon count | 1 | uint8\_t | Number of beacons/sats used in position calculation | Mask bit STATUS set |
| Solution type | 1 | uint8\_t | 1 = 2D/3D Fix: Standard GNSS positioning.  2 = DGNSS: Improved accuracy with corrections, generally less than 1 meter.  3 = RTKFloat: Decimetre-to-meter accuracy while attempting to achieve RTK fixed.  4 = RTK Fixed: High-precision positioning with 2 cm accuracy.  6 = IMU Coast: Positioning using IMU when GNSS is unavailable, accuracy degrades over time  32 = VIPS (UWB): Tracking with 2cm accuracy indoors | Mask bit STATUS set  |
| KFStatus | 2 | uint16\_t | Kalman Filter status indication (See 2.1.6 Kalman Filter Status), key signal of system status required to understand usage. | Mask bit STATUS set |
| Roll | 4 | 32 bit float | Roll angle in degrees. | Mask bit ORIENTATION set |
| Pitch | 4 | 32 bit float | Pitch angle in degrees. | Mask bit ORIENTATION set |
| Yaw | 4 | 32 bit float | Yaw/Heading angle in degrees. (3) | Mask bit ORIENTATION set |
| Speed / Vx | 4 | 32 bit float | If GLOBAL bit is set in options mask then horizontal speed in km/h. Otherwise velocity in the X direction in m/s | Mask bit VELOCITY |
| Heading / Vy | 4 | 32 bit float | If GLOBAL bit is set in options mask then compass heading in degrees. Otherwise velocity in the Y direction in m/s | Mask bit VELOCITY |
| Vertical velocity | 4 | 32 bit float | Vertical velocity (+ = up) in m/s Requires Kalman filter to be running. | Mask bit VERT-VELOCITY |
| Position uncertainty | 12 | 3x IEEE 32 bit float | X,Y,Z standard deviation (m) | Mask bit UNCERTAINTY set |
| Orientation uncertainty | 12 | 3x IEEE 32 bit float | Roll, Pitch, Yaw standard deviations (deg/s) | Mask bits UNCERTAINTY && ORIENTATION set |
| Velocity uncertainty | 12 | 3x IEEE 32 bit float | X,Y,Z Velocity standard deviations in m/s | Mask bits UNCERTAINTY && VELOCITY set |
| Position accuracy | 1 | uint8\_t | Position residual in meters \* 20. | Mask bit ACCURACY set |
| Reliability flags | 1 | uint8\_t | See Table 3 below | Mask bit ACCURACY set |
| Velocity accuracy | 1 | uint8\_t | Velocity residual in meters \* 10  (Undefined if VELOCITY not output) | Mask bit ACCURACY set |
| Rover/Camera ID | 1 | uint8\_t | Rover ID number (1)  In Airpixel, this represents Camera ID (1) | Mask bit ACCURACY set |
| Reserved1 | 24 |  | Racelogic debug outputs | Reserved1 set |
| Reserved2 | 24 |  | Racelogic debug outputs | Reserved2 set |
| Unit info | 3 | uint24\_t | Unit unique radio ID number | Bit UNIT\_INFO set |
| Rover ID | 1 | uint8\_t | Rover ID number  In Airpixel, this represents Camera ID | Bit UNIT\_INFO set |
| Focus Distance | 4 | uint32\_t | Focus distance in mm or raw encoder count (lens dependant) (2) (4) | Bit FIZ\_DATA set (4) |
| Iris | 2 | uint16\_t | Iris aperture in 100ths of a T-Stop or raw encoder count (lens dependant) (2) (4) | Bit FIZ\_DATA set (4) |
| Focus length (zoom) | 2 | uint16\_t | Focal length in mm or raw encoder count (lens dependant) (2) (4) | Bit FIZ\_DATA set (4) |
| Origin LLA | 20 | 2x IEEE 64 bit double + 1 x IEEE 32 bit float | Site origin location in degrees latitude, degrees longitude and meters. | Bit ORIGIN set |
| Origin rotation | 4 | IEEE 32 bit float | Site rotation angle in degrees. | Bit ORIGIN set |
| Beacons used | 12 | 12x uint8\_t | List of beacon IDs (VIPS) used in position calculation in no particular order. Always padded to 12 values with 0's for unused beacons. | Bit BEACON\_USED set |
| Frame rate | 1 | uint8\_t enum | Values of 0 - 11 indicate frame rates of None, 23.976, 24, 25, 29.97, 29.97DF, 30, 48, 50, 59.94, 59.94DF, and 60 respectively. If zero/none, no sync detected  If '0xFE/254', Unknown Sync Detected  If 0xFF/255, Free Running at 100hz | Bit VCU\_STATUS set(2) |
| Lens Type | 1 | uint8\_t enum | Values are: None (0), Preston (1), Fuji (2), Canon (3), Arri (Pending Support, 4), Zeiss (Pending Support, 5), with others potentially added later. If zero/none, no Lens detected. | Bit VCU\_STATUS set(2) |
| VCU Status | 1 | uint8\_t  Bitflag Field | 1: Using Timecode  2: 0=VIPS Data, 1=GPS Data (outdoor Airpixel)  4: Power Source (0=VIN, 1=PoE)  8: Running on Backup-Battery Warning  16: Backup-Battery Charging  32: Logging Active  64: SD Card over 80% Full  128: RESERVED | Bit VCU\_STATUS set(2) |
| Reserved | 1 | uint8\_t | RESERVED | Bit VCU\_STATUS set(2) |
| Orientation in quaternions | 16 | 4 x IEEE 32 bit float | Orientation as a Quaternion x, i, j & k values | Bit QUATERNION set |
| Focus Distance | 4 | uint32\_t | Bit 31 set:   *Focus distance in 100th/s mm*   Bit 31 unset: *Raw encoder count* | Bit FIZ\_EXTENDED set (2) |
| Iris | 4 | uint32\_t | Bit 31 set:   *Iris aperture in 100th/s of a T-Stop*   Bit 31 unset: *Raw encoder count* | Bit FIZ\_EXTENDED set (2) |
| Focal Length (Zoom) | 4 | uint32\_t | Bit 31 set:   *Focal length (zoom) in 100th/s of a mm* Bit 31 unset: *Raw encoder count*   Bits 30-24 (7 bits): *Telephoto Multiplier* | Bit FIZ\_EXTENDED set (2) |
| Checksum | 2 | 2x uint8\_t | Standard VBOX checksum. Header and length included. (See Checksum) | Always |

Notes:

1. Rover ID will be 254 for the first rover, 253 for the second etc...
   - *In Airpixel this will be converted to a standard camera number by the VCU/Fizbox (1, 2 ,3 etc) if set within settings*

2. FIZ related data cannot currently be output directly from the VIPS unit, it is included in the protocol to allow external modules to add the data to the packet (such as Fizbox or VCU)

3. Orientation-Yaw is the raw Kalman filer output, when outputting global coordinates (Lat, Long) it is not rotated to allow for the site rotation. If a correctly rotated value is needed use the heading field from the velocity block.

4. 'FIZ\_DATA' field will slowly be deprecated in favour of 'FIZ\_EXTENDED' field, however both are required to be supported in Virtual Production until at least 2026.

Table 3 Reliability Flag values

| Bit | Usage | Description |
| --- | --- | --- |
| 0 (0x01) | Outside beacons | Set if the calculated position is outside a bounding box defined by the beacon locations |
| 1 (0x02) | Insufficient beacons | Set if the number of beacons is below the minimum |
| 2-6 | Reserved |  |
| 7 (0x80) | Do not use | Set if the position is outside the bounding box or had insufficient beacons. Assuming beacon count is not zero the output will still be a best estimate available but the user should switch to alternative positioning systems if available. |

### Kalman Filter Status

The KF (Kalman Filter) Status is a 16-bit value representing the state of the Kalman Filter. While some bits in this value may change frequently to indicate data updates, the overall status functions like a state-machine, determined by specific combinations of bits. During start-up, certain bits that are typically masked out (using the mask **0xE634**) are visible and can be used to determine the initial state. Once the system is operational, these bits are masked out, and the Kalman Filter transitions into a stable operating mode.

**Start-up Specific States:**

- **0x0040**: **Reset / Startup** - The Kalman Filter is initializing or resetting. This status is only visible during the early stages of system start-up and does not appear during normal operation.
- **0x007E**: **Disabled** - The Kalman Filter is disabled. No Orientation data will be available.
- **0x007F**: **Looking for IMU** - The system is searching for input from the IMU (Inertial Measurement Unit). This state occurs during start-up but may indicate an issue with IMU connectivity if it persists.

**Operational States:**

The following statuses are determined by the bitfield after masking out frequently changing bits using the mask 0xE634. These states are more stable and indicate the system's operational status.

- **0x0014**: **Waiting for Position** - The system is waiting for external position data, either from VIPS or GNSS. This is normal during start-up, but may indicate a fault if this status persists.
- **0x0004 or 0x0024**: **No IMU Data** - The Kalman Filter is not receiving IMU data, which may occur during startup, but likely indicates a fault if this status persists.
- **0x0034**: **Static Initialisation** - The Kalman Filter is initialising while the system is stationary. This typically takes 30 seconds, during which time the device must not be moved.
- **0x0234**: **Ready for Motion** - The system has completed the stationary period and is ready to be moved in a forward's direction.
- **0x2234 or 0x4234**: **KF Starting** - The initialisation is complete, and the filter is transitioning into running mode. Continue moving until the system transitions into 'running' mode
- **0x8034**: **Running** - The Kalman Filter is fully operational and processing data correctly. System will remain in 'running' state unless it detects it has reached the stationary threshold for 'ZUPT' condition (see below).
- **0x0400**: **ZUPT Active** - Zero velocity updates (ZUPT) are being used to stabilize the system's position and orientation. This only occurs when the system has determined it is stationary.

For any other values, the status is considered **Unknown** as it doesn't match a known state.

**Note**: Bits excluded by the mask (**0xE634**) change frequently during operation and do not affect the overall state. These bits primarily indicate that data is being updated but should be ignored for determining the system's core status.

### Checksum

The checksum uses the standard Racelogic VBOX CRC calculation. This is a standard CRC-16 routine with an initial value of 0 and a polynomial of 0x1021. For legacy reasons this is sent in big endian format and so should be treated as two uint8\_t values rather than a single uint16\_t value.

### Example data

> **24 D9** 36 00 **46 00 00 00** 50 5F 00 00 **D9 CE F7 53 E3 A5 0B 40** 58 39 B4 C8 76 BE F3 BF **66 66 E6 3F** 0C **20** 35 02 **CD CC 8C BF** 00 00 C0 3F **CD 4C 0C 43** 00 00 00 **FE** 2F 25

`24 D9` - Header bytes

`36 00` - Message length = 0x0036 = 54 bytes

`46 00 00 00` - Options mask = 0x0000 0046 = Accuracy, orientation, status.

`50 5F 00 00` - Time = 0x00005F50 = 24,400 = 24 seconds, 400 ms after midnight.

`D9 CE F7 53 E3 A5 0B 40` - X = 3.456000

`58 39 B4 C8 76 BE F3 BF` - Y = -1.234000

`66 66 E6 3F` - Z = 1.800000

`0C` - Beacons = 12

`20` - Solution type = 32 (VIPS only)

`35 02` - Kalman filter status = 0x0235, static initialisation

`CD CC 8C BF` - Roll = -1.100000

`00 00 C0 3F` - Pitch =1.500000

`CD 4C 0C 43`- Yaw = 140.300003

`00 00 00` - Accuracy values

`FE` - Rover ID = 254 (rover 1)

`2F 25` - Checksum
