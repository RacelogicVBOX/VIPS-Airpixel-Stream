##
# @package   VIPS/Airpixel Flexible Binary Stream Utils
# @brief     A Collection of Tools for working with VIPS/Airpixel data
# @author    Jamie Bird
# @date      August 2020
# @copyright Racelogic Ltd., 2020
#

import struct as us


class RacelogicStreamError(Exception):
    pass

class RacelogicStream():
    sample_count = 0
    valid_sample = False
    beacons      =   0   # Number of Beacons/Sats
    time         =   0.0 # Number of 10ms ticks since midnight UTC
    pos_x        =   0.0 # Position x or Latitude  (* see Note 1 at below)
    pos_y        =   0.0 # Position y or Longitude (* see Note 1 at below)
    pos_z        =   0.0 # Height (* see note 2 below)
    solution     =   0   # solution type
    kf_status    =   0   # Status of Kalman Filter
    velocity_x   =   0   # {velocity in m/s and               }
    velocity_y   =   0   # {only when 'global' bit NOT set    }
    speed_kmh    =   0   # only when global bit IS set (or VB3i Msg)
    heading      =   0   # in degrees, only when global bit IS set (or VB3i Msg)
    roll         =   0.0 # degrees
    pitch        =   0.0 # degrees
    yaw          =   0.0 # degrees
    pos_acc      =   0
    reliability  =   0 # Bitfield, see docs
    vel_acc      =   0
    posx_uncert  =   0
    posy_uncert  =   0
    posz_uncert  =   0
    roll_uncert  =   0
    pitch_uncert =   0
    yaw_uncert   =   0
    velx_uncert  =   0
    vely_uncert  =   0
    velz_uncert  =   0
    rover_id     =   0
    origin_lat   =   0.0 # Degrees
    origin_lng   =   0.0 # Degrees
    origin_alt   =   0.0 # Metres
    origin_rot   =   0.0 # Degrees
    focus        =   0 # mm
    iris         =   0 # T-Stops
    zoom         =   0 # mm
    focusRAW     =   0 # Raw Encoder Count
    irisRAW      =   0 # Raw Encoder Count
    zoomRAW      =   0 # Raw Encoder Count
    VCU_Status   =   0 # Bitfield
    FrameRate    =   0 # Enum
    LensType     =   0 # Enum
    '''
    1)  Position output is in Metres from origin when using local reference,
        but is in degrees in if using real world coordinates.
            - North is Positive, South is Negative

    2)  Height is always in M, but if using local coordinates then it is
        measured from origin, otherwise (using real world coordinates) it 
        is altitude according to the WGS84 Model
    '''


    class Utils():
        speed       = us.Struct('<h')
        position    = us.Struct('<d')
        heading     = us.Struct('<h') # Same as speed but here for clarity
        height      = us.Struct('<i')

        uInt16      = us.Struct('<H')
        uInt32      = us.Struct('<I')
        float32     = us.Struct('<f')
        double      = us.Struct('<d')

    class Mask:
        GLOBAL              = 1<<0      #0x0001
        STATUS              = 1<<1      #0x0002
        ORIENTATION         = 1<<2      #0x0004
        VELOCITY            = 1<<3      #0x0008
        VERT_VELOCITY       = 1<<4      #0x0010
        UNCERTAINTY         = 1<<5      #0x0020
        ACCURACY            = 1<<6      #0x0040
        RAW_VIPS            = 1<<7      #0x0080
        RAW_IMU             = 1<<8      #0x0100
        UNIT_ID             = 1<<9      #0x0200
        FIZ_DATA            = 1<<10     #0x0400
        ORIGIN              = 1<<11     #0x0800
        BEACON_USED         = 1<<12     #0x1000
        VCU_STATUS          = 1<<13     #0x2000
        QUARTERNIONS        = 1<<14     #0x4000
        FIZ_EXTENDED        = 1<<15     #0x8000

    class SolutionType:
        NONE           = 0x00           # No solution available (or unknown)
        GNSS           = 0x01           # GNSS only, 2D/3D fix
        DGPS           = 0x02           # GNSS solution with DGPS corrections
        RTK_FLOAT      = 0x03           # GNSS with RTK corrections (unstable, full accuracy not achieved)
        RTK_FIXED      = 0x04           # GNSS with RTK corrections (fixed)
        FIXED          = 0x05           # Fixed location
        IMU_COAST      = 0x06           # Coasting on inertial data
        VIPS           = 0x20           # VIPS solution

    class FizOptions:
        FIZ_CALIBRATED_ZOOM = 0x80000000  # Bit 31 for Zoom
        FIZ_CALIBRATED_IRIS = 0x80000000  # Bit 31 for Iris
        FIZ_CALIBRATED_FOCUS = 0x80000000  # Bit 31 for Focus
        FIZ_ZOOM_MULTIPLIER = 0x7F000000  # Bits 30-24 for Zoom Multiplier
        FIZ_CALIBRATED_MASK = 0x7FFFFFFF  # Mask to extract data without the identifier

    class VCUOptions:
        VCU_UsingTimeCode = 1<<0
        VCU_UsingGPS = 1<<1
        VCU_UsingPoE = 1<<2
        VCU_UsingBattery = 1<<3
        VCU_BatteryCharging = 1<<4
        VCU_Logging = 1<<5
        VCU_SD_AlmostFull = 1<<6

    class VCUFrameRate:
        f23_976     = 1
        f24         = 2
        f25         = 3
        f29_97      = 4
        f29_97DF    = 5
        f30         = 6
        f48         = 7
        f50         = 8
        f59_94      = 9
        f59_94DF    = 10
        f60         = 11
        FreeRun     = 0xFE
        Unknown     = 0xFF

    class VCULensType:
        Preston     = 1
        Fuji        = 2
        Canon       = 3
        Arri        = 4
        Zeiss       = 5
        # Reserved for other manufacturers
        Other       = 255

    @classmethod
    def parse_new_msg(cls, msg, checksum=False):
        """Parses a VIPS message (in standalone format) and stores it within the VIPS class.

        Args:
            msg (bytestring): The raw message read from the serial port, including header, length and checksum
            checksum (bool, optional): Validates the checksum before parsing the message. Defaults to False.
            debug (bool, optional): prints out the data at a downsampled rate. Defaults to False.
        """
        if checksum:
            if not cls.checksum_valid(msg):
                return False
        cls.valid_sample = True
        #Skip header=2, msg_len=2
        read_pos = 4
        mask = RacelogicStream.Utils.uInt32.unpack_from(msg, read_pos)[0];                         read_pos +=4
        cls.time = RacelogicStream.Utils.uInt32.unpack_from(msg, read_pos)[0];                     read_pos +=4
        cls.pos_x = RacelogicStream.Utils.double.unpack_from(msg, read_pos)[0];                    read_pos +=8
        cls.pos_y = RacelogicStream.Utils.double.unpack_from(msg, read_pos)[0];                    read_pos +=8
        cls.pos_z = RacelogicStream.Utils.float32.unpack_from(msg, read_pos)[0];                   read_pos +=4

        if mask & cls.Mask.STATUS:
            cls.beacons = int(msg[read_pos]);                                           read_pos +=1
            cls.solution_type = int(msg[read_pos]);                                     read_pos +=1
            cls.kf_status = cls.Utils.uInt16.unpack_from(msg, read_pos)[0];             read_pos +=2

        if mask & cls.Mask.ORIENTATION:
            cls.roll = cls.Utils.float32.unpack_from(msg, read_pos)[0];                 read_pos +=4
            cls.pitch = cls.Utils.float32.unpack_from(msg, read_pos)[0];                read_pos +=4
            cls.yaw = cls.Utils.float32.unpack_from(msg, read_pos)[0];                  read_pos +=4

        if mask & cls.Mask.VELOCITY:
            if mask & cls.Mask.GLOBAL:
                cls.velocity_kmh = cls.Utils.float32.unpack_from(msg, read_pos)[0];     read_pos +=4
                cls.heading = cls.Utils.float32.unpack_from(msg, read_pos)[0];          read_pos +=4
            else:
                cls.velocity_x = cls.Utils.float32.unpack_from(msg, read_pos)[0];       read_pos +=4
                cls.velocity_y = cls.Utils.float32.unpack_from(msg, read_pos)[0];       read_pos +=4

        if mask & cls.Mask.VERT_VELOCITY:
            cls.vert_velocity = cls.Utils.float32.unpack_from(msg, read_pos)[0];        read_pos +=4

        if mask & cls.Mask.UNCERTAINTY:
            cls.posx_uncert = cls.Utils.float32.unpack_from(msg, read_pos)[0];          read_pos +=4
            cls.posy_uncert = cls.Utils.float32.unpack_from(msg, read_pos)[0];          read_pos +=4
            cls.posz_uncert = cls.Utils.float32.unpack_from(msg, read_pos)[0];          read_pos +=4
            if mask & cls.Mask.ORIENTATION:
                cls.roll_uncert  = cls.Utils.float32.unpack_from(msg, read_pos)[0];     read_pos +=4
                cls.pitch_uncert = cls.Utils.float32.unpack_from(msg, read_pos)[0];     read_pos +=4
                cls.yaw_uncert   = cls.Utils.float32.unpack_from(msg, read_pos)[0];     read_pos +=4
            if mask & cls.Mask.VELOCITY:
                cls.velx_uncert = cls.Utils.float32.unpack_from(msg, read_pos)[0];      read_pos +=4
                cls.vely_uncert = cls.Utils.float32.unpack_from(msg, read_pos)[0];      read_pos +=4
                cls.velz_uncert = cls.Utils.float32.unpack_from(msg, read_pos)[0];      read_pos +=4

        if mask & cls.Mask.ACCURACY:
            cls.pos_acc = int(msg[read_pos]);                                           read_pos +=1
            cls.reliability = int(msg[read_pos]);                                       read_pos +=1
            cls.vel_acc = int(msg[read_pos]);                                           read_pos +=1
            cls.rover_id = int(msg[read_pos]);                                          read_pos +=1

        if mask & cls.Mask.RAW_VIPS:
            read_pos += 24 #Undocumented: Racelogic Debug

        if mask & cls.Mask.RAW_IMU:
            read_pos += 24 #Undocumented: Racelogic Debug

        if mask & cls.Mask.UNIT_ID:
            read_pos += 3 # Radio ID
            cls.rover_id = int(msg[read_pos]);                                          read_pos +=1

        if mask & cls.Mask.FIZ_DATA: # Old FIZ Structure
            cls.focus = cls.Utils.uInt32.unpack_from(msg, read_pos)[0];     read_pos +=4
            cls.iris = cls.Utils.uInt16.unpack_from(msg, read_pos)[0];      read_pos +=2
            cls.zoom = cls.Utils.uInt16.unpack_from(msg, read_pos)[0];      read_pos +=2

        if mask & cls.Mask.ORIGIN:
            read_pos += 24

        if mask & cls.Mask.BEACON_USED:
            cls.beacons_used = msg[read_pos:read_pos+12]

        if mask & cls.Mask.VCU_STATUS:
            cls.FrameRate = int(msg[read_pos]);                                         read_pos +=1
            cls.LensType = int(msg[read_pos]);                                          read_pos +=1
            cls.VCU_Status = int(msg[read_pos]);                                        read_pos +=1
            read_pos += 1

        if mask & cls.Mask.QUARTERNIONS:
            cls.quart_x = cls.Utils.float32.unpack_from(msg, read_pos)[0];              read_pos +=4
            cls.quart_i = cls.Utils.float32.unpack_from(msg, read_pos)[0];              read_pos +=4
            cls.quart_j = cls.Utils.float32.unpack_from(msg, read_pos)[0];              read_pos +=4
            cls.quart_k = cls.Utils.float32.unpack_from(msg, read_pos)[0];              read_pos +=4

        if mask & cls.Mask.FIZ_EXTENDED:
            # Parse Focus
            focus_data = cls.Utils.uInt32.unpack_from(msg, read_pos)[0]
            if focus_data & cls.FizOptions.FIZ_CALIBRATED_FOCUS:
                cls.focus = (focus_data & cls.FizOptions.FIZ_CALIBRATED_MASK) / 100
            else:
                cls.focusRAW = focus_data
            read_pos += 4

            # Parse Iris
            iris_data = cls.Utils.uInt32.unpack_from(msg, read_pos)[0]
            if iris_data & cls.FizOptions.FIZ_CALIBRATED_IRIS:
                cls.iris = (iris_data & cls.FizOptions.FIZ_CALIBRATED_MASK) / 100
            else:
                cls.irisRAW = iris_data
            read_pos += 4

            # Parse Zoom
            zoom_data = cls.Utils.uInt32.unpack_from(msg, read_pos)[0]
            if zoom_data & cls.FizOptions.FIZ_CALIBRATED_ZOOM:
                multiplier = (zoom_data & cls.FizOptions.FIZ_ZOOM_MULTIPLIER) >> 24
                cls.zoom = ((zoom_data & 0x00FFFFFF) * multiplier) / 100
            else:
                cls.zoomRAW = zoom_data & 0x00FFFFFF
            read_pos += 4

        cls.sample_count +=1

        return True

    @classmethod
    def checksum_valid(cls, msg):
        """Confirms the checksum is valid

        Args:
            msg (bytestring): the entire message, including header, length and checksum bytes

        Returns:
            bool: checksum valid
        """
        CRC = cls.create_checksum(msg[:-2])

        if (msg[-2] == ((CRC >> 8) & 0xFF)):
            if (msg[-1] == (CRC & 0xFF)):
                return True
            else:
                return False
        else:
            return False

    @staticmethod
    def create_checksum(msg):
        """create a new checksum for a message buffer

        Args:
            msg (bytestring): the entire message, including header and length

        Returns:
            int: 16-bit checksum
        """
        Polynomial = 0x1021
        CRC = 0
        for byte in range(len(msg)):
            CRC = CRC ^ (msg[byte] << 8)
            CRC = CRC % 0x010000
            for _ in range(8):
                if ((CRC & 0x8000) == 0x8000):
                    CRC = CRC << 1
                    CRC = CRC ^ Polynomial
                else:
                    CRC = CRC << 1

            CRC = CRC % 0x010000

        # return ((CRC >> 8) & 0xFF), (CRC & 0xFF)

        return CRC


    @classmethod
    def get_mask_options(cls, mask):
        """returns a list of the mask options currently enabled in this message

        Args:
            mask (int): mask contained in the flexible vips message

        Returns:
            list: string names for mask options
        """
        return [x for x in  [name for name in dir(cls.Mask) if not name.startswith('_')] if (mask&getattr(cls.Mask, x))]

    @classmethod
    def decode_kf_status(cls, status):
        """parse the Kalman Filter Status into human readable status

        Args:
            status (int): Kalman Filter Status

        Returns:
            str: human readable KF status
        """
        if (status == 0x0040):
            return "Reset / Startup"
        elif(status == 0x007E):
            return "Disabled"
        elif(status == 0x007F):
            return "Looking for IMU"

        testValue = (status & 0xE634)
        if (testValue == 0x0014):
            return "Waiting for VIPS"
        elif ((testValue & 0x0400) == 0x0400):
            return "ZUPT active"
        elif ((testValue == 0x0004) or (testValue == 0x0024)):
            return "No IMU data"
        elif (testValue == 0x0034):
            return "Static initialisation"
        elif (testValue == 0x0234):
            return "Ready for motion"
        elif ((testValue == 0x2234) or (testValue == 0x4234)):
            return "KF Starting"
        elif ((testValue & 0x8FFF) == 0x8034):
            return "Running"
        else:
            return "Unknown"








#end
