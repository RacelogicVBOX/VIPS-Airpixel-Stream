#![warn(missing_docs)]

//! This library provides facilities for parsing and representing data packets
//! from Airpixel and VIPS messages, via the 'FlexibleBinaryStream' output from
//! selected Racelogic products.
//!
//! The messages are binary packets that can contain various types of
//! navigational, positional, and operational data, such as orientation, velocity,
//! uncertainty, lens information, and more. These packets can be parsed into an
//! `AirpixelVipsData` structure for further processing.
//!
//! The parsing logic uses `byteorder` for endian-aware reading of values from
//! the incoming byte stream. The library also validates message integrity via
//! checksums and checks that data length matches the options indicated by the
//! message mask fields.

use byteorder::{BigEndian, LittleEndian, ReadBytesExt};
use std::io::Read;

/// The minimum size in bytes of a packet.
/// This value is used as a basic length check before attempting to parse the packet.
pub const MIN_PACKET_SIZE: usize = 34;

/// An enum representing the various errors that can occur when parsing a FlexibleBinaryStream message.
#[derive(Debug)]
pub enum AirpixelVipsError {
    /// The message header (0x24D9) was not found at the start of the data.
    MissingHeader,
    /// The provided data buffer does not match the expected length specified in the message.
    IncorrectLength,
    /// The options mask does not match the expected payload length or contains unknown bits.
    MaskError,
    /// The checksum computed from the data does not match the one provided in the message.
    ChecksumError,
    /// An underlying I/O error occurred, e.g., reading from the buffer failed.
    IoError(std::io::Error),
}

impl From<std::io::Error> for AirpixelVipsError {
    fn from(err: std::io::Error) -> Self {
        AirpixelVipsError::IoError(err)
    }
}

impl std::fmt::Display for AirpixelVipsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::MissingHeader => write!(f, "Unable to find message start"),
            Self::IncorrectLength => write!(f, "Insufficient data length"),
            Self::MaskError => write!(f, "Unknown Mask Bytes or Length doesnt match"),
            Self::ChecksumError => write!(f, "Checksum Failed"),
            Self::IoError(_) => write!(f, "IO Error"),
        }
    }
}

/// An enumeration for various Kalman Filter (KF) related status flags.
///
/// These flags provide insight into the internal state of the navigation solution. See
/// the main documentation for further details.
#[allow(missing_docs)]
#[derive(Debug, Clone, Copy)]
pub enum KfFlags {
    Clear = 0x0000,
    NewIMUVersion = 0x0001,
    TestMode = 0x0002,
    Use = 0x0004,
    Initialised = 0x0008,

    IMUDataFound = 0x0010,
    GoodLock = 0x0020,
    Reset = 0x0040,
    CoastingTimeout = 0x0080,

    IMUFound = 0x0100,
    Initialise = 0x0200,
    ZuptActive = 0x0400,
    IMUmounting = 0x0800,

    WSSmeasureUpdate = 0x1000,
    MeasureUpdate = 0x2000,
    TimeUpdate = 0x4000,
    Active = 0x8000,
}

/// Reliability flags related to the positioning solution.
///
/// These may indicate if the solution should be considered usable or not.
#[derive(Debug, Clone, Copy)]
pub enum ReliabilityFlags {
    /// Outside of known beacon coverage (see VIPS Geofence).
    OutsideBeacons = 0x01,
    /// Insufficient number of beacons/sats for a robust solution.
    InsufficientBeacons = 0x02,
    /// The solution should not be used.
    DoNotUse = 0x80,
}

/// Flags related to the Airpixel VCU.
#[derive(Debug, Clone, Copy)]
pub enum VCUFlags {
    /// Timecode is available.
    Timecode = 0x01,
    /// GNSS data is present.
    GNSS = 0x02,
    /// Battery data is present.
    Battery = 0x04,
    /// Powered over Ethernet.
    PoE = 0x08,
    /// Battery is currently charging.
    BatteryCharging = 0x10,
    /// Logging is active.
    LoggingActive = 0x20,
    /// Media warning (No space or approaching full).
    MediaWarning = 0x40,
}

/// Frame rates as enums, representing the frame rates supported by the system.
///
/// Used primarily in Virtual Production use cases.
#[allow(non_camel_case_types, missing_docs)]
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub enum FrameRate {
    #[default]
    FPS_None = 0x00,
    FPS_23_976 = 0x01,
    FPS_24 = 0x02,
    FPS_25 = 0x03,
    FPS_29_97 = 0x04,
    FPS_29_97_DF = 0x05,
    FPS_30 = 0x06,
    FPS_47_95 = 0x07,
    FPS_48 = 0x08,
    FPS_50 = 0x09,
    FPS_59_94 = 0x0A,
    FPS_59_94_DF = 0x0B,
    FPS_60 = 0x0C,
    FPS_Unknown = 0xFE,
    FPS_Free_Run = 0xFF,
}

impl From<FrameRate> for u8 {
    fn from(rate: FrameRate) -> Self {
        rate as u8
    }
}

impl TryFrom<u8> for FrameRate {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x00 => Ok(FrameRate::FPS_None),
            0x01 => Ok(FrameRate::FPS_23_976),
            0x02 => Ok(FrameRate::FPS_24),
            0x03 => Ok(FrameRate::FPS_25),
            0x04 => Ok(FrameRate::FPS_29_97),
            0x05 => Ok(FrameRate::FPS_29_97_DF),
            0x06 => Ok(FrameRate::FPS_30),
            0x07 => Ok(FrameRate::FPS_47_95),
            0x08 => Ok(FrameRate::FPS_48),
            0x09 => Ok(FrameRate::FPS_50),
            0x0A => Ok(FrameRate::FPS_59_94),
            0x0B => Ok(FrameRate::FPS_59_94_DF),
            0x0C => Ok(FrameRate::FPS_60),
            0xFE => Ok(FrameRate::FPS_Unknown),
            0xFF => Ok(FrameRate::FPS_Free_Run),
            _ => Err(()),
        }
    }
}

impl FrameRate {
    /// Returns a string representation of the frame rate.
    pub fn to_string(&self) -> String {
        match *self {
            FrameRate::FPS_None => "None".to_string(),
            FrameRate::FPS_23_976 => "23.976".to_string(),
            FrameRate::FPS_24 => "24".to_string(),
            FrameRate::FPS_25 => "25".to_string(),
            FrameRate::FPS_29_97 => "29.97".to_string(),
            FrameRate::FPS_29_97_DF => "29.97(DF)".to_string(),
            FrameRate::FPS_30 => "30".to_string(),
            FrameRate::FPS_47_95 => "47.95".to_string(),
            FrameRate::FPS_48 => "48".to_string(),
            FrameRate::FPS_50 => "50".to_string(),
            FrameRate::FPS_59_94 => "59.94".to_string(),
            FrameRate::FPS_59_94_DF => "59.94(DF)".to_string(),
            FrameRate::FPS_60 => "60".to_string(),
            FrameRate::FPS_Unknown => "Unknown".to_string(),
            FrameRate::FPS_Free_Run => "Free Run".to_string(),
        }
    }
}

/// Lens types supported or identified by the system.
///
/// Used primarily in Virtual Production
#[allow(missing_docs)]
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub enum LensType {
    #[default]
    NoLens = 0x00,
    PrestonMdr3 = 0x01,
    Fuji = 0x02,
    Canon = 0x03,
    Arri = 0x04,
    Zeis = 0x05,
}

// Implementing conversion from LensType to u8
impl From<LensType> for u8 {
    fn from(lens: LensType) -> Self {
        lens as u8
    }
}

impl TryFrom<u8> for LensType {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x00 => Ok(LensType::NoLens),
            0x01 => Ok(LensType::PrestonMdr3),
            0x02 => Ok(LensType::Fuji),
            0x03 => Ok(LensType::Canon),
            0x04 => Ok(LensType::Arri),
            0x05 => Ok(LensType::Zeis),
            _ => Err(()),
        }
    }
}

impl LensType {
    /// Returns a string representation of the lens type.
    pub fn to_string(&self) -> String {
        match *self {
            LensType::NoLens => "None".to_string(),
            LensType::PrestonMdr3 => "Preston-MDR3".to_string(),
            LensType::Fuji => "Fuji".to_string(),
            LensType::Canon => "Canon".to_string(),
            LensType::Arri => "Arri".to_string(),
            LensType::Zeis => "Zeis".to_string(),
        }
    }
}

/// Bitfields representing which optional fields are present in the message.
///
/// Each bit corresponds to a specific data block that may or may not be present
#[derive(Debug, Clone, Copy)]
pub enum OptionField {
    /// Indicates that location and velocity data are provided in a global (latitude/longitude) frame
    /// rather than a local XYZ frame.
    Global = 0x0001,
    /// Indicates that status information (e.g., beacon count, solution type, Kalman filter status) is included.
    Status = 0x0002,
    /// Indicates that orientation data (roll, pitch, yaw) is included.
    Orientation = 0x0004,
    /// Indicates that velocity data (either global speed and heading or local velocity components) is included.
    Velocity = 0x0008,
    /// Indicates that a separate vertical velocity component is included.
    VertVelocity = 0x0010,
    /// Indicates that uncertainty data (position, optionally orientation and velocity uncertainties) is included.
    Uncertainty = 0x0020,
    /// Indicates that accuracy data (residuals and reliability) is included.
    Accuracy = 0x0040,
    /// Indicates that the first raw debug block (24 bytes) is included. Racelogic internal only
    RawDebug1 = 0x0080,
    /// Indicates that the second raw debug block (24 bytes) is included. Racelogic internal only.
    RawDebug2 = 0x0100,
    /// Indicates that unit identification information (radio ID, rover ID) is included.
    UnitId = 0x0200,
    /// Indicates that basic FIZ (Focus, Iris, Zoom) lens data is included.
    FizData = 0x0400,
    /// Indicates that origin data (reference latitude, longitude, altitude, and rotation) is included.
    Origin = 0x0800,
    /// Indicates that a list of which beacons are currently used is included.
    BeaconUsed = 0x1000,
    /// Indicates that VCU status data (frame rate, lens type, etc.) is included.
    VcuStatus = 0x2000,
    /// Indicates that orientation data is also provided as quaternions.
    Quaternion = 0x4000,
    /// Indicates that extended FIZ data is provided, including info on calibration and zoom multipliers
    FizExtended = 0x8000,
}

/// A generic 3D coordinate structure, used for positions or velocities.
#[derive(Debug, Clone, Copy, Default)]
pub struct Floatxyz {
    /// The x-component.
    pub x: f32,
    /// The y-component.
    pub y: f32,
    /// The z-component.
    pub z: f32,
}

/// Orientation data containing roll, pitch, and yaw in degrees.
#[derive(Debug, Clone, Copy, Default)]
pub struct OrientationField {
    /// Roll angle in degrees.
    pub roll: f32,
    /// Pitch angle in degrees.
    pub pitch: f32,
    /// Yaw angle in degrees.
    pub yaw: f32,
}

/// Status information including beacon count, solution type, and Kalman filter status flags.
#[derive(Debug, Clone, Copy, Default)]
pub struct StatusField {
    /// The number of beacons in view.
    pub beacon_count: u8,
    /// The type of solution currently established.
    pub solution_type: u8,
    /// Kalman filter status bitfield.
    pub kf_status: u16,
}

/// Velocity data in either global or local frame, depending on `Global` bit in `options_mask`.
#[derive(Debug, Clone, Copy, Default)]
pub struct VelocityField {
    /// Speed in km/h if Global data is present.
    pub speed_kmh: f32,
    /// Heading in degrees if Global data is present.
    pub heading: f32,
    /// Velocity in the X direction if Global flag not set.
    pub velocity_x: f32,
    /// Velocity in the Y direction if Global flag not set.
    pub velocity_y: f32,
}

/// Uncertainty fields for position, orientation, and velocity measurements.
#[derive(Debug, Clone, Copy, Default)]
pub struct UncertaintyField {
    /// Position uncertainty in meters.
    pub position: Floatxyz,
    /// Orientation uncertainty if ORIENTATION bit is set.
    pub orientation: Option<OrientationField>,
    /// Velocity uncertainty if VELOCITY bit is set.
    pub velocity: Option<Floatxyz>,
}

/// Accuracy fields providing residuals for position and velocity, and a reliability field.
/// Also contains Rover ID (VIPS)
#[derive(Debug, Clone, Copy, Default)]
pub struct AccuracyField {
    /// Position residual multiplied by 20.
    pub position: u8,
    /// Reliability indicator.
    pub reliability: u8,
    /// Velocity residual multiplied by 10.
    pub velocity: u8,
    /// Rover ID number.
    pub rover_id: u8,
}

/// Unit information including a unique radio ID and rover ID.
#[derive(Debug, Clone, Copy, Default)]
pub struct UnitInfoField {
    /// The unique radio ID number of the unit.
    pub radio_id_number: u32,
    /// Rover ID number (same as Accuracy Field).
    pub rover_id: u8,
}

/// Origin (reference) point data including latitude, longitude, altitude, and a rotation angle.
/// Allows Global to Local transform data.
#[derive(Debug, Clone, Copy, Default)]
pub struct OriginField {
    /// Latitude in degrees.
    pub latitude: f64,
    /// Longitude in degrees.
    pub longitude: f64,
    /// Altitude in meters.
    pub altitude: f32,
    /// Rotation angle in degrees.
    pub rotation: f32,
}

/// FIZ (Focus, Iris, Zoom) lens control data.
#[derive(Debug, Clone, Copy, Default)]
pub struct FizDataField {
    /// Focus distance in mm.
    pub focus: u32,
    /// Iris aperture in hundredths of a T-stop.
    pub iris: u32,
    /// Zoom focal length in mm.
    pub zoom: u32,
    /// Indicates if focus is calibrated.
    pub calibrated_focus: bool,
    /// Indicates if iris is calibrated.
    pub calibrated_iris: bool,
    /// Indicates if zoom is calibrated.
    pub calibrated_zoom: bool,
}

/// Airpixel VCU specific data, including frame rate and lens type.
#[derive(Debug, Clone, Copy, Default)]
pub struct VcuDataField {
    /// The current frame rate mode.
    pub frame_rate: FrameRate,
    /// The type of lens attached.
    pub lens_type: LensType,
    /// A status bitfield related to the VCU.
    pub status: u8,
}

/// Quaternion-based orientation data, possibly used instead of Euler angles.
#[derive(Debug, Clone, Copy, Default)]
pub struct QuaternionDataField {
    /// The quaternion `x` component.
    pub x: f32,
    /// The quaternion `i` component.
    pub i: f32,
    /// The quaternion `j` component.
    pub j: f32,
    /// The quaternion `k` component.
    pub k: f32,
}

/// The main data structure containing all information parsed from a VIPS message.
///
/// Not all fields may be populated, depending on the `options_mask`. Check if data
/// is present using the `if let Some(...)` pattern.
#[derive(Debug, Clone, Copy, Default)]
pub struct AirpixelVipsData {
    /// The total length of this message in bytes.
    pub message_length: u16,
    /// Bitmask indicating which optional data fields are included.
    pub options_mask: u32,
    /// Timestamp in milliseconds.
    pub time_ms: u32,
    /// Position X in meters if `GLOBAL` is not set.
    pub pos_x: f64,
    /// Position Y in meters if `GLOBAL` is not set.
    pub pos_y: f64,
    /// Position Z in meters if `GLOBAL` is not set.
    pub pos_z: f32,
    /// Latitude in degrees if `GLOBAL` is set.
    pub latitude: f64,
    /// Longitude in degrees if `GLOBAL` is set.
    pub longitude: f64,
    /// Altitude in meters if `GLOBAL` is set.
    pub altitude: f32,
    /// Status field if `STATUS` bit is set.
    pub status: Option<StatusField>,
    /// Orientation field if `ORIENTATION` bit is set.
    pub orientation: Option<OrientationField>,
    /// Velocity field if `VELOCITY` bit is set.
    pub velocity: Option<VelocityField>,
    /// Vertical velocity if `VERT_VELOCITY` bit is set.
    pub vert_velocity: Option<f32>,
    /// Uncertainty field if `UNCERTAINTY` bit is set.
    pub uncertainty: Option<UncertaintyField>,
    /// Accuracy field if `ACCURACY` bit is set.
    pub accuracy: Option<AccuracyField>,
    /// Unit info if `UNIT_ID` bit is set.
    pub unit_info: Option<UnitInfoField>,
    /// FIZ (Focus, Iris, Zoom) data if `FIZ_DATA` or `FIZ_EXTENDED` bit is set.
    pub fiz: Option<FizDataField>,
    /// Origin data if `ORIGIN` bit is set.
    pub origin: Option<OriginField>,
    /// Beacons used if `BEACON_USED` bit is set.
    pub beacons_used: Option<[u8; 12]>,
    /// VCU status data if `VCU_STATUS` bit is set.
    pub vcu: Option<VcuDataField>,
    /// Quaternion data if `QUATERNION` bit is set.
    pub quaternions: Option<QuaternionDataField>,
    /// The checksum of this message, validated after parsing.
    pub checksum: u16,
}

/// An internal lazy helper macro for generating inline boolean "has_xxx_data" methods on `AirpixelVipsData`.
/// Each generated method checks whether a given option bit is set in `options_mask`.
macro_rules! option_methods {
    ($($name:ident => $variant:ident),+) => {
        impl AirpixelVipsData {
            $(
                #[inline]
                fn $name(&self) -> bool {
                    (self.options_mask & OptionField::$variant as u32) != 0
                }
            )+
        }
    };
}

option_methods! {
    has_global_data => Global,
    has_status_data => Status,
    has_orientation_data => Orientation,
    has_velocity_data => Velocity,
    has_vert_velocity_data => VertVelocity,
    has_uncertainty_data => Uncertainty,
    has_accuracy_data => Accuracy,
    has_raw_debug_1=> RawDebug1,
    has_raw_debug_2 => RawDebug2,
    has_unit_id_data => UnitId,
    has_fiz_data => FizData,
    has_origin_data => Origin,
    has_beacon_used_data => BeaconUsed,
    has_vcu_data => VcuStatus,
    has_quaternion_data => Quaternion,
    has_fiz_extended_data => FizExtended
}

impl AirpixelVipsData {
    /// Returns a human-readable string describing the Kalman Filter status based on `kf_status`.
    ///
    /// This uses various bit checks to determine the current state of the KF solution.
    pub fn get_kf_string(&self) -> &str {
        if let Some(status) = self.status {
            if status.kf_status == 0xFFFF {
                //New PNT Format, not properly defined yet
                if (status.kf_status & 0x0200) == 0x0200 {
                    return "Reset";
                }
                if (status.kf_status & 0x0001) == 0 {
                    return "Looking for IMU";
                }
                if (status.kf_status & 0x0100) == 0x0100 {
                    return "IMU coast timeout";
                }

                if (status.kf_status & 0x0002) == 0 {
                    // no GPS
                    if (status.kf_status & 0x0008) == 0 {
                        // not initalised
                        return "Waiting for good GPS";
                    } else {
                        return "Lost GPS";
                    }
                }
                if (status.kf_status & 0x0400) == 0x0400 {
                    // running
                    if (status.kf_status & 0x0002) == 0 {
                        return "IMU coast"; // running and not good lock
                    } else {
                        return "Running";
                    }
                } else {
                    return "Unknown PNT";
                }
            } else {
                if status.kf_status == 0x0040 {
                    return "Reset / Startup";
                } else if status.kf_status == 0x007E {
                    return "Disabled";
                } else if status.kf_status == 0x007F {
                    return "Looking for IMU";
                }

                let kf = status.kf_status & 0xE634;
                if (kf & 0x0400) == 0x0400 {
                    return "ZUPT Active";
                }
                match kf {
                    0x0014 => "Waiting For VIPS/GNSS",
                    0x0004 | 0x0024 => "No IMU Data",
                    0x0034 => "Static Initialisation",
                    0x0234 => "Ready for Motion",
                    0x2234 | 0x4234 => "Starting",
                    _ => {
                        if kf & 0x8fff == 0x8034 {
                            return "Running";
                        } else {
                            return "Unknown";
                        }
                    }
                }
            }
        } else {
            "Unknown"
        }
    }

    /// Returns a string representation of the timestamp in HH:MM:SS:MS format,
    pub fn get_time_string(&self) -> String {
        let hours = self.time_ms / 3600000;
        let minutes = (self.time_ms / 60000) % 60;
        let seconds = (self.time_ms / 1000) % 60;
        let hundreths = self.time_ms % 1000;
        format!(
            "{:02}:{:02}:{:02}.{:03}",
            hours, minutes, seconds, hundreths
        )
    }

    /// Computes the expected length of the message based on the bits set in `options_mask`.
    fn expected_length(&self) -> u16 {
        let mut msg_len = MIN_PACKET_SIZE as u16; // Header, Message length, Options mask, Time, Location, Checksum
        if self.has_status_data() {
            msg_len += 4;
        }
        if self.has_orientation_data() {
            msg_len += 12;
        }
        if self.has_velocity_data() {
            msg_len += 8;
        }
        if self.has_vert_velocity_data() {
            msg_len += 4
        }
        if self.has_uncertainty_data() {
            msg_len += 12;
            if self.has_orientation_data() {
                msg_len += 12;
            }
            if self.has_velocity_data() {
                msg_len += 12;
            }
        }
        if self.has_accuracy_data() {
            msg_len += 4;
        }
        if self.has_raw_debug_1() {
            msg_len += 24;
        }
        if self.has_raw_debug_2() {
            msg_len += 24;
        }
        if self.has_unit_id_data() {
            msg_len += 4;
        }
        if self.has_fiz_data() {
            msg_len += 8;
        }
        if self.has_origin_data() {
            msg_len += 24;
        }
        if self.has_beacon_used_data() {
            msg_len += 12;
        }
        if self.has_vcu_data() {
            msg_len += 4;
        }
        if self.has_quaternion_data() {
            msg_len += 16;
        }
        if self.has_fiz_extended_data() {
            msg_len += 12;
        }

        return msg_len;
    }
}

/// Computes the 16-bit CRC checksum of the given message slice.
///
/// Returns the computed CRC as a `u16`. If `msg` is `None`, returns 0.
pub fn calculate_checksum(msg: Option<&[u8]>) -> u16 {
    //[u8;2]
    let mut crc: u16 = 0x0;
    match msg {
        Some(msg) => {
            for byte in msg.iter() {
                let mut x = ((crc >> 8) ^ (*byte as u16)) & 255;
                x ^= x >> 4;
                crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
            }
            // [((crc >> 8) & 0xFF).try_into().unwrap(), (crc & 0xFF).try_into().unwrap()]
        }
        None => (),
    }
    crc
}

/// Parses a raw FlexibleBinaryStream message and returns a populated `AirpixelVipsData` structure.
///
/// This function:
/// - Checks for the correct header (0x24D9).
/// - Reads the message length and verifies it matches the expected length from `options_mask`.
/// - Computes and verifies the CRC checksum.
/// - Extracts mandatory and optional data fields based on `options_mask`.
///
/// # Arguments
///
/// * `msg` - A byte slice containing a raw Airpixel VIPS packet.
///
/// # Returns
///
/// * `Ok(AirpixelVipsData)` if the parse is successful and data is valid.
/// * `Err(AirpixelVipsError)` if any validation fails or if data is insufficient.
pub fn parse_racelogic_data(mut msg: &[u8]) -> Result<AirpixelVipsData, AirpixelVipsError> {
    use OptionField::*;
    let mut vips_data = AirpixelVipsData::default();

    if msg.len() < MIN_PACKET_SIZE {
        return Err(AirpixelVipsError::IncorrectLength);
    }

    // We know the packet is at least 34 bytes so no risk of panic
    let (buf, mut checksum) = msg.split_at(msg.len() - 2);

    vips_data.checksum = calculate_checksum(Some(&buf));

    if vips_data.checksum != checksum.read_u16::<BigEndian>()? {
        return Err(AirpixelVipsError::ChecksumError);
    }

    // Check for the expected header
    if msg.read_u16::<BigEndian>()? != 0x24D9 {
        return Err(AirpixelVipsError::MissingHeader);
    }

    // Read the fixed size part of the structure
    vips_data.message_length = msg
        .read_u16::<LittleEndian>()
        .unwrap_or(MIN_PACKET_SIZE as u16);

    if (msg.len() + 4) != vips_data.message_length as usize {
        return Err(AirpixelVipsError::IncorrectLength);
    }

    vips_data.options_mask = msg.read_u32::<LittleEndian>()?;

    if vips_data.message_length != vips_data.expected_length() {
        return Err(AirpixelVipsError::MaskError);
    }

    // Read the mandatory fields
    vips_data.time_ms = msg.read_u32::<LittleEndian>()?;
    if vips_data.has_global_data() {
        vips_data.latitude = msg.read_f64::<LittleEndian>()?;
        vips_data.longitude = msg.read_f64::<LittleEndian>()?;
        vips_data.altitude = msg.read_f32::<LittleEndian>()?;
    } else {
        vips_data.pos_x = msg.read_f64::<LittleEndian>()?;
        vips_data.pos_y = msg.read_f64::<LittleEndian>()?;
        vips_data.pos_z = msg.read_f32::<LittleEndian>()?;
    }

    // Check options mask and read optional fields
    if vips_data.has_status_data() {
        let status = vips_data.status.insert(StatusField::default());
        status.beacon_count = msg.read_u8()?;
        status.solution_type = msg.read_u8()?;
        status.kf_status = msg.read_u16::<LittleEndian>()?;
    }

    if vips_data.has_orientation_data() {
        let orientation = vips_data.orientation.insert(OrientationField::default());
        orientation.roll = msg.read_f32::<LittleEndian>()?;
        orientation.pitch = msg.read_f32::<LittleEndian>()?;
        orientation.yaw = msg.read_f32::<LittleEndian>()?;
    }

    if vips_data.has_velocity_data() {
        let velocity = vips_data.velocity.insert(VelocityField::default());
        if vips_data.options_mask & (Global as u32) != 0 {
            velocity.speed_kmh = msg.read_f32::<LittleEndian>()?;
            velocity.heading = msg.read_f32::<LittleEndian>()?;
        } else {
            velocity.velocity_x = msg.read_f32::<LittleEndian>()?;
            velocity.velocity_y = msg.read_f32::<LittleEndian>()?;
        }
    }

    if vips_data.has_vert_velocity_data() {
        let _ = vips_data
            .vert_velocity
            .insert(msg.read_f32::<LittleEndian>()?);
    }

    if vips_data.has_uncertainty_data() {
        let uncertainty = vips_data.uncertainty.insert(UncertaintyField::default());
        uncertainty.position.x = msg.read_f32::<LittleEndian>()?;
        uncertainty.position.y = msg.read_f32::<LittleEndian>()?;
        uncertainty.position.z = msg.read_f32::<LittleEndian>()?;
        if vips_data.options_mask & (Orientation as u32) != 0 {
            let ori = uncertainty.orientation.insert(OrientationField::default());
            ori.roll = msg.read_f32::<LittleEndian>()?;
            ori.pitch = msg.read_f32::<LittleEndian>()?;
            ori.yaw = msg.read_f32::<LittleEndian>()?;
        }
        if vips_data.options_mask & (Velocity as u32) != 0 {
            let vel = uncertainty.velocity.insert(Floatxyz::default());
            vel.x = msg.read_f32::<LittleEndian>()?;
            vel.y = msg.read_f32::<LittleEndian>()?;
            vel.z = msg.read_f32::<LittleEndian>()?;
        }
    }

    if vips_data.has_raw_debug_1() {
        let mut raw: [u8; 24] = [0; 24];
        let _ = msg.read(&mut raw);
    }

    if vips_data.has_raw_debug_2() {
        let mut raw: [u8; 24] = [0; 24];
        let _ = msg.read(&mut raw);
    }

    if vips_data.has_accuracy_data() {
        let accuracy = vips_data.accuracy.insert(AccuracyField::default());
        accuracy.position = msg.read_u8()?;
        accuracy.reliability = msg.read_u8()?;
        accuracy.velocity = msg.read_u8()?;
        accuracy.rover_id = msg.read_u8()?;
    }

    if vips_data.has_unit_id_data() {
        let unit_id = vips_data.unit_info.insert(UnitInfoField::default());
        unit_id.radio_id_number = msg.read_u24::<LittleEndian>()?;
        unit_id.rover_id = msg.read_u8()?;
    }
    if vips_data.has_fiz_data() {
        let fiz_data = vips_data.fiz.insert(FizDataField::default());
        fiz_data.focus = msg.read_u32::<LittleEndian>()?;
        fiz_data.iris = msg.read_u16::<LittleEndian>()? as u32;
        fiz_data.zoom = msg.read_u16::<LittleEndian>()? as u32;
    }

    if vips_data.has_origin_data() {
        let origin = vips_data.origin.insert(OriginField::default());
        origin.latitude = msg.read_f64::<LittleEndian>()?;
        origin.longitude = msg.read_f64::<LittleEndian>()?;
        origin.altitude = msg.read_f32::<LittleEndian>()?;
        origin.rotation = msg.read_f32::<LittleEndian>()?;
    }

    if vips_data.has_beacon_used_data() {
        let beacons = vips_data.beacons_used.insert([0u8; 12]);
        msg.read_exact(beacons)?;
    }

    if vips_data.has_vcu_data() {
        let vcu = vips_data.vcu.insert(VcuDataField::default());
        vcu.frame_rate = FrameRate::try_from(msg.read_u8()?).unwrap_or_default();
        vcu.lens_type = LensType::try_from(msg.read_u8()?).unwrap_or_default();
        vcu.status = msg.read_u8()?;
        let _ = msg.read_u8()?; //Currently reserved, add when spec'd
    }

    if vips_data.has_quaternion_data() {
        let quarternions = vips_data.quaternions.insert(QuaternionDataField::default());
        quarternions.x = msg.read_f32::<LittleEndian>()?;
        quarternions.i = msg.read_f32::<LittleEndian>()?;
        quarternions.j = msg.read_f32::<LittleEndian>()?;
        quarternions.k = msg.read_f32::<LittleEndian>()?;
    }

    // Extended FIZ data overwrites the previous FIZ field if present.
    if vips_data.has_fiz_extended_data() {
        let fiz_data = vips_data.fiz.insert(FizDataField::default());
        let focus = msg.read_u32::<LittleEndian>()?;
        let iris = msg.read_u32::<LittleEndian>()?;
        let zoom = msg.read_u32::<LittleEndian>()?;

        const CALIBRATION_BIT: u32 = 1 << 31;

        if focus & CALIBRATION_BIT != 0 {
            fiz_data.focus = (focus & 0x7FFFFFFF) / 100;
            fiz_data.calibrated_focus = true;
        } else {
            fiz_data.calibrated_focus = false;
            fiz_data.focus = focus;
        }

        if iris & CALIBRATION_BIT != 0 {
            fiz_data.iris = (iris & 0x7FFFFFFF) / 100;
            fiz_data.calibrated_iris = true;
        } else {
            fiz_data.calibrated_iris = false;
            fiz_data.iris = iris;
        }

        if zoom & CALIBRATION_BIT != 0 {
            let multiplier = (zoom & 0x7F000000) >> 24;
            fiz_data.zoom = (zoom & 0x00FFFFFF) * multiplier / 100;
            fiz_data.calibrated_zoom = true;
        } else {
            fiz_data.calibrated_zoom = false;
            fiz_data.zoom = zoom & 0x00FFFFFF; // Even uncalibrated, mask out top byte
        }
    }

    Ok(vips_data)
}

#[cfg(test)]
mod tests {
    use super::*;

    // Rust has no built-in 'close' check for floating point numbers, this is a
    // tiny macro to verify floating point values within a tolerance.
    macro_rules! assert_close {
        ($x:expr, $y:expr, $d:expr) => {
            if !($x - $y < $d || $y - $x < $d) {
                panic!();
            }
        };
    }

    // Example test message to validate parsing.
    const EXAMPLE_MSG: [u8; 170] = [
        0x24, 0xD9, 0xAA, 0x00, 0x7E, 0x7E, 0x00, 0x00, 0x19, 0x59, 0x64, 0x02, 0x58, 0x39, 0xB4,
        0xC8, 0x76, 0xBE, 0xF3, 0x3F, 0x83, 0xC0, 0xCA, 0xA1, 0x45, 0xB6, 0x16, 0x40, 0x9A, 0x99,
        0x11, 0x41, 0x0A, 0x20, 0x3D, 0xA0, 0x00, 0x00, 0x34, 0x42, 0x00, 0x00, 0xA0, 0x40, 0xCD,
        0x4C, 0x34, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFA, 0x7A, 0x00, 0x00, 0xFA, 0x32,
        0x00, 0x00, 0x00, 0xC8, 0x00, 0x88, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4A, 0x40,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xBF, 0x00, 0x00, 0x20, 0x43, 0x00, 0x00, 0x00,
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x00, 0x00, 0x02, 0x02,
        0x00, 0x00, 0xE7, 0x42, 0x24, 0xBF, 0x10, 0xCA, 0x4E, 0x3E, 0xD1, 0xC8, 0x1E, 0x3F, 0x2B,
        0x95, 0xCE, 0x3E, 0xEA, 0xC5,
    ];

    #[test]
    fn parse_vips() {
        let result = parse_racelogic_data(&EXAMPLE_MSG).unwrap();

        assert_eq!(result.message_length, 170);
        assert_eq!(result.options_mask, 0x7E7E);
        assert!(!result.has_global_data());

        assert_eq!(result.time_ms, 40130841);
        assert_eq!(result.get_time_string(), "11:08:50.841");
        assert_eq!(result.latitude, 0.0);
        assert_eq!(result.longitude, 0.0);
        assert_eq!(result.altitude, 0.0);
        assert_eq!(result.pos_x, 1.234);
        assert_eq!(result.pos_y, 5.678);
        assert_eq!(result.pos_z, 9.1);

        assert!(result.has_status_data());
        if let Some(status) = &result.status {
            assert_eq!(status.beacon_count, 10);
            assert_eq!(status.solution_type, 32);
            assert_eq!(status.kf_status, 0xA03D);
        }

        assert!(result.has_orientation_data());
        if let Some(orientation) = &result.orientation {
            assert_eq!(orientation.roll, 45.0);
            assert_eq!(orientation.pitch, 5.0);
            assert_eq!(orientation.yaw, 180.3);
        }

        assert!(result.has_velocity_data());
        if let Some(velocity) = &result.velocity {
            assert_eq!(velocity.speed_kmh, 0.0);
            assert_eq!(velocity.heading, 0.0);
            assert_eq!(velocity.velocity_x, 0.0);
            assert_eq!(velocity.velocity_y, 0.0);
        }

        assert!(result.has_vert_velocity_data());
        if let Some(vert_velocity) = result.vert_velocity {
            assert_eq!(vert_velocity, 0.0);
        }

        assert!(result.has_uncertainty_data());
        if let Some(uncertainty) = &result.uncertainty {
            assert_eq!(uncertainty.position.x, 0.0);
            assert_eq!(uncertainty.position.y, 0.0);
            assert_eq!(uncertainty.position.z, 0.0);
            if let Some(ori) = &uncertainty.orientation {
                assert_eq!(ori.roll, 0.0);
                assert_eq!(ori.pitch, 0.0);
                assert_eq!(ori.yaw, 0.0);
            }
            if let Some(vel) = &uncertainty.velocity {
                assert_eq!(vel.x, 0.0);
                assert_eq!(vel.y, 0.0);
                assert_eq!(vel.z, 0.0);
            }
        }

        assert!(!result.has_raw_debug_1());
        assert!(!result.has_raw_debug_2());

        assert!(result.has_accuracy_data());
        if let Some(accuracy) = &result.accuracy {
            assert_eq!(accuracy.position, 0);
            assert_eq!(accuracy.reliability, 0);
            assert_eq!(accuracy.velocity, 0);
            assert_eq!(accuracy.rover_id, 250);
        }

        assert!(result.has_unit_id_data());
        if let Some(unit_info) = &result.unit_info {
            assert_eq!(unit_info.radio_id_number, 122);
            assert_eq!(unit_info.rover_id, 250);
        }

        assert!(result.has_fiz_data());
        if let Some(fiz) = &result.fiz {
            assert_eq!(fiz.focus, 50);
            assert_eq!(fiz.iris, 200);
            assert_eq!(fiz.zoom, 5000);
        }

        if let Some(origin) = &result.origin {
            assert_eq!(origin.latitude, 52.0);
            assert_eq!(origin.longitude, -1.0);
            assert_eq!(origin.altitude, 160.0);
            assert_eq!(origin.rotation, 0.0);
        }

        if let Some(beacons) = &result.beacons_used {
            assert_eq!(beacons, &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 0, 0]);
        }

        if let Some(vcu) = &result.vcu {
            assert_eq!(vcu.frame_rate, FrameRate::FPS_24);
            assert_eq!(vcu.lens_type, LensType::Fuji);
            assert_eq!(vcu.status, 0);
        }

        if let Some(quaternions) = &result.quaternions {
            assert_close!(quaternions.x, -0.6416, 0.001);
            assert_close!(quaternions.i, 10.0, 0.001);
            assert_close!(quaternions.j, 10.0, 0.001);
            assert_close!(quaternions.k, 10.0, 0.001);
        }
    }
}
