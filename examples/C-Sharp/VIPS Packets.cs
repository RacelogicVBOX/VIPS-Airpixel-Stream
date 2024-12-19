//Note this library assumes you are running on a LittleEndian system (which most PCs are)

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RacelogicVIPS {

    class VIPSPacket {

        public enum FIZ_FrameRate { None, f23_976, f24, f25, f29_97, f29_97DF, f30, f48, f50, f59_94, f59_94DF, f60, FreeRun=254, Other };
        public enum LensType { None, Preston, Fuji, Canon, Arri, Zeiss, Other };


        public UInt32 timeMS;

        // only one of the two location formats is valid, check LocationIsLatitudeLongitude to determin which one.
        public double X_Meters;
        public double Y_Meters;
        public double Z_Meters;

        public double Latitude; // degrees
        public double Longitude; // degrees
        public double Altitude; // meters

        public bool LocationIsLatitudeLongitude { get; private set; }

        // all in degrees
        public float Roll;
        public float Pitch;
        public float Yaw;
        public bool EulerValid { get; private set; }

        public float Quart_x;
        public float Quart_i;
        public float Quart_j;
        public float Quart_k;
        public bool QuarternianValid { get; private set; }

        public int Beacons;
        public int SolutionType;
        public int KFStatus;
        public int ID;
        public int Unit_ID;

        public bool SpeedValid { get; private set; }
        public float SpeedKMH;
        public float Heading; // degrees

        public bool VerticalVelocityValid { get; private set; }
        public float SpeedUpMS;

        public int[] UsedBeacons;
        public bool UncertaintiesValid { get; private set; }
        public float XUncert;
        public float YUncert;
        public float ZUncert;
        public float RollUncert;
        public float PitchUncert;
        public float YawUncert;
        public float VxUncert;
        public float VyUncert;
        public float VzUncert;


        public float PosResidual;
        public float VelResidual;
        int ReliabilityFlags;
        public bool OutsideBeacons { get { return ((ReliabilityFlags & 0x01) != 0); } }
        public bool InsufficentBeacons { get { return ((ReliabilityFlags & 0x02) != 0); } }
        public bool DoNotUse { get { return ((ReliabilityFlags & 0x80) != 0); } }

        public int RoverID;


        public bool FIZ_Valid { get; private set; }
        public bool FIZ_CalibratedFocus { get; private set; }
        public bool FIZ_CalibratedIris { get; private set; }
        public bool FIZ_CalibratedZoom { get; private set; }
        public float FocusDistanceMM;
        public float IrisTStops;
        public float FocalLengthMM;

        public int FocusDistanceRAW;
        public int IrisRAW;
        public int FocalLengthRAW;
        public int TelephotoMultiplier;

        public bool OriginKnown { get; private set; }
        public double OriginLatitude;
        public double OriginLongitude;
        public float OriginAltitude;
        public float OriginRotation;

        public FIZ_FrameRate FrameRate;
        public LensType Lens;
        private int VCU_Status;
        public bool VCU_UsingTimeCode { get { return ((VCU_Status & 0x01) != 0); } }
        public bool VCU_UsingGPS { get { return ((VCU_Status & 0x02) != 0); } }
        public bool VCU_UsingPoE { get { return ((VCU_Status & 0x04) != 0); } }
        public bool VCU_UsingBattery { get { return ((VCU_Status & 0x08) != 0); } }
        public bool VCU_Logging { get { return ((VCU_Status & 0x10) != 0); } }
        public bool VCU_SD_AlmostFull { get { return ((VCU_Status & 0x20) != 0); } }


        const UInt32 OUTPUT_GLOBAL_POSITION = 0x01 << 0;
        const UInt32 OUTPUT_STATUS = 0x01 << 1;
        const UInt32 OUTPUT_ORIENTATION = 0x01 << 2;
        const UInt32 OUTPUT_VELOCITY = 0x01 << 3;
        const UInt32 OUTPUT_VERT_VELOCITY = 0x01 << 4;
        const UInt32 OUTPUT_UNCERTAINTY = 0x01 << 5;
        const UInt32 OUTPUT_ACCURACY = 0x01 << 6;
        const UInt32 OUTPUT_RAW_VIPS = 0x01 << 7;
        const UInt32 OUTPUT_RAW_IMU = 0x01 << 8;
        const UInt32 OUTPUT_UNIT_ID = 0x01 << 9;
        const UInt32 OUTPUT_FIZ_DATA = 0x01 << 10;
        const UInt32 OUTPUT_ORIGIN = 0x01 << 11;
        const UInt32 OUTPUT_USED_IDs = 0x01 << 12;
        const UInt32 OUTPUT_VCU_STATUS = 0x01 << 13;
        const UInt32 OUTPUT_QUATERNIONS = 0x01 << 14;
        const UInt32 OUTPUT_FIZ_EXTENDED = 0x01 << 15;
        /***************************************************/

        /// <summary>
        /// Decodes VIPS format message from a byte array buffer
        /// </summary>
        /// <param name="messageBuffer">Incomming serial buffer</param>
        /// <param name="offset">Location in buffer for the start of the packet</param>
        /// <param name="VIPSPacket">The resulting packet</param>
        /// <returns>True if a packet was sucessfully decoded</returns>
        public static bool parseNewFormatMessage(byte[] messageBuffer, int offset, out VIPSPacket VIPSPacket) {
            VIPSPacket = new VIPSPacket();
            bool messageValid = true;
            int readPoint = offset + 2;

            // read message length
            int byteCount = (int)BitConverter.ToUInt16(messageBuffer, readPoint);
            //            BeaconMask = 0;

            // check crc
            uint Polynomial = 4129;
            uint CRC = 0;
            for (int count = 0; count < byteCount - 2; count++) {
                uint Temp = messageBuffer[offset + count];
                CRC = CRC ^ (Temp << 8);
                CRC = CRC % 0x010000;
                for (int i = 0; i < 8; i++) {
                    if ((CRC & 0x8000) == 0x8000) {
                        CRC = CRC << 1;
                        CRC = CRC ^ Polynomial;
                    } else {
                        CRC = CRC << 1;
                    }
                    CRC = CRC % 0x010000;
                }
            }

            if (offset + byteCount < 2)
                return false;

            if (messageBuffer[offset + byteCount - 2] == (byte)((CRC >> 8) & 0x0ff))
                if (messageBuffer[offset + byteCount - 1] == (byte)(CRC & 0xff))
                    messageValid = true;
                else
                    messageValid = false;
            else
                messageValid = false;

            if (messageValid == false)
                return false;

            // move read point past the length
            readPoint += 2;

            //mask values for included data
            UInt32 maskFlags = BitConverter.ToUInt32(messageBuffer, readPoint);
            readPoint += 4;

            VIPSPacket.timeMS = BitConverter.ToUInt32(messageBuffer, readPoint);
            readPoint += 4;

            if ((maskFlags & OUTPUT_GLOBAL_POSITION) == OUTPUT_GLOBAL_POSITION) { // 20 bytes, meaning depends on global flag.
                VIPSPacket.Latitude = BitConverter.ToDouble(messageBuffer, readPoint);
                readPoint += 8;
                VIPSPacket.Longitude = BitConverter.ToDouble(messageBuffer, readPoint);
                readPoint += 8;
                VIPSPacket.Altitude = BitConverter.ToSingle(messageBuffer, readPoint);
                readPoint += 4;
                VIPSPacket.LocationIsLatitudeLongitude = true;
            } else {
                VIPSPacket.X_Meters = BitConverter.ToDouble(messageBuffer, readPoint);
                readPoint += 8;
                VIPSPacket.Y_Meters = BitConverter.ToDouble(messageBuffer, readPoint);
                readPoint += 8;
                VIPSPacket.Z_Meters = BitConverter.ToSingle(messageBuffer, readPoint);
                readPoint += 4;
                VIPSPacket.LocationIsLatitudeLongitude = false;
            }

            // read status block
            if ((maskFlags & OUTPUT_STATUS) == OUTPUT_STATUS) { // 4 bytes
                VIPSPacket.Beacons = messageBuffer[readPoint];
                readPoint++;
                VIPSPacket.SolutionType = messageBuffer[readPoint];
                readPoint++;
                VIPSPacket.KFStatus = BitConverter.ToUInt16(messageBuffer, readPoint);
                readPoint += 2;
            } else {
                VIPSPacket.Beacons = -1;
                VIPSPacket.SolutionType = -1;
                VIPSPacket.KFStatus = -1;
            }

            // read orientation block
            if ((maskFlags & OUTPUT_ORIENTATION) == OUTPUT_ORIENTATION) {  // 12 bytes
                VIPSPacket.Roll = BitConverter.ToSingle(messageBuffer, readPoint);
                readPoint += 4;
                VIPSPacket.Pitch = BitConverter.ToSingle(messageBuffer, readPoint);
                readPoint += 4;
                VIPSPacket.Yaw = BitConverter.ToSingle(messageBuffer, readPoint);
                readPoint += 4;
				EulerValid = true;
            }

            // read velocity
            if ((maskFlags & OUTPUT_VELOCITY) == OUTPUT_VELOCITY) {  // 8 bytes
                VIPSPacket.SpeedValid = true;
                if (VIPSPacket.LocationIsLatitudeLongitude) {
                    VIPSPacket.SpeedKMH = BitConverter.ToSingle(messageBuffer, readPoint); // output is km/h
                    readPoint += 4;
                    VIPSPacket.Heading = BitConverter.ToSingle(messageBuffer, readPoint);
                    readPoint += 4;
                } else { // output was vx/vy in m/s not speed and heading
                    float vx = BitConverter.ToSingle(messageBuffer, readPoint); 
                    readPoint += 4;
                    float vy = BitConverter.ToSingle(messageBuffer, readPoint);
                    readPoint += 4;
                    VIPSPacket.SpeedKMH = (float)(Math.Sqrt(vx * vx + vy * vy) / 3.6); // output is m/s, scale to km/h
                    VIPSPacket.Heading = (float)(Math.Atan2(vy, vx) * 180 / Math.PI);
                }
            } else {
                VIPSPacket.SpeedValid = false;
            }

            if ((maskFlags & OUTPUT_VERT_VELOCITY) == OUTPUT_VERT_VELOCITY) {  // 4 bytes
                VIPSPacket.SpeedUpMS = BitConverter.ToSingle(messageBuffer, readPoint);
                readPoint += 4;
                VIPSPacket.VerticalVelocityValid = true;
            } else {
                VIPSPacket.VerticalVelocityValid = false;
            }

            if ((maskFlags & OUTPUT_UNCERTAINTY) == OUTPUT_UNCERTAINTY) {  // 12, 24 or 36 bytes depending on outputs
                VIPSPacket.UncertaintiesValid = true;
                VIPSPacket.XUncert = BitConverter.ToSingle(messageBuffer, readPoint);
                readPoint += 4;
                VIPSPacket.YUncert = BitConverter.ToSingle(messageBuffer, readPoint);
                readPoint += 4;
                VIPSPacket.ZUncert = BitConverter.ToSingle(messageBuffer, readPoint);
                readPoint += 4;
                if ((maskFlags & OUTPUT_ORIENTATION) == OUTPUT_ORIENTATION) {
                    VIPSPacket.RollUncert = BitConverter.ToSingle(messageBuffer, readPoint);
                    readPoint += 4;
                    VIPSPacket.PitchUncert = BitConverter.ToSingle(messageBuffer, readPoint);
                    readPoint += 4;
                    VIPSPacket.YawUncert = BitConverter.ToSingle(messageBuffer, readPoint);
                    readPoint += 4;
                }
                if ((maskFlags & OUTPUT_VELOCITY) == OUTPUT_VELOCITY) {
                    VIPSPacket.VxUncert = BitConverter.ToSingle(messageBuffer, readPoint);
                    readPoint += 4;
                    VIPSPacket.VyUncert = BitConverter.ToSingle(messageBuffer, readPoint);
                    readPoint += 4;
                    VIPSPacket.VzUncert = BitConverter.ToSingle(messageBuffer, readPoint);
                    readPoint += 4;
                }
            } else {
                VIPSPacket.UncertaintiesValid = false;
            }

            if ((maskFlags & OUTPUT_ACCURACY) == OUTPUT_ACCURACY) { // 4 bytes
                VIPSPacket.PosResidual = messageBuffer[readPoint] / 20.0f;
                readPoint++;
                VIPSPacket.ReliabilityFlags = messageBuffer[readPoint];
                readPoint++;
                VIPSPacket.VelResidual = messageBuffer[readPoint] / 10.0f;
                readPoint++;
                VIPSPacket.RoverID = messageBuffer[readPoint];
                readPoint++;
            } else {
                VIPSPacket.PosResidual = -1;
                VIPSPacket.ReliabilityFlags = 0;
                VIPSPacket.VelResidual = -1;
                VIPSPacket.RoverID = 0;
            }

            if ((maskFlags & OUTPUT_RAW_VIPS) == OUTPUT_RAW_VIPS) {  // 24 bytes
                // Contains non-KF VIPS data and more resolution for quality metrics. Details avalible on request, 
                readPoint += 24;
            }

            if ((maskFlags & OUTPUT_RAW_IMU) == OUTPUT_RAW_IMU) { // 24 bytes
                readPoint += 24;
            }
            if ((maskFlags & OUTPUT_UNIT_ID) == OUTPUT_UNIT_ID) { // 4 bytes
                readPoint += 3;
                VIPSPacket.RoverID = messageBuffer[readPoint++];
            }


            if ((maskFlags & OUTPUT_FIZ_DATA) == OUTPUT_FIZ_DATA) { // 8 bytes
                VIPSPacket.FIZ_Valid = true;
                if (VIPSPacket.FIZ_CalibratedFocus) {
                    VIPSPacket.FocusDistanceMM = (float)BitConverter.ToUInt32(messageBuffer, readPoint);
                } else {
                    VIPSPacket.FocusDistanceRAW = BitConverter.ToUInt32(messageBuffer, readPoint);
                }
                readPoint += 4;
                if (VIPSPacket.FIZ_CalibratedIris) {
                    VIPSPacket.IrisTStops = (float)(BitConverter.ToUInt16(messageBuffer, readPoint) / 100.0f);
                } else {
                    VIPSPacket.IrisRAW = BitConverter.ToUInt16(messageBuffer, readPoint);
                }
                readPoint += 2;
                if (VIPSPacket.FIZ_CalibratedZoom) {
                    VIPSPacket.FocalLengthMM = (float)BitConverter.ToUInt16(messageBuffer, readPoint) / 100.0f;
                } else {
                    VIPSPacket.FocalLengthRAW = BitConverter.ToUInt16(messageBuffer, readPoint);
                }
                readPoint += 2;
            }

            if ((maskFlags & OUTPUT_ORIGIN) == OUTPUT_ORIGIN) { // 24 bytes
                VIPSPacket.OriginKnown = true;
                VIPSPacket.OriginLatitude = BitConverter.ToDouble(messageBuffer, readPoint);
                readPoint += 8;
                VIPSPacket.OriginLongitude = BitConverter.ToDouble(messageBuffer, readPoint);
                readPoint += 8;
                VIPSPacket.OriginAltitude = BitConverter.ToSingle(messageBuffer, readPoint);
                readPoint += 4;
                VIPSPacket.OriginRotation = BitConverter.ToSingle(messageBuffer, readPoint);
                readPoint += 4;
            } else
                VIPSPacket.OriginKnown = false;
             
            if ((maskFlags & OUTPUT_USED_IDs) == OUTPUT_USED_IDs) { // 12 bytes
                VIPSPacket.UsedBeacons = new int[12];
                Array.Copy(messageBuffer, readPoint, VIPSPacket.UsedBeacons, 0, 12);
                readPoint += 12;
            }

            if ((maskFlags & OUTPUT_VCU_STATUS) == OUTPUT_VCU_STATUS) {  // 4 bytes
                switch (messageBuffer[readPoint++]) {
                    case 0: VIPSPacket.FrameRate = FIZ_FrameRate.None; break;
                    case 1: VIPSPacket.FrameRate = FIZ_FrameRate.f23_976; break;
                    case 2: VIPSPacket.FrameRate = FIZ_FrameRate.f24; break;
                    case 3: VIPSPacket.FrameRate = FIZ_FrameRate.f25; break;
                    case 4: VIPSPacket.FrameRate = FIZ_FrameRate.f29_97; break;
                    case 5: VIPSPacket.FrameRate = FIZ_FrameRate.f29_97DF; break;
                    case 6: VIPSPacket.FrameRate = FIZ_FrameRate.f30; break;
                    case 7: VIPSPacket.FrameRate = FIZ_FrameRate.f48; break;
                    case 8: VIPSPacket.FrameRate = FIZ_FrameRate.f50; break;
                    case 9: VIPSPacket.FrameRate = FIZ_FrameRate.f59_94; break;
                    case 10: VIPSPacket.FrameRate = FIZ_FrameRate.f59_94DF; break;
                    case 11: VIPSPacket.FrameRate = FIZ_FrameRate.f60; break;
                    case 254: VIPSPacket.FrameRate = FIZ_FrameRate.FreeRun; break;
                    default: VIPSPacket.FrameRate = FIZ_FrameRate.Other; break;
                }
                switch (messageBuffer[readPoint++]) {
                    case 0: VIPSPacket.Lens = LensType.None; break;
                    case 1: VIPSPacket.Lens = LensType.Preston; break;
                    case 2: VIPSPacket.Lens = LensType.Fuji; break;
                    case 3: VIPSPacket.Lens = LensType.Canon; break;
                    case 4: VIPSPacket.Lens = LensType.Arri; break;
                    case 5: VIPSPacket.Lens = LensType.Zeiss; break;
                    default: VIPSPacket.Lens = LensType.Other; break;
                }

                VIPSPacket.VCU_Status = messageBuffer[readPoint++];
                readPoint++;
            } else {
                VIPSPacket.FrameRate = FIZ_FrameRate.None;
                VIPSPacket.Lens = LensType.None;
                VIPSPacket.VCU_Status = 0;
            }

            if ((maskFlags & OUTPUT_QUATERNIONS) == OUTPUT_QUATERNIONS) { // 16 bytes

                VIPSPacket.QuarternianValid = true;
                VIPSPacket.Quart_x = BitConverter.ToSingle(messageBuffer, readPoint);
                VIPSPacket.Quart_i = BitConverter.ToSingle(messageBuffer, readPoint);
                VIPSPacket.Quart_j = BitConverter.ToSingle(messageBuffer, readPoint);
                VIPSPacket.Quart_k = BitConverter.ToSingle(messageBuffer, readPoint);

            } else
                VIPSPacket.QuarternianValid = false;

            if ((maskFlags & OUTPUT_FIZ_EXTENDED) == OUTPUT_FIZ_EXTENDED) { // 12 bytes
                // Favour this field over the older FIZ field
                VIPSPacket.FIZ_Valid = true;

                var FocusDistance = BitConverter.ToUInt32(messageBuffer, readPoint);
                readPoint += 4;
                var Iris = BitConverter.ToUInt32(messageBuffer, readPoint);
                readPoint += 4;
                var Zoom = BitConverter.ToUInt32(messageBuffer, readPoint);
                readPoint += 4;

                VIPSPacket.FIZ_CalibratedFocus = ((FocusDistance & (1 << 31)) == 1 << 31);
                VIPSPacket.FIZ_CalibratedIris = ((Iris & (1 << 31)) == 1 << 31);
                VIPSPacket.FIZ_CalibratedZoom = ((Zoom & (1 << 31)) == 1 << 31);
                VIPSPacket.TelephotoMultiplier = (Zoom >> 24) & 0x7f;

                if (VIPSPacket.FIZ_CalibratedFocus) {
                    VIPSPacket.FocusDistanceMM = (FocusDistance & ~(1<<31))/100.0f;
                } else {
                    VIPSPacket.FocusDistanceRAW = FocusDistance;
                }

                if (VIPSPacket.FIZ_CalibratedIris) {
                    VIPSPacket.IrisTStops = (Iris & ~(1 << 31)) / 100.0f;
                } else {
                    VIPSPacket.IrisRAW = Iris;
                }
                if (VIPSPacket.FIZ_CalibratedZoom) {
                    VIPSPacket.FocusLengthMM = (Zoom & 0x00ffffff)/100.0f;
                } else {
                    VIPSPacket.FocusLengthRAW = (Zoom & 0x00ffffff)
                }
            }

            return messageValid;
        }
    }
}
