// MFRC522 documentation link: https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf

using System;
using System.Device.Gpio;
using System.Device.Spi;
using System.Linq;

namespace RFID522
{
    public class MFRC522
    {
        private const byte CommandReg = 0x01; // Starts and stops command execution.
        private const byte ComIEnReg = 0x02; // Control bits to enable and disable the passing of interrupt requests.
        private const byte ComIrqReg = 0x04; // Interrupt request bits.
        private const byte FIFODataReg = 0x09; // Input and output of 64 byte FIFO buffer.
        private const byte FIFOLevelReg = 0x0A; // Indicates the number of bytes stored in the FIFO.
        private const byte BitFramingReg = 0x0D; // Adjustments for bit-oriented frames.
        private const byte ModeReg = 0x11; // Defines general mode settings for transmitting and receiving.
        private const byte TxControlReg = 0x14; // Controls the logical behavior of the antenna driver pins TX1 and TX2.
        private const byte TxASKReg = 0x15; // Controls transmit modulation settings.
        private const byte TModeReg = 0x2A; // Defines the timer settings.
        private const byte TPrescalerReg = 0x2B; // Defines the timer settings.
        private const byte TReloadRegH = 0x2C; // Defines the 16-bit timer reload value (high 8 bit).
        private const byte TReloadRegL = 0x2D; // Defines the 16-bit timer reload value (low 8 bit).
        private const byte AutoTestReg = 0x36; // Controls the digital self-test.
        private const byte VersionReg = 0x37; // Shows the MFRC522 software version.

        private const byte Command_Idle = 0x00;
        private const byte Command_Mem = 0x01;
        private const byte Command_CalcCRC = 0x03;
        private const byte Command_Authenticate = 0x0E;
        private const byte Command_Transceive = 0x0C;
        private const byte Command_SoftReset = 0x0F;

        private const byte IRQControlBit_TimerIEn = 0x01;
        private const byte IRQControlBit_ErrIEn = 0x02;
        private const byte IRQControlBit_LoAlertIEn = 0x04;
        private const byte IRQControlBit_HiAlertIEn = 0x08;
        private const byte IRQControlBit_IdleIEn = 0x10;
        private const byte IRQControlBit_RxIEn = 0x20;
        private const byte IRQControlBit_TxIEn = 0x40;
        private const byte IRQControlBit_IRqInv = 0x80;

        private const byte AutoTestReg_DefaultOperationMode = 0x00;
        private const byte ComIrqReg_ControlBit_Set1 = 0x80;
        private const byte FIFOLevelReg_FlushFifoBuffer = 0x80;
        private const byte AutoTestReg_EnableSelfTest = 0x09;
        private const byte TxASKReg_Force100ASK = 0x40;
        private const byte BitFramingReg_StartSend = 0x80;

        private readonly GpioController _controller;
        private readonly SpiDevice _spiDevice;
        private byte _connectedResetPinForRfidBoard = 22;

        public MFRC522(SpiDevice spiDevice, byte resetPin = 22)
        {
            _spiDevice = spiDevice;
            _connectedResetPinForRfidBoard = resetPin;
            _controller = new GpioController();
        }

        public void Init()
        {
            _controller.OpenPin(_connectedResetPinForRfidBoard, PinMode.Output);
            _controller.Write(_connectedResetPinForRfidBoard, PinValue.High);

            this.WriteRegister(MFRC522.CommandReg, MFRC522.Command_SoftReset);

            this.WriteRegister(MFRC522.TPrescalerReg, 0x3E); //00111110b -> set as PreScaler lower 8 bits

            var checkValue = this.ReadRegister(MFRC522.TPrescalerReg);

            // The following 4 lines set the timer scale and the timer timeout
            this.WriteRegister(MFRC522.TModeReg, 0x8D); //10001101b -> set 1101 as PreScaler higher 4 bits and set TAuto bit to 1
            this.WriteRegister(MFRC522.TPrescalerReg, 0x3E); //00111110b -> set as PreScaler lower 8 bits
            this.WriteRegister(MFRC522.TReloadRegL, 30);
            this.WriteRegister(MFRC522.TReloadRegH, 0);

            //force using ASK (amplitude shift keying) modulation
            this.WriteRegister(MFRC522.TxASKReg, MFRC522.TxASKReg_Force100ASK);
            this.WriteRegister(MFRC522.ModeReg, 0x3D); //00111101b -> CRC preset value to 6363h

            this.AntennaOn();
        }

        // Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
        public(byte status, int backBits) Request(byte requestMode)
        {
            byte[] tagType = new byte[1]
            {
                requestMode
            };

            this.WriteRegister(MFRC522.BitFramingReg, 0x07); // number of bits in the last byte to be transmitted - for requests, the frame size is 7 bits.

            var(status, backData, backBits) = SendCommand(Command_Transceive, tagType);

            if ((status != 0) | (backBits != 0x10))
            {
                status = 2;
            }

            return (status, backBits);
        }

        public void SelfTest()
        {
            // Perform a soft reset
            this.WriteRegister(MFRC522.CommandReg, MFRC522.Command_SoftReset);

            // Flush FIFO buffer
            this.SetBitMaskForRegister(MFRC522.FIFOLevelReg, MFRC522.FIFOLevelReg_FlushFifoBuffer);
            // Clear the internal buffer by writing 25 bytes of 00h
            for (int i = 0; i < 25; i++)
            {
                this.WriteRegister(MFRC522.FIFODataReg, 0x00);
            }
            // Store the FIFO buffer content into the internal buffer
            this.WriteRegister(MFRC522.CommandReg, MFRC522.Command_Mem);

            // Enable the self test
            this.WriteRegister(MFRC522.AutoTestReg, MFRC522.AutoTestReg_EnableSelfTest);

            // Write 00h to the FIFO buffer
            this.WriteRegister(MFRC522.FIFODataReg, 0x00);

            // Start the self test with the CalcCRC command
            this.WriteRegister(MFRC522.CommandReg, MFRC522.Command_CalcCRC);

            // Wait for the FIFO buffer to contain 64 bytes of data
            byte numberOfBytesStoredInFIFO = 0;
            for (int i = 0; i < 255; i++)
            {
                numberOfBytesStoredInFIFO = this.ReadRegister(MFRC522.FIFOLevelReg);
                if (numberOfBytesStoredInFIFO >= 64)
                {
                    break;
                }
            }
            this.WriteRegister(MFRC522.CommandReg, MFRC522.Command_Idle);

            byte[] selfTestResult = new byte[64];
            for (int i = 0; i < 64; i++)
            {
                selfTestResult[i] = this.ReadRegister(MFRC522.FIFODataReg);
            }

            this.WriteRegister(MFRC522.AutoTestReg, MFRC522.AutoTestReg_DefaultOperationMode);

            byte version = this.ReadRegister(MFRC522.VersionReg);
            this.WriteVersionAndSelfTestResult(version, selfTestResult);
        }

        private(byte status, byte[] backData, byte backLen) SendCommand(byte command, byte[] sendData)
        {
            byte[] backData = new byte[64];
            byte backLen = 0;
            byte status = 2;
            byte enabledIrqRequestTypes = 0x00;
            byte waitForIrqTypes = 0x00;
            byte n = 0;
            var i = 0;

            switch (command)
            {
                case MFRC522.Command_Authenticate:
                    enabledIrqRequestTypes = MFRC522.IRQControlBit_ErrIEn | MFRC522.IRQControlBit_IdleIEn;
                    waitForIrqTypes = MFRC522.IRQControlBit_IdleIEn;
                    break;

                case MFRC522.Command_Transceive:
                    enabledIrqRequestTypes = MFRC522.IRQControlBit_TimerIEn | MFRC522.IRQControlBit_ErrIEn | MFRC522.IRQControlBit_LoAlertIEn |
                        MFRC522.IRQControlBit_IdleIEn | MFRC522.IRQControlBit_RxIEn | MFRC522.IRQControlBit_TxIEn;
                    waitForIrqTypes = MFRC522.IRQControlBit_IdleIEn | MFRC522.IRQControlBit_RxIEn;
                    break;
            }
            // enable required IRQ types
            this.WriteRegister(MFRC522.ComIEnReg, (byte) (enabledIrqRequestTypes | MFRC522.IRQControlBit_IRqInv));
            // Indicate that the marked bits in the ComIrqReg register are cleared (=IMO clear the IRQ bits)
            this.ClearBitMaskForRegister(MFRC522.ComIrqReg, ComIrqReg_ControlBit_Set1);
            // Flush FIFO buffer
            this.SetBitMaskForRegister(MFRC522.FIFOLevelReg, MFRC522.FIFOLevelReg_FlushFifoBuffer);
            // cancel all ongoing commands
            this.WriteRegister(MFRC522.CommandReg, MFRC522.Command_Idle);
            // push command data into FIFO buffer
            int dataBytePointer = 0;
            while (dataBytePointer < sendData.Length)
            {
                this.WriteRegister(MFRC522.FIFODataReg, sendData[dataBytePointer]);
                dataBytePointer++;
            }
            // execute the command
            this.WriteRegister(MFRC522.CommandReg, command);
            // start the transceive command
            if (command == MFRC522.Command_Transceive)
            {
                this.SetBitMaskForRegister(MFRC522.BitFramingReg, MFRC522.BitFramingReg_StartSend);
            }

            i = 2000;
            do
            {
                n = this.ReadRegister(MFRC522.ComIrqReg);
            }
            while (--i != 0 && (n & 0x01) == 0 && (n & waitForIrqTypes) == 0);

            this.ClearBitMaskForRegister(MFRC522.BitFramingReg, MFRC522.BitFramingReg_StartSend);

            if (i == 0) return (status, backData.ToArray(), backLen);

            // if ((ReadSpi(ErrorReg) & 0x1B) == WriteMask)
            // {
            //     status = OK;

            //     if (Convert.ToBoolean(n & enabledIrqRequestTypes & 0x01))
            //     {
            //         status = NoTag;
            //     }

            //     if (command == Transceive)
            //     {
            //         n = ReadSpi(FIFOLevelReg);
            //         byte? lastBits = (byte)(ReadSpi(ControlReg) & 0x07);
            //         if (lastBits != 0)
            //         {
            //             backLen = (byte)(((n - 1) * 8) + (byte)lastBits);
            //         }
            //         else
            //         {
            //             backLen = (byte)(n * 8);
            //         }

            //         if (n == 0)
            //         {
            //             n = 1;
            //         }

            //         if (n > MAX_LEN)
            //         {
            //             n = MAX_LEN;
            //         }

            //         i = 0;
            //         while (i < n)
            //         {
            //             backData.Add(ReadSpi(FIFODataReg));
            //             i++;
            //         }
            //     }
            // }
            // else
            // {
            //     status = Error;
            // }

            return (status, backData, backLen);
        }

        private void WriteVersionAndSelfTestResult(byte rawVersion, byte[] selfTestResult)
        {
            byte[] expectedResultVersion1 = new byte[64]
            {
                0x00,
                0xC6,
                0x37,
                0xD5,
                0x32,
                0xB7,
                0x57,
                0x5C,
                0xC2,
                0xD8,
                0x7C,
                0x4D,
                0xD9,
                0x70,
                0xC7,
                0x73,
                0x10,
                0xE6,
                0xD2,
                0xAA,
                0x5E,
                0xA1,
                0x3E,
                0x5A,
                0x14,
                0xAF,
                0x30,
                0x61,
                0xC9,
                0x70,
                0xDB,
                0x2E,
                0x64,
                0x22,
                0x72,
                0xB5,
                0xBD,
                0x65,
                0xF4,
                0xEC,
                0x22,
                0xBC,
                0xD3,
                0x72,
                0x35,
                0xCD,
                0xAA,
                0x41,
                0x1F,
                0xA7,
                0xF3,
                0x53,
                0x14,
                0xDE,
                0x7E,
                0x02,
                0xD9,
                0x0F,
                0xB5,
                0x5E,
                0x25,
                0x1D,
                0x29,
                0x79
            };

            byte[] expectedResultVersion2 = new byte[64]
            {
                0x00,
                0xEB,
                0x66,
                0xBA,
                0x57,
                0xBF,
                0x23,
                0x95,
                0xD0,
                0xE3,
                0x0D,
                0x3D,
                0x27,
                0x89,
                0x5C,
                0xDE,
                0x9D,
                0x3B,
                0xA7,
                0x00,
                0x21,
                0x5B,
                0x89,
                0x82,
                0x51,
                0x3A,
                0xEB,
                0x02,
                0x0C,
                0xA5,
                0x00,
                0x49,
                0x7C,
                0x84,
                0x4D,
                0xB3,
                0xCC,
                0xD2,
                0x1B,
                0x81,
                0x5D,
                0x48,
                0x76,
                0xD5,
                0x71,
                0x61,
                0x21,
                0xA9,
                0x86,
                0x96,
                0x83,
                0x38,
                0xCF,
                0x9D,
                0x5B,
                0x6D,
                0xDC,
                0x15,
                0xBA,
                0x3E,
                0x7D,
                0x95,
                0x3B,
                0x2F
            };

            byte VersionBitMask = 0x0F;
            var version = rawVersion & VersionBitMask;
            System.Console.WriteLine($"Software version: {version}.0");
            bool testPassed = version == 1 ?
                selfTestResult.SequenceEqual(expectedResultVersion1) :
                selfTestResult.SequenceEqual(expectedResultVersion2);
            System.Console.WriteLine($"Self test passed: {testPassed}");
        }

        private void AntennaOn()
        {
            const byte Tx1RFEn_bit = 1;
            const byte Tx2RFEn_bit = 2;
            const byte Tx12RFEnabled_bits = Tx1RFEn_bit & Tx2RFEn_bit;

            byte temp = this.ReadRegister(MFRC522.TxControlReg);
            if (((byte) (temp & Tx12RFEnabled_bits)) == 0)
            {
                this.SetBitMaskForRegister(MFRC522.TxControlReg, Tx12RFEnabled_bits);
            }
        }

        private void SetBitMaskForRegister(byte address, byte bitmask)
        {
            byte temp = this.ReadRegister(address);
            this.WriteRegister(address, (byte) (temp | bitmask));
        }

        private void ClearBitMaskForRegister(byte address, byte bitmask)
        {
            byte temp = this.ReadRegister(address);
            this.WriteRegister(address, (byte) (temp & (~bitmask)));
        }

        private void WriteRegister(byte address, byte value)
        {
            Span<byte> buffer = stackalloc byte[2]
            {
                GetAddressByte(address),
                value
            };

            _spiDevice.Write(buffer);
        }

        private byte ReadRegister(byte address)
        {
            Span<byte> buffer = stackalloc byte[2]
            {
                GetAddressByte(address, true),
                0
            };
            _spiDevice.TransferFullDuplex(buffer, buffer);
            return buffer[1];
        }

        // Calculate address byte according to MFRC522 documentation, section 8.1.2.3
        private byte GetAddressByte(byte address, bool forRead = false)
        {
            // Set MSB of the address byte to 1 for read, 0 to write 
            const byte ReadMask = 0x80;
            const byte WriteMask = 0x00;

            return forRead ?
                (byte) (((address << 1) & 0x7E) | ReadMask) :
                (byte) (((address << 1) & 0x7E) | WriteMask);
        }
    }
}