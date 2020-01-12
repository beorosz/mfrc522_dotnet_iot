// MFRC522 documentation link: https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf

using System;
using System.Collections.Generic;
using System.Device.Gpio;
using System.Device.Spi;
using System.Linq;
using System.Text;

namespace RFID522
{
    public class MFRC522
    {
        private const byte CommandReg = 0x01; // Starts and stops command execution.
        private const byte ComIEnReg = 0x02; // Control bits to enable and disable the passing of interrupt requests.
        private const byte ComIrqReg = 0x04; // Interrupt request bits.
        private const byte ErrorReg = 0x06; // Error bit register showing the error status of the last command executed.
        private const byte FIFODataReg = 0x09; // Input and output of 64 byte FIFO buffer.
        private const byte FIFOLevelReg = 0x0A; // Indicates the number of bytes stored in the FIFO.
        private const byte WaterLevelReg = 0x0B; // Defines the level for FIFO under- and overflow warning.
        private const byte ControlReg = 0x0C; // Miscellaneous control bits.
        private const byte BitFramingReg = 0x0D; // Adjustments for bit-oriented frames.
        private const byte ModeReg = 0x11; // Defines general mode settings for transmitting and receiving.
        private const byte TxModeReg = 0x12; // Defines the data rate during transmission.
        private const byte RxModeReg = 0x13; // Defines the data rate during reception.
        private const byte TxControlReg = 0x14; // Controls the logical behavior of the antenna driver pins TX1 and TX2.
        private const byte TxASKReg = 0x15; // Controls transmit modulation settings.
        private const byte ModWidthReg = 0x24; // Sets the modulation width.
        private const byte TModeReg = 0x2A; // Defines the timer settings.
        private const byte TPrescalerReg = 0x2B; // Defines the timer settings.
        private const byte TReloadRegH = 0x2C; // Defines the 16-bit timer reload value (high 8 bit).
        private const byte TReloadRegL = 0x2D; // Defines the 16-bit timer reload value (low 8 bit).
        private const byte TCounterValRegH = 0x2E; // The timer value bits are contained in two 8-bit registers.
        private const byte TCounterValRegL = 0x2F; // The timer value bits are contained in two 8-bit registers.

        private const byte AutoTestReg = 0x36; // Controls the digital self-test.
        private const byte VersionReg = 0x37; // Shows the MFRC522 software version.

        private const byte Command_Idle = 0x00;
        private const byte Command_Mem = 0x01;
        private const byte Command_CalcCRC = 0x03;
        private const byte Command_Transceive = 0x0C;
        private const byte Command_Authenticate = 0x0E;
        private const byte Command_SoftReset = 0x0F;

        private const byte IRQControlBit_TimerIEn = 0x01;
        private const byte IRQControlBit_ErrIEn = 0x02;
        private const byte IRQControlBit_LoAlertIEn = 0x04;
        private const byte IRQControlBit_HiAlertIEn = 0x08;
        private const byte IRQControlBit_IdleIEn = 0x10;
        private const byte IRQControlBit_RxIEn = 0x20;
        private const byte IRQControlBit_TxIEn = 0x40;
        private const byte IRQControlBit_IRqInv = 0x80;

        private const byte TxControlReg_Tx1RFEn_bit = 0x01;
        private const byte TxControlReg_Tx2RFEn_bit = 0x02;

        private const byte ErrorReg_ProtocolErr_bit = 0x01;
        private const byte ErrorReg_ParityErr_bit = 0x02;
        private const byte ErrorReg_CRCErr_bit = 0x04;
        private const byte ErrorReg_CollErr_bit = 0x08;
        private const byte ErrorReg_BufferOvfl_bit = 0x10;
        private const byte ErrorReg_Reserved_bit = 0x20;
        private const byte ErrorReg_TempErr_bit = 0x40;
        private const byte ErrorReg_WrErr_bit = 0x80;

        private const byte AutoTestReg_DefaultOperationMode = 0x00;
        private const byte ComIrqReg_ControlBit_Set1 = 0x80;
        private const byte FIFOLevelReg_FlushFifoBuffer = 0x80;
        private const byte AutoTestReg_EnableSelfTest = 0x09;
        private const byte TxASKReg_Force100ASK = 0x40;
        private const byte BitFramingReg_StartSend = 0x80;
        private const byte ControlReg_RxLastBits = 0x07;

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

            // The following 4 lines set the timer scale and the timer timeout
            // PreScaler=0x0D3E -> timer tick interval downscaled to: 13.56 MHz / (2*PreScaler+1) = 2000 Hz = 0.0005 s = 0.5 ms
            // reload value = 30 - timeout happens within 30 * 0.5 ms = 15 ms.
            this.WriteRegister(MFRC522.TModeReg, 0x8D); // PreScaler high 4 bits(0x0D) + set TAuto (0x80) bit to 1
            this.WriteRegister(MFRC522.TPrescalerReg, 0x3E); // PreScaler lower 8 bits

            this.WriteRegister(MFRC522.TReloadRegL, 0x1D);
            this.WriteRegister(MFRC522.TReloadRegH, 0x0);

            //force using ASK (amplitude shift keying) modulation
            this.WriteRegister(MFRC522.TxASKReg, MFRC522.TxASKReg_Force100ASK);
            this.WriteRegister(MFRC522.ModeReg, 0x3D); // CRC preset value to 6363h
        }

        // Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
        public Status Request(byte requestMode)
        {
            this.AntennaOn();

            byte[] tagType = new byte[1]
            {
                requestMode
            };

            // Number of bits in the last byte to be transmitted - for requests, the frame size is 7 bits.
            this.WriteRegister(MFRC522.BitFramingReg, 0x07);

            var(status, backData, backBits) = SendCommand(Command_Transceive, tagType);

            this.AntennaOff();

            return status;
        }

        public (Status status, byte[] serialNumber) AntiCollision(byte collisionCommand)
        {
            var serialNumber = new List<byte>();

            this.AntennaOn();

            // Number of bits in the last byte to be transmitted - for requests, the frame size is 8 bits (= 000b must be set to framing reg 0..2 bits).
            this.WriteRegister(MFRC522.BitFramingReg, 0x00);

            byte[] serialNumberReqData = new byte[2]
            {
                collisionCommand,
                0x20
            };

            var(status, backData, backBits) = SendCommand(Command_Transceive, serialNumberReqData);

            this.AntennaOff();

            return (status, backData);
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

        private(Status status, byte[] backData, int backLen) SendCommand(byte command, byte[] sendData)
        {
            var backData = new List<byte>();
            int backLen = 0;
            Status status = Status.NoTag;
            byte enabledIrqRequestTypes = 0x00;
            byte waitForIrqTypes = 0x00;

            switch (command)
            {
                case MFRC522.Command_Authenticate:
                    enabledIrqRequestTypes = MFRC522.IRQControlBit_ErrIEn | MFRC522.IRQControlBit_IdleIEn;
                    waitForIrqTypes = MFRC522.IRQControlBit_IdleIEn;
                    break;

                case MFRC522.Command_Transceive:
                    enabledIrqRequestTypes = MFRC522.IRQControlBit_TimerIEn | MFRC522.IRQControlBit_ErrIEn | MFRC522.IRQControlBit_LoAlertIEn |
                        MFRC522.IRQControlBit_IdleIEn | MFRC522.IRQControlBit_RxIEn | MFRC522.IRQControlBit_TxIEn;
                    waitForIrqTypes = MFRC522.IRQControlBit_RxIEn;
                    break;
            }
            // enable required IRQ types
            this.WriteRegister(MFRC522.ComIEnReg, (byte) (enabledIrqRequestTypes | IRQControlBit_IRqInv));

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
                this.WriteRegister(MFRC522.FIFODataReg, sendData[dataBytePointer++]);
            }

            // execute the command
            this.WriteRegister(MFRC522.CommandReg, command);
            // start the transceive command
            if (command == MFRC522.Command_Transceive)
            {
                this.SetBitMaskForRegister(MFRC522.BitFramingReg, MFRC522.BitFramingReg_StartSend);
            }

            byte currentIRQbits;
            int numberOfCycles = 2000;
            do
            {
                currentIRQbits = this.ReadRegister(MFRC522.ComIrqReg);
                numberOfCycles--;
            }
            // run while number of cycles was not reached and timer timeout did not happen and expected interrupts were not fired 
            while (
                numberOfCycles != 0 &&
                (currentIRQbits & MFRC522.IRQControlBit_TimerIEn) == 0 &&
                (currentIRQbits & waitForIrqTypes) == 0);

            // stop transceive command
            this.ClearBitMaskForRegister(MFRC522.BitFramingReg, MFRC522.BitFramingReg_StartSend);

            // if timer timeout interrupt was fired, no tag was found
            if ((currentIRQbits & MFRC522.IRQControlBit_TimerIEn) == MFRC522.IRQControlBit_TimerIEn)
            {
                status = Status.NoTag;
            }
            else if ((currentIRQbits & waitForIrqTypes) > 0) // one or more of the expected IRQs were fired
            {
                byte errorByte = this.ReadRegister(MFRC522.ErrorReg);
                byte fatalErrorsMask = MFRC522.ErrorReg_ProtocolErr_bit | MFRC522.ErrorReg_ParityErr_bit |
                    MFRC522.ErrorReg_CollErr_bit | MFRC522.ErrorReg_BufferOvfl_bit;

                if ((errorByte & fatalErrorsMask) == 0) // no fatal error occured
                {
                    status = Status.OK;

                    if (command == MFRC522.Command_Transceive)
                    {
                        byte numberOfBytesStoredInFIFO = this.ReadRegister(MFRC522.FIFOLevelReg);

                        int lastBits = this.ReadRegister(MFRC522.ControlReg) & MFRC522.ControlReg_RxLastBits;
                        backLen = lastBits != 0 ?
                            (numberOfBytesStoredInFIFO - 1) * 8 + lastBits :
                            numberOfBytesStoredInFIFO * 8;

                        if (numberOfBytesStoredInFIFO == 0)
                            numberOfBytesStoredInFIFO = 1;
                        if (numberOfBytesStoredInFIFO > 16)
                            numberOfBytesStoredInFIFO = 16;

                        int i = 0;
                        while (i++ < numberOfBytesStoredInFIFO)
                        {
                            backData.Add(this.ReadRegister(MFRC522.FIFODataReg));
                        }
                    }
                }
                else
                {
                    status = Status.Error;
                }
            }

            return (status, backData.ToArray(), backLen);
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
            const byte Tx12RFEnabled_bits = MFRC522.TxControlReg_Tx1RFEn_bit | MFRC522.TxControlReg_Tx2RFEn_bit;

            byte temp = this.ReadRegister(MFRC522.TxControlReg);
            if ((temp & Tx12RFEnabled_bits) != Tx12RFEnabled_bits)
            {
                this.SetBitMaskForRegister(MFRC522.TxControlReg, Tx12RFEnabled_bits);
            }
        }

        private void AntennaOff()
        {
            const byte Tx12RFEnabled_bits = MFRC522.TxControlReg_Tx1RFEn_bit | MFRC522.TxControlReg_Tx2RFEn_bit;

            this.SetBitMaskForRegister(MFRC522.TxControlReg, Tx12RFEnabled_bits);
        }

        private void SetBitMaskForRegister(byte address, byte bitmask, bool log = false)
        {
            byte temp = this.ReadRegister(address);
            temp = (byte) (temp | bitmask);
            if (log)
            {
                System.Console.WriteLine($".set={address:X2}h, val={temp:X2}h");
            }
            this.WriteRegister(address, temp);
        }

        private void ClearBitMaskForRegister(byte address, byte bitmask, bool log = false)
        {
            byte temp = this.ReadRegister(address);
            temp = (byte) (temp & (~bitmask));
            if (log)
            {
                System.Console.WriteLine($".clr={address:X2}h, val={temp:X2}h.");
            }
            this.WriteRegister(address, temp);
        }

        private void WriteRegister(byte address, byte value, bool log = false)
        {
            Span<byte> buffer = stackalloc byte[2]
            {
            GetAddressByte(address),
            value
            };
            if (log)
            {
                System.Console.WriteLine($".wri={address:X2}h, buf={buffer[1]:X2}h");
            }

            _spiDevice.Write(buffer);
        }

        private byte ReadRegister(byte address, bool log = false)
        {
            Span<byte> buffer = stackalloc byte[2]
            {
            GetAddressByte(address, true),
            0
            };
            _spiDevice.TransferFullDuplex(buffer, buffer);
            if (log)
            {
                System.Console.WriteLine($".read={address:X2}h, buf={buffer[1]:X2}h");
            }
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