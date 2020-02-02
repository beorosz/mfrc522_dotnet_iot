namespace RFID522
{
    public class Register
    {
        public const byte CommandReg = 0x01; // Starts and stops command execution.
        public const byte ComIEnReg = 0x02; // Control bits to enable and disable the passing of interrupt requests.
        public const byte ComIrqReg = 0x04; // Interrupt request bits.
        public const byte ErrorReg = 0x06; // Error bit register showing the error status of the last command executed.
        public const byte FIFODataReg = 0x09; // Input and output of 64 byte FIFO buffer.
        public const byte FIFOLevelReg = 0x0A; // Indicates the number of bytes stored in the FIFO.
        public const byte WaterLevelReg = 0x0B; // Defines the level for FIFO under- and overflow warning.
        public const byte ControlReg = 0x0C; // Miscellaneous control bits.
        public const byte BitFramingReg = 0x0D; // Adjustments for bit-oriented frames.
        public const byte CollReg = 0x0E; // Defines the first bit-collision detected on the RF interface.
        public const byte ModeReg = 0x11; // Defines general mode settings for transmitting and receiving.
        public const byte TxModeReg = 0x12; // Defines the data rate during transmission.
        public const byte RxModeReg = 0x13; // Defines the data rate during reception.
        public const byte TxControlReg = 0x14; // Controls the logical behavior of the antenna driver pins TX1 and TX2.
        public const byte TxASKReg = 0x15; // Controls transmit modulation settings.
        public const byte ModWidthReg = 0x24; // Sets the modulation width.
        public const byte TModeReg = 0x2A; // Defines the timer settings.
        public const byte TPrescalerReg = 0x2B; // Defines the timer settings.
        public const byte TReloadRegH = 0x2C; // Defines the 16-bit timer reload value (high 8 bit).
        public const byte TReloadRegL = 0x2D; // Defines the 16-bit timer reload value (low 8 bit).
        public const byte TCounterValRegH = 0x2E; // The timer value bits are contained in two 8-bit registers.
        public const byte TCounterValRegL = 0x2F; // The timer value bits are contained in two 8-bit registers.
        public const byte AutoTestReg = 0x36; // Controls the digital self-test.
        public const byte VersionReg = 0x37; // Shows the MFRC522 software version.
    }
}