using System;
using System.Device.Spi;
using System.Threading;

namespace RFID522
{
    public class Program
    {
        // Abbrevations:
        // PICC - Proximity integrated circuit card
        // PCD - Proximity coupling device

        // Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection
        private const byte PICC_COMMAND_REQUEST_A = 0x26;

        static void Main(string[] args)
        {
            var connection = new SpiConnectionSettings(0, 0);
            connection.ClockFrequency = 500000;
            var spi = SpiDevice.Create(connection);

            var mfrc = new MFRC522(spi);

            Console.WriteLine("Starting self test...");
            mfrc.SelfTest();

            mfrc.Init();

            while (true)
            {
                var result = mfrc.Request(PICC_COMMAND_REQUEST_A);

                if (result.status != 0)
                    continue;

                Thread.Sleep(500);
            }
        }
    }
}