using System;
using System.Device.Spi;
using System.Text;
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
        // Anti collision/Select, Cascade Level 1
        private const byte PICC_COMMAND_SEL_CL1 = 0x93;

        static void Main(string[] args)
        {
            var connection = new SpiConnectionSettings(0, 0);
            connection.ClockFrequency = 1000000;
            var spi = SpiDevice.Create(connection);

            var mfrc = new MFRC522(spi);

            // Console.WriteLine("Running self test...");
            // mfrc.SelfTest();

            mfrc.Init();
            Console.WriteLine("Start scanning for tags...");
            while (true)
            {
                var requestStatus = mfrc.Request(PICC_COMMAND_REQUEST_A);
                if (requestStatus == Status.OK)
                {
                    var(antiCollStatus, serialNumber) = mfrc.AntiCollision(PICC_COMMAND_SEL_CL1);
                    var sb = new StringBuilder();
                    foreach (var dataByte in serialNumber)
                    {
                        sb.Append($"{dataByte:X2}");
                    }
                    System.Console.WriteLine($"Status={antiCollStatus}, data={sb.ToString()}");
                }
                Thread.Sleep(500);
            }
        }
    }
}