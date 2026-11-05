Tests show that Game Boy Camera does not show an error for missing 0x81 bytes on inquiry packets.
Only one reply was missing 0x81 in each run.
Tested positions 1, 4, 6, 7, 9.

The mapping of the packets follows (from captures in mofosyne's arduino-gameboy-printer-emulator repository): 
INIT
DATA1 INQUIRY1
DATA2  DATA3  INQUIRY2
DATA4  DATA5  INQUIRY3
DATA6  DATA7  INQUIRY4
DATA8  DATA9  DATA10 (data10 is empty end marker)
PRINT
INQUIRY(5,6,7...)