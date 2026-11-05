Tests show that Game Boy Camera does not act on 'status | 0x40' bytes for certain INQUIRY commands. 
The result follows: 
no error shown on inquiry1 
no error shown on inquiry2
no error shown on inquiry3
no error shown on inquiry4
no error shown on inquiry5
no error shown on inquiry6
error shown on inquiry7 LED ON (led on means that the error was read and acted after the print command)
error shown on inquiry8 LED ON
error shown on inquiry9 LED ON
(later inquiry packets were not tested and only the Nth INQUIRY packet sent out 'status | 0x40'.)

The mapping of the packets follows (from captures in mofosyne's arduino-gameboy-printer-emulator repository): 
INIT
DATA1 INQUIRY1
DATA2  DATA3  INQUIRY2
DATA4  DATA5  INQUIRY3
DATA6  DATA7  INQUIRY4
DATA8  DATA9  DATA10 (data10 is empty end marker)
PRINT
INQUIRY(5,6,7...) (looped until status changes from busy (ignoring INQUIRY5 and INQUIRY6)) (the camera does not act on 'status | 0x40' from INQUIRY5 and INQUIRY6, but behavior is not fully tested.)

Tests show that the camera cannot be stopped mid transfer from INQUIRY packets and ignores 2 polls after PRINT.