# NTURT Radio Communicator

1. resolve can't find ttyUSB* devices:
    - used dmesg | grep "tty" and found that the usb device was closed inmmediately after connection # brltty
    - resolved by : https://unix.stackexchange.com/questions/670636/unable-to-use-usb-dongle-based-on-usb-serial-converter-chip
2. code source: https://stackoverflow.com/questions/58952752/input-output-error-when-getting-termios-attributes-on-ttys0-serial-port