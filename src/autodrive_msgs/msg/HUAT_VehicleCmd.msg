uint8 head1            # Sync byte 1, fixed 0xAA
uint8 head2            # Sync byte 2, fixed 0x55
uint8 length           # Frame payload length [bytes]
uint8 steering         # Steering angle command [0-220, center=110, unit: 0.1 deg offset]
uint8 brake_force      # Brake force [0-100, unit: %]
uint8 pedal_ratio      # Throttle pedal ratio [0-100, unit: %]
uint8 gear_position    # Gear position [0=N, 1=D, 2=R]
uint8 working_mode     # Working mode [0=manual, 1=autonomous, 2=EBS]
uint8 racing_num       # Racing number identifier [0-255]
uint8 racing_status    # Racing status [0=idle, 1=ready, 2=running, 3=finished]
uint16 checksum        # Sum-of-bytes checksum over preceding fields [little-endian]
