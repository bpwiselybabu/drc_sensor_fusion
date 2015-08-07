#Message for representing the status of the battery
Header header                 #header data

int32 charging
float32    voltage
float32    current
float32    remaining_time     # seconds of battery life remaining
float32    remaining_amp_hours
float32    remaining_charge_percentage
int32      cycle_count        # battery cycles - not relevant on emulator
