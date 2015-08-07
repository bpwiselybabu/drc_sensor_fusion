#Message for representing the status of the hydrolic pump on the Atlas robot
Header header                 #header data

float32 pump_inlet_pressure   #AtlasHydraulicPressureSetting pump_pressure
float32 pump_supply_pressure  #Pressure at pump supply, in PSI. 
#float32 pump_return_pressure  #Pump return pressure, in PSI. 
float32 air_sump_pressure     #Air pressure in accusump, in PSI
float32 pump_time_meter       #How many hours pump has been run since installation
float32 current_pump_rpm      #Current RPM of pump
float32 pump_supply_temperature
float32 motor_temperature
float32 motor_driver_temperature