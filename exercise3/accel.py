import sys

class Accel:
    def __init__(self, name):
        self.name = name
        self.fsr_map = {
            '2g': 16384,
            '4g': 8192,
            '8g': 4096,
            '16g': 2048
        }
        
    def send_command(self, command):
        # sends a command to the accelerometer
        # returns the accelerometer response
        pass
    
    def read_acceleration(self):
        # returns a list of 3 acceleration axis
        # [AXIS_X, AXIS_Y, AXIS_Z]
        # each axis is a 16 bit signed value
        pass
    
    def read_fsr(self):
        fsr = self.send_command('f')  
        # assuming the response is always “2g”, “4g”, “8g”, “16g”
        return self.fsr_map.get(fsr, 16384)  
    
    def read_bias(self):
        bias = self.send_command('b')
        # assuming the response is always “<axis_x>,<axis_y>,<axis_z>”
        return list(map(float, bias.split(',')))

def main():
    # Read device names from command line arguments
    device_names = sys.argv[1:]
    
    if not device_names:
        print("You must specify the accelerometer BLE names")
        sys.exit(1)
        
    if len(device_names) > 5:
        print("You can only specify up to 5 accelerometers")
        sys.exit(1)
    
    # Create Accel objects for each device
    accels = [Accel(name) for name in device_names]
    
    # Read acceleration data from each device
    for accel in accels:
        bias_x, bias_y, bias_z = accel.read_bias()
        fsr = accel.read_fsr()
        measured_x, measured_y, measured_z = accel.read_acceleration()

        # Convert raw acceleration values to g-units
        measured_x_in_g = measured_x / fsr
        measured_y_in_g = measured_y / fsr
        measured_z_in_g = measured_z / fsr
        
        # Apply bias correction
        corrected_x = measured_x_in_g - bias_x
        corrected_y = measured_y_in_g - bias_y
        corrected_z = measured_z_in_g - bias_z
        
        # Check if the device is in a perfectly horizontal position
        tolerance = 0.02  # Allow small variations
        if abs(corrected_x) < tolerance and abs(corrected_y) < tolerance and corrected_z > 0:
            print(f"{accel.name} is in a perfectly horizontal position")
        else:
            print(f"{accel.name} is not in a perfectly horizontal position")

if __name__ == "__main__":
    main()