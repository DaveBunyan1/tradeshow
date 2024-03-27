import time
import smbus
import bme680
from pyModbusTCP.client import ModbusClient
from pyModbusTCP.utils import encode_ieee, decode_ieee, \
                              long_list_to_word, word_list_to_long



class FloatModbusClient(ModbusClient):
    """A ModbusClient class with float support."""

    def read_float(self, address, number=1):
        """Read float(s) with read holding registers."""
        reg_l = self.read_holding_registers(address, number * 2)
        if reg_l:
            return [decode_ieee(f) for f in word_list_to_long(reg_l)]
        else:
            return None

    def write_float(self, address, floats_list):
        """Write float(s) with write multiple registers."""
        b32_l = [encode_ieee(f) for f in floats_list]
        b16_l = long_list_to_word(b32_l)
        return self.write_multiple_registers(address, b16_l)
    
HEARTBEAT_FILE = '/home/pi/heartbeat.txt'

# Define the I2C address of the FS3000 sensor
# FS3000_ADDRESS = 0x28
XENSIV_ADDRESS = 0x28

# Create an instance of the SMBus
# airbus = smbus.SMBus(0)
bus = smbus.SMBus(1)  # Use 1 for I2C bus 1

# Function to read data from the FS3000 sensor
# def read_fs3000_data():
#     # Send command to read data from FS3000
#     airbus.write_byte(FS3000_ADDRESS, 0x80)  # Assuming 0x80 is the command for reading data
# 
#     # Wait for a short time to ensure the sensor has processed the command
#     time.sleep(0.1)
# 
#     # Read data from the sensor
#     data = airbus.read_i2c_block_data(FS3000_ADDRESS, 0x00, 3)  # Assuming 3 bytes of data
# 
#     # Parse the received data
#     checksum = data[0]  # Checksum byte
#     flow_data = ((data[1] << 8) | data[2]) & 0xFFF  # Combine two bytes to get 12-bit flow data
# 
#     return checksum, flow_data

def main():
    # TCP auto connect on first modbus request
    c = FloatModbusClient(host="10.250.66.22", port=502, unit_id=1, auto_open=True)
    
    while True:
        # Write current timestamp to heartbeat file
        with open(HEARTBEAT_FILE, 'w') as f:
            f.write(str(time.time()))
        try:
            # Initialize the CO2 sensor
            id = bus.read_byte_data(XENSIV_ADDRESS, 0x00)
            print("ID:", hex(id))
            status = bus.read_byte_data(XENSIV_ADDRESS, 0x01)
            print("Status:", hex(status))
            # Idle mode
            bus.write_byte_data(XENSIV_ADDRESS, 0x04, 0x00)
            time.sleep(0.4)

            # Set measurement rate to 10 s
            bus.write_byte_data(XENSIV_ADDRESS, 0x02, 0x00)
            bus.write_byte_data(XENSIV_ADDRESS, 0x03, 0x0A)

            # Configure continuous mode
            bus.write_byte_data(XENSIV_ADDRESS, 0x04, 0x02)
        except:
            pass
        try:
            # Initialize the BME680 sensor
            sensor = bme680.BME680(i2c_addr=0x77)

            # Change this to the appropriate I2C address for your sensor if necessary
            sensor.set_gas_status(bme680.ENABLE_GAS_MEAS)

            # Change this to the appropriate humidity oversampling setting
            sensor.set_humidity_oversample(bme680.OS_2X)

            # Change this to the appropriate temperature oversampling setting
            sensor.set_temperature_oversample(bme680.OS_8X)

            # Change this to the appropriate pressure oversampling setting
            sensor.set_pressure_oversample(bme680.OS_4X)

            # Change this to the appropriate filter setting
            sensor.set_filter(bme680.FILTER_SIZE_3)

            # Change this to the appropriate gas heater temperature setting
            sensor.set_gas_heater_temperature(320)

            # Change this to the appropriate gas heater duration setting
            sensor.set_gas_heater_duration(150)
        except:
            pass
        # Start measuring
        while True:
            with open(HEARTBEAT_FILE, 'w') as f:
                f.write(str(time.time()))
#             try:    
#                 checksum, flow_data = read_fs3000_data()
#                 flow_data = (flow_data - 400) / 555.42
#                 print("Flow Data:", flow_data)
#                 c.write_float(0, [flow_data])
#             except:
#                 pass
            
            try:
                # Poll measurement status
                meas_sts = bus.read_byte_data(XENSIV_ADDRESS, 0x07)
                time.sleep(0.1)

                if meas_sts == 0x10:
                    # Get PPM value
                    value1 = bus.read_byte_data(XENSIV_ADDRESS, 0x05)
                    time.sleep(0.005)
                    value2 = bus.read_byte_data(XENSIV_ADDRESS, 0x06)
                    time.sleep(0.005)

                    # Calculate ppm value
                    result = (value1 << 8) | value2
                    print("CO2:", result, "ppm")
                    c.write_float(0, [result])
            except:
                pass
            try:
                if sensor.get_sensor_data():
                    temperature = sensor.data.temperature
                    pressure = sensor.data.pressure
                    humidity = sensor.data.humidity
                    gas_resistance = sensor.data.gas_resistance / 1000

                    print(f"Temperature: {temperature:.2f} °C")
                    print(f"Pressure: {pressure:.2f} hPa")
                    print(f"Humidity: {humidity:.2f} %")
                    print(f"Gas Resistance: {gas_resistance:.2f} kΩ\n")
                    
                    c.write_float(2, [temperature, pressure, humidity, gas_resistance])
            except:
                pass
            time.sleep(5)
            

if __name__ == "__main__":
    main()