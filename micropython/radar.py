from ulab import numpy as np
import math
import time
import micropython
import machine
import struct
import os

c = 299792458.0
B = 63e9 - 58e9

class RadarSensor:
    def __init__(self, spi, cs_pin, debug=False):
        self.spi = spi
        self.cs = machine.Pin(cs_pin, machine.Pin.OUT)
        self.cs.value(1)  # CS is active low, so initialize it to high
        self.debug = debug  # Debug flag for enabling/disabling print statements
        self.fifo_data = None

        # Register map
        self.register_map = {
            0x0000: "MAIN",
            0x0001: "ADC0",
            0x0004: "PACR1",
            0x0005: "PACR2",
            0x0006: "SFCTL",
            0x000B: "CSCI",
            0x000F: "CSCDS",
            0x0010: "CS1_U_0",
            0x0011: "CS1_U_1",
            0x0012: "CS1_U_2",
            0x0016: "CS1",
            0x001D: "CS2",
            0x0024: "CS3",
            0x002B: "CS4",
            0x002C: "CCR0",
            0x002D: "CCR1",
            0x002E: "CCR2",
            0x002F: "CCR3",
            0x0030: "PLL1_0",
            0x0031: "PLL1_1",
            0x0032: "PLL1_2",
            0x0033: "PLL1_3",
            0x0037: "PLL1_7",
            0x003F: "PLL2_7",
            0x0047: "PLL3_7",
            0x004F: "PLL4_7",
            0x005F: "FD",
            0x0060: "WU",
            0x0061: "STAT0",
            0x0063: "FSTAT",
            0x0003: "STAT1"
        }

    def _write_register(self, register, value):
        """
        Write a 24-bit value to the specified register.
        """
        # Ensure register is an integer address
        if isinstance(register, str):
            register = next(key for key, val in self.register_map.items() if val == register)

        cmd_byte = (register << 1) | 1  # Set R/W bit to 1 for write
        data = struct.pack('>I', value)[1:]  # Pack value into 4 bytes, use only last 3

        if self.debug:
            print(f"Writing to {register}: CMD=0x{cmd_byte:02X}, DATA=0x{value:06X}")

        self.cs.value(0)  # CS low to start communication
        self.spi.write(bytes([cmd_byte]) + data)  # Send command byte + 3 data bytes
        self.cs.value(1)  # CS high to end communication

    def _read_register(self, register):
        """
        Read a 24-bit value from the specified register.
        """
        # Ensure register is an integer address
        if isinstance(register, str):
            register = next(key for key, val in self.register_map.items() if val == register)

        cmd_byte = (register << 1) & ~1  # Clear R/W bit for read

        if self.debug:
            print(f"Reading from {register}: CMD=0x{cmd_byte:02X}")

        tx_data = bytes([cmd_byte]) + b'\x00\x00\x00'
        rx_data = bytearray(4)

        self.cs.value(0)  # CS low to start communication
        self.spi.write_readinto(tx_data, rx_data)
        self.cs.value(1)  # CS high to end communication

        gsr0 = rx_data[0]
        register_value = struct.unpack('>I', b'\x00' + rx_data[1:])[0] & 0xFFFFFF  # Mask 24-bit data

        if self.debug:
            print(f"GSR0: 0x{gsr0:02X}, Register {register} value: 0x{register_value:06X}")

        return register_value

    def populate_registers_from_flash(self):
        """
        Populate the radar sensor registers with predefined register-value pairs from a file in flash.
        The file format is 'name address value', where the value should be reduced by removing padded zeros.
        Prints a warning if the written value does not match the read-back value.
        """
        try:
            # Open the 'regs' file from flash
            with open('regs', 'r') as f:
                for line in f:
                    # Split the line into name, address, and value
                    parts = line.strip().split()
                    if len(parts) != 3:
                        continue  # Skip lines that don't have exactly 3 parts

                    name = parts[0]
                    address = int(parts[1], 16)  # Convert address to integer
                    value = int(parts[2], 16)  # Convert value to integer

                    # No need to shift the values, just directly use them
                    # Write the value to the register
                    self._write_register(address, value)

                    # Read back the value
                    read_value = self._read_register(address)

                    # Print the written and read-back values for debugging
                    if self.debug:
                        # Format the print statement to avoid padded zeros
                        print(f"Written Register {name} (0x{address:X}): 0x{value:X}, Read Back: 0x{read_value:X}")

                    # Check if the written value matches the read-back value
                    if value != read_value:
                        print(
                            f"Warning: Mismatch for Register {name} (0x{address:X}). Written: 0x{value:X}, Read Back: 0x{read_value:X}")

        except FileNotFoundError:
            print("Error: 'regs' file not found in flash.")

    def populate_registers(self):
        """
        Populate the radar sensor registers with predefined register-value pairs.
        Prints a warning if the written value does not match the read-back value.
        """
        register_values = {
            0x01: 0x050010,  # ADC0    => 00001010 00000010 00010000
            0x04: 0xE967FD,  # PACR1   => 11101001 01100111 11111101
            0x05: 0x4805B4,  # PACR2   => 01001000 00000101 10110100
            # 0x06: 0x1087FF,  # SFCTL   => 00010000 10000111 11111111
            0x0B: 0xD0D9E0,  # CSCI    => 11010000 11011001 11100000
            0x0F: 0x000960,  # CSCDS   => 00000000 00001001 01100000
            0x10: 0x003C51,  # CS1_U_0 => 00000000 00111100 01010001
            0x11: 0x14041F,  # CS1_U_1 => 00010100 00000100 00011111
            0x12: 0x00000B,  # CS1_U_2 => 00000000 00000000 00001011
            0x16: 0x000490,  # CS1     => 00000000 00000100 10010000
            0x1D: 0x000480,  # CS2     => 00000000 00000100 10000000
            0x24: 0x000480,  # CS3     => 00000000 00000100 10000000
            0x2B: 0x000480,  # CS4     => 00000000 00000100 10000000
            0x2C: 0x11BE0E,  # CCR0    => 00010001 10111110 00001110
            0x2D: 0x989c0a,  # CCR1    => 01100010 11111100 00001010
            0x2E: 0x000000,  # CCR2    => 00000011 11110000 00000000
            0x2F: 0xBF3E1E,  # CCR3    => 10111111 00111110 00011110
            0x30: 0xa83662,  # PLL1_0  => 10100011 01010011 01101000
            0x31: 0x00030d,  # PLL1_1  => 00000000 00001011 01100100
            0x32: 0x000532,  # PLL1_2  => 00000000 00010111 00100010
            0x33: 0x000200,  # PLL1_3  => 00000000 00000000 00100000
            0x37: 0x000110,  # PLL1_7  =>
            0x3F: 0x000100,  # PLL2_7  => 00000000 00000001 00000000
            0x47: 0x000100,  # PLL3_7  => 00000000 00000001 00000000
            0x4F: 0x000100,  # PLL4_7  => 00000000 00000001 00000000
            0x50: 0x0a0000,  # ADC1
            0x5F: 0x000400,  # FD      => 00000000 00000100 00000000
            0x60: 0x000827,  # WU      => 00000000 10000010 01110111
            0x00: 0x1C0E20,  # MAIN    => 00011100 00001110 00100000
        }

        for register, value in register_values.items():
            # Write the value to the register
            self._write_register(register, value)

            # Read back the value
            read_value = self._read_register(register)

            # Print the written and read-back values for debugging
            if self.debug:
                print("Written Register 0x%04X: 0x%08X, Read Back: 0x%08X" % (register, value, read_value))

            # Check if the written value matches the read-back value
            if value != read_value:
                print("Warning: Mismatch for Register 0x%04X. Written: 0x%08X, Read Back: 0x%08X" % (
                    register, value, read_value))

    def wait_for_adc_ready(self, timeout=5):
        """
        Poll the ADC status to check if it becomes ready within a timeout.
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            stat0 = self._read_register('STAT0')
            adc_ready = (stat0 >> 1) & 0x1
            if adc_ready:
                if self.debug:
                    print("ADC is ready.")
                return True
            time.sleep(0.1)  # Poll every 100ms

        print("Timeout: ADC is not ready.")
        return False

    def enable_oscillator(self):
        """
        Enable the oscillator clock by setting OSCCLKEN in the PACR1 register (bit 23).
        """
        # Read the current value of the PACR1 register
        pacr1 = self._read_register('PACR1')

        # Set the OSCCLKEN bit (bit 23) to 1
        pacr1 |= (1 << 23)

        # Write the modified value back to the PACR1 register
        self._write_register('PACR1', pacr1)

        # Verify the change
        pacr1_check = self._read_register('PACR1')
        if self.debug:
            print(f"Oscillator enabled: PACR1 = 0x{pacr1_check:06X}")

    def check_ldo_adc_ready(self):
        """
        Check if LDO and ADC are ready by reading the STAT0 register.
        """
        stat0 = self._read_register('STAT0')

        # Check LDO_RDY (bit 3)
        ldo_ready = (stat0 >> 3) & 0x1
        # Check ADC_RDY (bit 1)
        adc_ready = (stat0 >> 1) & 0x1

        if ldo_ready:
            print("LDO is ready.")
        else:
            print("LDO is not ready.")

        if adc_ready:
            print("ADC is ready.")
        else:
            print("ADC is not ready.")

        return ldo_ready, adc_ready

    def initialize_sensor(self):
        """
        Initialize the radar sensor by configuring PLL, ADC, FSM, and enabling the oscillator.
        """
        # Populate the radar's registers with predefined values
        self.populate_registers_from_flash()

        # Enable the oscillator clock
        self.enable_oscillator()

        self.enable_madc_bbch1()

        # Reset the FIFO to ensure it starts clean
        self.reset('fifo')

        if self.debug:
            print("Sensor initialized.")

    def reset(self, reset_type):
        """
        Reset the radar system based on the reset type.

        :param reset_type: 'fsm' for FSM_RESET, 'fifo' for FIFO_RESET, 'sw' for SW_RESET.
        """
        # Mapping of reset types to corresponding bits in the MAIN register
        reset_map = {
            'fsm': 2,  # FSM_RESET is bit 2
            'fifo': 3,  # FIFO_RESET is bit 3
            'sw': 1,  # SW_RESET is bit 1
        }

        if reset_type not in reset_map:
            raise ValueError(f"Invalid reset type: {reset_type}. Valid types are 'fsm', 'fifo', 'sw'.")

        # Read the current value of the MAIN register
        main_reg = self._read_register('MAIN')

        # Set the appropriate reset bit
        main_reg |= (1 << reset_map[reset_type])

        # Write the modified value back to the MAIN register
        self._write_register('MAIN', main_reg)

        # Debugging information
        if self.debug:
            print(f"{reset_type.upper()} reset: MAIN = 0x{main_reg:06X}")

    def enable_madc_bbch1(self):
        """
        Enable MADC_BBCH1_EN bit (bit 20) to enable the baseband filters, amplifiers, and ADC on channel 1.
        """
        # Read the current value of the register at address 0x011 (Channel Set Up Register)
        channel_set_reg = self._read_register(0x011)

        # Set the MADC_BBCH1_EN bit (bit 20) to 1
        channel_set_reg |= (1 << 20)

        # Write the modified value back to the register
        self._write_register(0x011, channel_set_reg)

        # Verify the change
        channel_set_reg_check = self._read_register(0x011)
        if self.debug:
            print(f"MADC_BBCH1_EN enabled: Register 0x011 = 0x{channel_set_reg_check:08X}")

    def check_status_registers(self):
        """
        Check the status of the radar using STAT0, STAT1, and FSTAT registers.
        """
        # Check if the ADC is ready using polling
        # adc_ready = self.wait_for_adc_ready()
        # if not adc_ready:
        #     print("ADC is still not ready.")

        # Check the power mode of the FSM
        stat0 = self._read_register('STAT0')
        power_mode = (stat0 >> 5) & 0x7
        print(f"FSM Power Mode: {power_mode}")

        # Read the FIFO status register (0x063)
        fstat = self._read_register('FSTAT')

        # Check for FIFO overflow, underflow, and empty status
        fifo_empty = (fstat >> 20) & 0x1
        fifo_full = (fstat >> 22) & 0x1
        fifo_overflow = (fstat >> 23) & 0x1

        if fifo_empty:
            print("FIFO is empty.")
        if fifo_full:
            print("FIFO is full.")
        if fifo_overflow:
            print("FIFO overflow occurred.")
        else:
            print("FIFO status OK.")

    def wait_for_data(self, timeout=5):
        """
        Poll the FIFO status to check if data is available.
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            fstat = self._read_register('FSTAT')
            fifo_empty = (fstat >> 20) & 0x1
            if not fifo_empty:
                if self.debug:
                    print("Data available in FIFO.")
                return True
            time.sleep(0.1)  # Wait for 100ms before polling again

        print("Timeout: No data in FIFO.")
        return False

    def burst_read_fifo(self, num_blocks=2048):
        """
        Perform a burst read from the FIFO in blocks (each block is 3 bytes or 24 bits).
        Return the result as a single bytearray structure.
        """
        # Construct the burst read command:
        # - ADDR (31:25): 0x7F for burst mode
        # - RW (24): 1 for write mode (set bit 24 to 1)
        # - SADDR (23:17): 0x64 for FIFO access
        # - RWB (16): 0 for read burst
        # - NBURSTS (15:9): 0 for unbounded burst access
        # - RSVD (8:0): Reserved, set to 0
        burst_cmd = (0x7F << 25) | (1 << 24) | (0x64 << 17) | (0 << 16) | (0 << 9)

        if self.debug:
            print(f"Burst Read Command: 0x{burst_cmd:08X}")

        self.cs.value(0)  # CS low to start communication

        # Prepare the command to be sent (32 bits)
        tx_data = struct.pack('>I', burst_cmd)
        self.spi.write(tx_data)

        # Create a single bytearray to hold all the received data blocks
        all_data = bytearray()

        # Read the FIFO data in blocks (each block is 3 bytes)
        for _ in range(num_blocks):
            block = bytearray(3)  # Each block is 24 bits (3 bytes)
            self.spi.readinto(block)
            all_data.extend(block)  # Append each block to the large bytearray

        self.cs.value(1)  # CS high to end communication

        return all_data

    def run_frame(self):
        # Frame start a frame, then reset the FSM
        self.start_frame()
        self.reset('fsm')

    def measure(self, skip=12):
        self.reset('fifo')
        self.start_frame()
        # self.fifo_status()
        self.read_fifo()
        # self.write_fifo_to_flash()
        # self.fifo_status()
        da = DistanceAlgo(512)
        distance, data = da.compute_distance_from_data(self.fifo_data, 512, skip)
        return distance
        # print(f"Data: {data}")

    def start_frame(self):
        """
        Start the radar frame by setting the FRAME_START bit (bit 0) in the MAIN register.
        """
        # 1. Read the current value of the MAIN register
        main_reg = self._read_register('MAIN')

        # 2. Set the FRAME_START bit (bit 0) to 1 in the MAIN register
        main_reg |= (1 << 0)  # Bit 0 corresponds to FRAME_START

        # 3. Write the modified value back to the MAIN register
        self._write_register('MAIN', main_reg)

        # 4. If debug mode is enabled, print the current MAIN register value
        if self.debug:
            print(f"Radar frame started: MAIN = 0x{main_reg:06X}")

    def read_fifo(self, num_blocks=2048):
        # Read fifo into memory
        self.fifo_data = self.burst_read_fifo(num_blocks=num_blocks)

    def write_fifo_to_flash(self):
        # Write the contents of the FIFO to a file
        try:
            os.remove('fifo_data.bin')
        except:
            pass
        with open('fifo_data.bin', 'wb') as file:
            file.write(self.fifo_data)

    def fifo_status(self):
        """
        Check the FIFO fill status and return the percentage full.
        Also check for FIFO overflow using FSTAT:FOF_ERR or GSR0:FOF_ERR.
        """
        # Read FILL_STATUS (bits 0-13 from FSTAT register)
        fstat = self._read_register('FSTAT')

        # Extract the FIFO fill status (bits 13:0)
        fifo_fill_status = fstat & 0x3FFF  # Mask for the lower 14 bits (13:0)

        # Calculate the fill percentage
        fill_percentage = (fifo_fill_status / 0x0800) * 100

        # Check for FIFO overflow (bit 23)
        if fstat & (1 << 23):  # FIFO overflow bit
            print("FIFO overflow error detected.")

        print(f"FIFO fill percentage: {fill_percentage:.2f}%")

        return fill_percentage

def save_array_to_file(arr, filename="data.txt"):
    with open(filename, 'w') as f:
        for item in arr:
            if isinstance(item, complex):
                # Save real and imaginary parts separately
                f.write("{:.6f},{:.6f}\n".format(item.real, item.imag))
            else:
                # Save regular floats
                f.write("{:.6f}\n".format(float(item)))


@micropython.native
def extract_samples(data, num_samples):
    samples = [0] * num_samples  # Preallocate list for better performance
    sample_index = 0
    data_length = len(data)
    i = 0

    # Process data in chunks of 3 bytes to extract two 12-bit samples each
    while i + 2 < data_length and sample_index < num_samples:
        b0, b1, b2 = data[i], data[i+1], data[i+2]

        # First 12-bit sample
        samples[sample_index] = (b0 << 4) | (b1 >> 4)
        sample_index += 1
        if sample_index >= num_samples:
            break

        # Second 12-bit sample
        samples[sample_index] = ((b1 & 0x0F) << 8) | b2
        sample_index += 1
        i += 3  # Move to the next set of 3 bytes

    return samples

# Return the number of samples extracted

def blackmanharris(M):
    """
    Generate a Blackman-Harris window.
    M: number of samples.
    """
    if M < 1:
        return []

    a0 = 0.35875
    a1 = 0.48829
    a2 = 0.14128
    a3 = 0.01168

    window = []
    for n in range(M):
        term1 = a1 * math.cos(2.0 * math.pi * n / (M - 1))
        term2 = a2 * math.cos(4.0 * math.pi * n / (M - 1))
        term3 = a3 * math.cos(6.0 * math.pi * n / (M - 1))
        value = a0 - term1 + term2 - term3
        window.append(value)

    return window

class DistanceAlgo:
    """Algorithm for computation of distance FFT from raw data"""

    def __init__(self, num_samples):
        self.num_samples = num_samples
        self.bandwidth_hz = B

        # Compute Blackman-Harris Window over chirp samples
        self.range_window = np.array(blackmanharris(num_samples))

        # FFT size with zero-padding
        fft_size = num_samples * 2
        self.range_bin_length = c / (2 * self.bandwidth_hz * fft_size / num_samples)

    # Compute distance from passed in data, extract samples from data
    def compute_distance_from_data(self, data, size, skip=12):
        samples = extract_samples(data, size)
        return self.compute_distance(samples, skip)

    def compute_distance(self, chirp_data, skip=12):
        """
        Computes distance using chirp data (single chirp).

        :param chirp_data: Input data from chirp (ulab array)
        :param skip: Number of initial samples to skip in peak search
        :return: Tuple containing peak distance in meters and absolute FFT spectrum
        """

        # Step 1 - Calculate range FFT spectrum of the frame
        range_fft = self.fft_spectrum(chirp_data, self.range_window)

        # Step 2 - Convert to absolute spectrum
        range_fft_abs = abs(range_fft)

        # Step 3 - Vectorized Summation
        self.distance_data = range_fft_abs / self.num_samples

        # Step 4 - Peak search and distance calculation
        distance_peak = np.argmax(self.distance_data[skip:])
        distance_peak_m = self.range_bin_length * (distance_peak + skip)

        return distance_peak_m, range_fft_abs

    def compute_distance_old(self, chirp_data, skip=12):
        # Computes distance using chirp data (single chirp)

        #save_array_to_file(chirp_data, "data/1-chirp_data.txt")

        # Step 1 - calculate range FFT spectrum of the frame
        range_fft = self.fft_spectrum(chirp_data, self.range_window)
        #save_array_to_file(range_fft, "data/2-fft.txt")

        # Step 2 - convert to absolute spectrum
        range_fft_abs = abs(range_fft)
        #save_array_to_file(range_fft_abs, "data/3-fft-abs.txt")

        # Manually sum along axis=0 (since range_fft_abs is likely a 1D array)
        distance_data = np.zeros(len(range_fft_abs))
        for i in range(len(range_fft_abs)):
            distance_data[i] = range_fft_abs[i] / self.num_samples

        #save_array_to_file(distance_data, "data/4-distance.txt")

        # Step 3 - peak search and distance calculation
        distance_peak = np.argmax(distance_data[skip:])

        distance_peak_m = self.range_bin_length * (distance_peak + skip)
        return distance_peak_m, range_fft_abs

    def fft_spectrum(self, chirp_data, range_window):
        # Calculate FFT spectrum
        # chirp_data: single chirp data (1D array)
        # range_window: window applied on input data before FFT

        # Step 1 - remove DC bias from the single chirp
        avgs = sum(chirp_data) / len(chirp_data)  # Manually compute the average

        # Subtract the average from each element in chirp_data
        chirp_data = [sample - avgs for sample in chirp_data]

        # Convert chirp_data to numpy array for further operations
        chirp_data = np.array(chirp_data)

        # Step 2 - Windowing the Data
        chirp_data = chirp_data * range_window

        #print(f"chirp_data: {chirp_data}")

        # Step 3 - Add zero padding manually
        # Manually append zeros for padding
        padded_chirp = np.zeros(len(chirp_data) * 2)
        for i in range(len(chirp_data)):
            padded_chirp[i] = chirp_data[i]

        # Step 4 - Compute FFT for distance information
        range_fft = np.fft.fft(padded_chirp) / self.num_samples

        # Step 5 - Ignore redundant negative spectrum and double the magnitude
        range_fft = 2 * range_fft[:self.num_samples]

        return range_fft


