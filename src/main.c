/*
 * BGT60 Radar Shell Interface using Sensor API
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/sensor/bgt60.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>

// Distance measurement constants
#define NUM_SAMPLES 512
#define FFT_SIZE 1024
#define SPEED_OF_LIGHT 299792458.0f
#define BANDWIDTH_HZ 5000000000.0f
#define PI 3.14159265358979323846f

// Blackman-Harris window coefficients
#define BH_A0 0.35875f
#define BH_A1 0.48829f
#define BH_A2 0.14128f
#define BH_A3 0.01168f

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

// Get the BGT60 device
static const struct device *bgt60_dev;

// GPIO for radar crystal power control
static const struct gpio_dt_spec radar_crystal = GPIO_DT_SPEC_GET(DT_ALIAS(radar_crystal), gpios);
static bool radar_powered = false;  // Default to OFF for safety

// Try to get the device node directly instead of using the macro
#define BGT60_NODE DT_NODELABEL(bgt60)

// Define RTIO I/O device for reading radar chirp data - try different approaches
#if DT_NODE_EXISTS(BGT60_NODE)
SENSOR_DT_READ_IODEV(iodev, BGT60_NODE, {SENSOR_CHAN_RADAR_CHIRP, 0});
#else
#error "BGT60 device node not found!"
#endif

// Define RTIO context (standard definition with memory pool support)
RTIO_DEFINE(ctx, 8, 8);

// Buffer for sensor data (3 bytes per FIFO block)
#define BLOCK_BYTES    3
#define BUF_SIZE       BLOCK_BYTES * CONFIG_BGT60_RADAR_SENSOR_DEFAULT_FIFO_BLOCKS
static uint8_t sensor_buffer[BUF_SIZE];

// Global state
static bool radar_initialized = false;
static int current_skip = 25;  // Default skip value, configurable

// Complex number structure for FFT
typedef struct {
	float real;
	float imag;
} complex_t;

// Distance processing variables
static float range_bin_length;
static float window_coeffs[NUM_SAMPLES];

// Data export buffers (static to persist between calls)
static uint16_t last_samples[NUM_SAMPLES];
static complex_t last_fft_data[FFT_SIZE];
static int last_num_samples = 0;
static bool data_available = false;


/**
 * @brief Extract 12-bit samples from raw FIFO data, skipping frame header
 * @param raw_data Raw FIFO data (3 bytes per block)
 * @param num_blocks Number of 3-byte blocks 
 * @param samples Output array for 12-bit samples
 * @return Number of samples extracted
 */
static int extract_samples(const uint8_t *raw_data, int num_blocks, uint16_t *samples)
{
	int sample_idx = 0;
	int start_block = 0;
	
	// Check for sync word at start of frame (datasheet section 5.1, Table 24)
	// Sync word is 0x000000 (24 bits across word #0)
	// Note: If MADC outputs 0x000000, it's automatically changed to 0x001
	if (num_blocks >= 3 && raw_data[0] == 0x00 && raw_data[1] == 0x00 && raw_data[2] == 0x00) {
		// Frame header present - structure per Table 24:
		// Word #0 (3 bytes): SYNC_WORD1 (bits 23:12) | SYNC_WORD0 (bits 11:0) = 0x000000
		// Word #1 (3 bytes): FRAME_CNT (bits 23:12) | SHAPE_GRP_CNT (bits 11:0)
		// Word #2 (3 bytes): TEMP_RESULT (bits 23:12) | POWER_RESULT (bits 11:0)
		// Word #3+ (3 bytes each): DATA0 (bits 23:12) | DATA1 (bits 11:0) - actual radar samples
		
		// Header is exactly 3 words (9 bytes = 3 blocks) before radar data starts
		start_block = 3;
	}
	// else: No sync word found, assume no header (PREFIX_EN disabled)
	
	// Extract samples starting after frame header
	for (int i = start_block; i < num_blocks; i++) {
		const uint8_t *block = &raw_data[i * 3];
		


		// Extract two 12-bit samples from 3 bytes (matching datasheet format)
		// DATA0 (MSB data): bits 23:12 = (b0 << 4) | (b1 >> 4)
		// DATA1 (LSB data): bits 11:0 = ((b1 & 0x0F) << 8) | b2
		samples[sample_idx++] = (block[0] << 4) | (block[1] >> 4);
		samples[sample_idx++] = ((block[1] & 0x0F) << 8) | block[2];
	}
	
	return sample_idx;
}

/**
 * @brief Remove DC bias from samples using manufacturer's recommended algorithm
 * Per manufacturer: For single chirp, calculate mean and subtract from each sample
 * @param samples Input/output sample array
 * @param num_samples Number of samples
 */
static void remove_dc_bias(uint16_t *samples, int num_samples)
{
	// Manufacturer's algorithm: Calculate average across the single chirp
	uint32_t sum = 0;
	for (int i = 0; i < num_samples; i++) {
		sum += samples[i];
	}
	uint16_t avg = sum / num_samples;
	
	// Remove DC bias by subtracting mean from each sample
	for (int i = 0; i < num_samples; i++) {
		if (samples[i] >= avg) {
			samples[i] -= avg;
		} else {
			samples[i] = 0;  // Prevent underflow
		}
	}
}




/**
 * @brief Initialize distance processing
 */
static void init_distance_processing(void)
{
	// Calculate range bin length
	range_bin_length = SPEED_OF_LIGHT / (2.0f * BANDWIDTH_HZ * FFT_SIZE / NUM_SAMPLES);
	
	// Precompute Blackman-Harris window coefficients
	for (int i = 0; i < NUM_SAMPLES; i++) {
		float n = (float)i;
		float M = (float)(NUM_SAMPLES - 1);
		window_coeffs[i] = BH_A0 
		                 - BH_A1 * cosf(2.0f * PI * n / M)
		                 + BH_A2 * cosf(4.0f * PI * n / M) 
		                 - BH_A3 * cosf(6.0f * PI * n / M);
	}
	
	// Convert to micrometers for integer display
	int range_bin_um = (int)(range_bin_length * 1000000);
	printk("Distance processing initialized: range_bin_length = %d.%06d m\n", 
	       range_bin_um / 1000000, range_bin_um % 1000000);
}


/**
 * @brief Bit-reverse permutation for FFT
 * @param x Input array
 * @param N Size (must be power of 2)
 */
static void bit_reverse(complex_t *x, int N)
{
	int j = 0;
	for (int i = 1; i < N; i++) {
		int bit = N >> 1;
		while (j & bit) {
			j ^= bit;
			bit >>= 1;
		}
		j ^= bit;
		if (i < j) {
			// Swap x[i] and x[j]
			complex_t temp = x[i];
			x[i] = x[j];
			x[j] = temp;
		}
	}
}

/**
 * @brief Radix-2 Cooley-Tukey FFT implementation
 * @param x Input/output array (modified in-place)
 * @param N Size (must be power of 2)
 * @param quiet Suppress debug output if true
 */
static void fft(complex_t *x, int N, bool quiet)
{
	// Bit-reverse permutation
	bit_reverse(x, N);
	
	// FFT computation
	for (int len = 2; len <= N; len <<= 1) {
		float angle = -2.0f * PI / len;
		complex_t wlen = {cosf(angle), sinf(angle)};
		
		for (int i = 0; i < N; i += len) {
			complex_t w = {1.0f, 0.0f};
			
			for (int j = 0; j < len / 2; j++) {
				complex_t u = x[i + j];
				
				// Complex multiplication: w * x[i + j + len/2]
				complex_t v;
				v.real = w.real * x[i + j + len/2].real - w.imag * x[i + j + len/2].imag;
				v.imag = w.real * x[i + j + len/2].imag + w.imag * x[i + j + len/2].real;
				
				x[i + j].real = u.real + v.real;
				x[i + j].imag = u.imag + v.imag;
				x[i + j + len/2].real = u.real - v.real;
				x[i + j + len/2].imag = u.imag - v.imag;
				
				// Update w = w * wlen (complex multiplication)
				complex_t w_new;
				w_new.real = w.real * wlen.real - w.imag * wlen.imag;
				w_new.imag = w.real * wlen.imag + w.imag * wlen.real;
				w = w_new;
			}
		}
	}
}

/**
 * @brief Calculate magnitude of complex number
 */
static float complex_magnitude(complex_t c)
{
	return sqrtf(c.real * c.real + c.imag * c.imag);
}

/**
 * @brief Proper FFT-based distance calculation matching Python implementation
 * @param windowed_samples Input windowed samples
 * @param num_samples Number of samples (should be 512)
 * @param skip Number of bins to skip at start (default 12)
 * @param quiet Suppress debug output if true
 * @return Distance in meters (or -1.0 if no peak found)
 */
// Static buffers to avoid stack overflow
static complex_t fft_data[FFT_SIZE];

static float calculate_distance_fft(const float *windowed_samples, int num_samples, int skip, bool quiet)
{
	// Copy samples to FFT input (real part only, zero imaginary part)
	for (int i = 0; i < num_samples; i++) {
		fft_data[i].real = windowed_samples[i];
		fft_data[i].imag = 0.0f;
	}
	
	// Zero-padding for high resolution FFT (manufacturer's recommendation)
	// This doubles the FFT size from 512 samples to 1024 points for better resolution
	for (int i = num_samples; i < FFT_SIZE; i++) {
		fft_data[i].real = 0.0f;
		fft_data[i].imag = 0.0f;
	}
	
	// Perform FFT (with reduced debug output)
	fft(fft_data, FFT_SIZE, quiet);
	
	// Save FFT data for export
	for (int i = 0; i < FFT_SIZE; i++) {
		last_fft_data[i] = fft_data[i];
	}
	
	// Calculate magnitude spectrum and find peak
	// Per manufacturer: Apply energy compensation (2x magnitude for positive spectrum)
	float max_magnitude = 0.0f;
	int peak_bin = -1;
	
	// Search first half of spectrum (skip DC and low-frequency noise)
	int end_bin = MIN(FFT_SIZE / 2, num_samples);  // Only search meaningful range
	
	// Track relative index within the searched range (matching Python logic)
	int relative_peak_bin = -1;
	
	for (int i = skip; i < end_bin; i++) {
		// Calculate magnitude
		float magnitude = complex_magnitude(fft_data[i]);
		// Manufacturer's algorithm: 2x energy compensation for positive spectrum
		// This matches Python: range_fft = 2 * range_fft[:, range(int(num_samples))]
		magnitude = 2.0f * magnitude;
		
		if (magnitude > max_magnitude) {
			max_magnitude = magnitude;
			relative_peak_bin = i - skip;  // Store relative index like Python
			peak_bin = i;  // Store absolute index for display
		}
	}
	
	if (!quiet) {
		// Convert magnitude to integer for display (x100 for 2 decimal places)
		int magnitude_int = (int)(max_magnitude * 100);
		printk("FFT peak found at bin %d with magnitude %d.%02d\n", peak_bin, magnitude_int / 100, magnitude_int % 100);
	}
	
	if (relative_peak_bin < 0 || max_magnitude < 1.0f) {  // Reasonable threshold for FFT magnitude
		return -1.0f;  // No valid peak found
	}
	
	// Convert bin to distance using proper range bin length (matching Python: distance_peak + skip)
	float distance = range_bin_length * (relative_peak_bin + skip);
	
	if (!quiet) {
		// Convert to millimeters for integer display
		int distance_mm = (int)(distance * 1000);
		int range_bin_um = (int)(range_bin_length * 1000000);
		printk("Distance calculation: bin %d × %d.%06d m/bin = %d.%03d m\n", 
		       peak_bin, range_bin_um / 1000000, range_bin_um % 1000000, 
		       distance_mm / 1000, distance_mm % 1000);
	}
	
	
	return distance;
}

/**
 * @brief Process raw radar data to calculate distance
 * @param raw_data Raw FIFO data
 * @param data_size Size of raw data in bytes
 * @param skip Number of bins to skip at start (default 12)
 * @param quiet Suppress debug output if true
 * @return Distance in meters (or -1.0 if processing failed)
 */
static float process_radar_distance(const uint8_t *raw_data, size_t data_size, int skip, bool quiet)
{
	if (!quiet) printk("Starting distance processing...\n");
	
	int num_blocks = data_size / 3;
	if (num_blocks < 32) {  // Need minimum amount of data for meaningful FFT
		if (!quiet) printk("Insufficient data for distance calculation: %d blocks (need at least 32)\n", num_blocks);
		return -1.0f;
	}
	
	// Extract samples from ALL available raw data - use static buffers to avoid stack overflow
	static uint16_t samples[512];  // Buffer for up to 256 blocks = 512 samples
	int max_blocks = MIN(num_blocks, 256);  // Use all available blocks up to 256
	int num_samples = extract_samples(raw_data, max_blocks, samples);
	
	if (!quiet) printk("Extracted %d samples from %d blocks\n", num_samples, max_blocks);
	
	// Save raw samples for export (before DC bias removal)
	for (int i = 0; i < num_samples && i < NUM_SAMPLES; i++) {
		last_samples[i] = samples[i];
	}
	last_num_samples = num_samples;
	data_available = true;
	
	// Remove DC bias
	remove_dc_bias(samples, num_samples);
	
	// Apply windowing to all samples - use static buffer
	static float windowed_samples[512];
	// Use Hann window for the full sample count
	for (int i = 0; i < num_samples; i++) {
		float window_val = 0.5f - 0.5f * cosf(2.0f * PI * i / (num_samples - 1));  // Hann window
		windowed_samples[i] = (float)samples[i] * window_val;
	}
	
	// Calculate distance using proper FFT
	float distance = calculate_distance_fft(windowed_samples, num_samples, skip, quiet);
	
	if (!quiet) printk("Distance processing complete\n");
	return distance;
}

// Shell command: radar init
static int cmd_radar_init(const struct shell *sh, size_t argc, char **argv)
{
	// Check if radar is powered on
	if (!radar_powered) {
		shell_error(sh, "Radar crystal is powered off! Run 'radar power on' first.");
		return -1;
	}
	
	shell_print(sh, "Initializing BGT60 radar using sensor API...");
	
	// Get the BGT60 device
	bgt60_dev = DEVICE_DT_GET(BGT60_NODE);
	
	if (!device_is_ready(bgt60_dev)) {
		shell_error(sh, "BGT60 device not ready!");
		return -1;
	}
	
	shell_print(sh, "✓ BGT60 device ready: %s", bgt60_dev->name);
	
	// The driver handles all initialization internally when device_is_ready() returns true
	// This includes:
	// - Register initialization
	// - Oscillator enable
	// - MADC enable
	// - FIFO reset
	
	radar_initialized = true;
	shell_print(sh, "✓ BGT60 initialization complete!");
	shell_print(sh, "  Buffer size: %d bytes (%d blocks)", BUF_SIZE, BUF_SIZE / BLOCK_BYTES);
	
	// Initialize distance processing
	init_distance_processing();
	shell_print(sh, "✓ Distance processing initialized");
	
	return 0;
}

// Shell command: radar measure [skip]
static int cmd_radar_measure(const struct shell *sh, size_t argc, char **argv)
{
	if (!radar_powered) {
		shell_error(sh, "Radar crystal is powered off! Run 'radar power on' first.");
		return -1;
	}
	
	if (!radar_initialized) {
		shell_error(sh, "Radar not initialized! Run 'radar init' first.");
		return -1;
	}
	
	// Parse optional skip parameter (default to current saved value)
	int skip = current_skip;  // Use saved skip value
	if (argc > 1) {
		skip = atoi(argv[1]);
		if (skip < 0 || skip > 100) {
			shell_error(sh, "Invalid skip value: %d (must be 0-100)", skip);
			return -1;
		}
		current_skip = skip;  // Save new skip value
		shell_print(sh, "Skip value updated to %d", current_skip);
	}
	
	shell_print(sh, "Triggering radar measurement (skip=%d)...", skip);
	shell_print(sh, "Buffer size: %d bytes", BUF_SIZE);
	
	// BGT60 driver only supports RTIO sensor_read API, not traditional sample_fetch
	shell_print(sh, "Using RTIO sensor_read API...");
	shell_print(sh, "Calling sensor_read with buffer %p, size %d", 
	           (void*)sensor_buffer, sizeof(sensor_buffer));
	
	int ret = sensor_read(&iodev, &ctx, sensor_buffer, sizeof(sensor_buffer));
	if (ret < 0) {
		shell_error(sh, "sensor_read() failed: %d", ret);
		return ret;
	}
	shell_print(sh, "sensor_read() successful: %d bytes", ret);
	
	shell_print(sh, "Successfully read %d bytes from radar", BUF_SIZE);
	
	// Process radar data for distance measurement
	shell_print(sh, "\n--- Distance Processing ---");
	float distance = process_radar_distance(sensor_buffer, BUF_SIZE, skip, false);
	
	if (distance >= 0.0f) {
		// Convert to millimeters and centimeters for integer display
		int distance_mm = (int)(distance * 1000);
		int distance_cm_int = (int)(distance * 1000);  // Convert to mm, then display as cm
		shell_print(sh, "✓ Distance detected: %d.%03d meters (%d.%d cm)", 
		           distance_mm / 1000, distance_mm % 1000, distance_cm_int / 10, distance_cm_int % 10);
	} else {
		shell_print(sh, "⚠ No clear distance signal detected");
	}
	
	// Check if data is repeating pattern (indicates dummy data)
	bool is_repeating = true;
	if (BUF_SIZE >= 6) {
		for (int i = 3; i < BUF_SIZE - 2; i += 3) {
			if (sensor_buffer[i] != sensor_buffer[0] || 
			    sensor_buffer[i+1] != sensor_buffer[1] || 
			    sensor_buffer[i+2] != sensor_buffer[2]) {
				is_repeating = false;
				break;
			}
		}
	}
	
	if (is_repeating) {
		shell_print(sh, "⚠️  WARNING: Data shows repeating pattern - likely dummy/default data!");
		shell_print(sh, "Pattern: %02X %02X %02X (repeating %d times)", 
		       sensor_buffer[0], sensor_buffer[1], sensor_buffer[2], BUF_SIZE/3);
	} else {
		shell_print(sh, "✓ Data shows variation - appears to be real radar measurements!");
	}
	
	// Print first 48 bytes as hex dump
	int bytes_to_print = (BUF_SIZE < 48) ? BUF_SIZE : 48;
	shell_print(sh, "First %d bytes:", bytes_to_print);
	for (int i = 0; i < bytes_to_print; i++) {
		shell_fprintf(sh, SHELL_NORMAL, "%02X ", sensor_buffer[i]);
		if ((i + 1) % 16 == 0) shell_print(sh, "");
	}
	if (bytes_to_print % 16 != 0) shell_print(sh, "");
	
	// Show first few decoded samples (matching Python bit manipulation)
	shell_print(sh, "\nFirst few 12-bit samples:");
	for (int i = 0; i < bytes_to_print - 2; i += 3) {
		uint16_t sample1 = (sensor_buffer[i] << 4) | (sensor_buffer[i+1] >> 4);
		uint16_t sample2 = ((sensor_buffer[i+1] & 0x0F) << 8) | sensor_buffer[i+2];
		shell_print(sh, "Sample %d: %d, Sample %d: %d", i/3*2, sample1, i/3*2+1, sample2);
		if (i >= 12) break;  // Show first few samples only
	}
	
	return 0;
}

// Shell command: radar status
static int cmd_radar_status(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "BGT60 Radar Status:");
	shell_print(sh, "  Initialized: %s", radar_initialized ? "Yes" : "No");
	shell_print(sh, "  Skip bins:   %d", current_skip);
	
	if (radar_initialized) {
		shell_print(sh, "  Device:      %s", bgt60_dev->name);
		shell_print(sh, "  Buffer size: %d bytes", BUF_SIZE);
		shell_print(sh, "  Blocks:      %d", CONFIG_BGT60_RADAR_SENSOR_DEFAULT_FIFO_BLOCKS);
		// Convert to micrometers for integer display
		int range_bin_um = (int)(range_bin_length * 1000000);
		shell_print(sh, "  Range/bin:   %d.%06d m", range_bin_um / 1000000, range_bin_um % 1000000);
	}
	
	return 0;
}

// Shell command: radar config skip [value]
static int cmd_radar_config_skip(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "Current skip value: %d bins", current_skip);
		return 0;
	}
	
	int new_skip = atoi(argv[1]);
	if (new_skip < 0 || new_skip > 100) {
		shell_error(sh, "Invalid skip value: %d (must be 0-100)", new_skip);
		return -1;
	}
	
	current_skip = new_skip;
	shell_print(sh, "Skip value set to %d bins", current_skip);
	return 0;
}

// Shell command: radar config gain [value_in_db]
static int cmd_radar_config_gain(const struct shell *sh, size_t argc, char **argv)
{
	// NOTE: This implementation requires adding attr_set/get to the BGT60 driver
	// For now, it shows what the implementation would look like
	
	struct sensor_value val;
	int ret;
	
	// Check if sensor is initialized
	if (!bgt60_dev) {
		shell_error(sh, "Sensor not initialized. Run 'radar init' first.");
		return -1;
	}
	
	if (argc < 2) {
		// Read current gain from CSP_D_2 register (0x0E)
		val.val1 = 0x0E;  // Register address
		val.val2 = 0;
		
		// Use SENSOR_ATTR_PRIV_START as custom attribute for register access
		ret = sensor_attr_get(bgt60_dev, SENSOR_CHAN_ALL, SENSOR_ATTR_PRIV_START, &val);
		if (ret) {
			shell_error(sh, "Failed to read register (driver may need attr_get implementation)");
			shell_print(sh, "To add register access to the driver, implement attr_set/get functions");
			return ret;
		}
		
		// Extract VGA_GAIN1 from bits 5:3
		int gain_setting = (val.val1 >> 3) & 0x07;
		int gain_db = gain_setting * 5;  // Each step is 5dB
		shell_print(sh, "Current gain: %d dB (setting %d, register 0x%06X)", 
		           gain_db, gain_setting, val.val1);
		return 0;
	}
	
	// Set new gain value
	int gain_db = atoi(argv[1]);
	int gain_setting;
	
	// Convert dB to setting value
	switch (gain_db) {
		case 0:  gain_setting = 0; break;
		case 5:  gain_setting = 1; break;
		case 10: gain_setting = 2; break;
		case 15: gain_setting = 3; break;
		case 20: gain_setting = 4; break;
		case 25: gain_setting = 5; break;
		case 30: gain_setting = 6; break;
		default:
			shell_error(sh, "Invalid gain: %d dB (valid: 0,5,10,15,20,25,30)", gain_db);
			return -1;
	}
	
	// Read current register value first
	val.val1 = 0x0E;  // CSP_D_2 register
	ret = sensor_attr_get(bgt60_dev, SENSOR_CHAN_ALL, SENSOR_ATTR_PRIV_START, &val);
	if (ret) {
		shell_error(sh, "Failed to read current register value");
		return ret;
	}
	
	// Modify only VGA_GAIN1 bits (5:3), preserve other bits
	uint32_t reg_val = val.val1;
	reg_val = (reg_val & ~(0x07 << 3)) | (gain_setting << 3);
	
	// Write back modified value
	val.val1 = 0x0E;  // Register address
	val.val2 = reg_val;  // New value
	ret = sensor_attr_set(bgt60_dev, SENSOR_CHAN_ALL, SENSOR_ATTR_PRIV_START, &val);
	if (ret) {
		shell_error(sh, "Failed to write gain setting (driver may need attr_set implementation)");
		return ret;
	}
	
	shell_print(sh, "Gain set to %d dB (setting %d)", gain_db, gain_setting);
	return 0;
}

// Shell command: radar config hpf [frequency_khz]
static int cmd_radar_config_hpf(const struct shell *sh, size_t argc, char **argv)
{
	struct sensor_value val;
	int ret;
	
	// Check if sensor is initialized
	if (!bgt60_dev) {
		shell_error(sh, "Sensor not initialized. Run 'radar init' first.");
		return -1;
	}
	
	if (argc < 2) {
		// Read current HPF setting from CSP_I_2 register (0x0A)
		val.val1 = 0x0A;  // Register address
		val.val2 = 0;
		
		ret = sensor_attr_get(bgt60_dev, SENSOR_CHAN_ALL, SENSOR_ATTR_PRIV_START, &val);
		if (ret) {
			shell_error(sh, "Failed to read register");
			return ret;
		}
		
		// Extract HPF_SEL1 from bits 2:0
		int hpf_setting = val.val1 & 0x07;
		int frequency_khz;
		
		switch (hpf_setting) {
			case 0: frequency_khz = 20; break;
			case 1: frequency_khz = 40; break;
			case 2: frequency_khz = 80; break;
			case 3: frequency_khz = 140; break;
			case 4: frequency_khz = 160; break;
			default: frequency_khz = -1; break;
		}
		
		if (frequency_khz > 0) {
			shell_print(sh, "Current HPF: %d kHz (setting %d, register 0x%06X)", 
			           frequency_khz, hpf_setting, val.val1);
		} else {
			shell_print(sh, "Current HPF: Invalid setting %d (register 0x%06X)", 
			           hpf_setting, val.val1);
		}
		return 0;
	}
	
	// Set new HPF frequency
	int frequency_khz = atoi(argv[1]);
	int hpf_setting;
	
	// Convert frequency to setting value
	switch (frequency_khz) {
		case 20:  hpf_setting = 0; break;
		case 40:  hpf_setting = 1; break;
		case 80:  hpf_setting = 2; break;
		case 140: hpf_setting = 3; break;
		case 160: hpf_setting = 4; break;
		default:
			shell_error(sh, "Invalid frequency: %d kHz (valid: 20,40,80,140,160)", frequency_khz);
			return -1;
	}
	
	// Read current register value first
	val.val1 = 0x0A;  // CSP_I_2 register
	ret = sensor_attr_get(bgt60_dev, SENSOR_CHAN_ALL, SENSOR_ATTR_PRIV_START, &val);
	if (ret) {
		shell_error(sh, "Failed to read current register value");
		return ret;
	}
	
	// Modify only HPF_SEL1 bits (2:0), preserve other bits
	uint32_t reg_val = val.val1;
	reg_val = (reg_val & ~0x07) | hpf_setting;
	
	// Write back modified value
	val.val1 = 0x0A;  // Register address
	val.val2 = reg_val;  // New value
	ret = sensor_attr_set(bgt60_dev, SENSOR_CHAN_ALL, SENSOR_ATTR_PRIV_START, &val);
	if (ret) {
		shell_error(sh, "Failed to write HPF setting");
		return ret;
	}
	
	shell_print(sh, "HPF set to %d kHz (setting %d)", frequency_khz, hpf_setting);
	return 0;
}

// Shell command: radar export raw
static int cmd_radar_export_raw(const struct shell *sh, size_t argc, char **argv)
{
	if (!data_available) {
		shell_error(sh, "No data available. Run 'radar measure' first.");
		return -1;
	}
	
	shell_print(sh, "RAW_START");
	shell_print(sh, "SAMPLES:%d", last_num_samples);
	
	for (int i = 0; i < last_num_samples; i++) {
		shell_print(sh, "%d", last_samples[i]);
	}
	
	shell_print(sh, "RAW_END");
	return 0;
}

// Shell command: radar export fft
static int cmd_radar_export_fft(const struct shell *sh, size_t argc, char **argv)
{
	if (!data_available) {
		shell_error(sh, "No data available. Run 'radar measure' first.");
		return -1;
	}
	
	shell_print(sh, "FFT_START");
	shell_print(sh, "SIZE:%d", FFT_SIZE);
	
	for (int i = 0; i < FFT_SIZE; i++) {
		// Convert to integer to avoid floating point issues
		int real_int = (int)(last_fft_data[i].real * 1000);
		int imag_int = (int)(last_fft_data[i].imag * 1000);
		shell_print(sh, "%d,%d", real_int, imag_int);
	}
	
	shell_print(sh, "FFT_END");
	return 0;
}

// Shell command: radar register read <address>
static int cmd_radar_register_read(const struct shell *sh, size_t argc, char **argv)
{
	if (!bgt60_dev) {
		shell_error(sh, "Sensor not initialized. Run 'radar init' first.");
		return -1;
	}
	
	if (argc < 2) {
		shell_error(sh, "Usage: radar register read <address>");
		shell_print(sh, "Example: radar register read 0x0E");
		return -1;
	}
	
	// Parse register address
	char *endptr;
	uint32_t addr = strtoul(argv[1], &endptr, 0);  // Support hex (0x) and decimal
	if (*endptr != '\0' || addr > 0x7F) {
		shell_error(sh, "Invalid address: %s (must be 0x00-0x7F)", argv[1]);
		return -1;
	}
	
	// Read register using new driver API
	struct sensor_value val;
	val.val1 = addr;
	int ret = sensor_attr_get(bgt60_dev, SENSOR_CHAN_ALL, SENSOR_ATTR_PRIV_START, &val);
	if (ret) {
		shell_error(sh, "Failed to read register 0x%02X: %d", addr, ret);
		return ret;
	}
	
	shell_print(sh, "Register 0x%02X = 0x%06X", addr, val.val1);
	return 0;
}

// Shell command: radar register write <address> <value>
static int cmd_radar_register_write(const struct shell *sh, size_t argc, char **argv)
{
	if (!bgt60_dev) {
		shell_error(sh, "Sensor not initialized. Run 'radar init' first.");
		return -1;
	}
	
	if (argc < 3) {
		shell_error(sh, "Usage: radar register write <address> <value>");
		shell_print(sh, "Example: radar register write 0x0E 0x123456");
		return -1;
	}
	
	// Parse register address
	char *endptr;
	uint32_t addr = strtoul(argv[1], &endptr, 0);
	if (*endptr != '\0' || addr > 0x7F) {
		shell_error(sh, "Invalid address: %s (must be 0x00-0x7F)", argv[1]);
		return -1;
	}
	
	// Parse value
	uint32_t value = strtoul(argv[2], &endptr, 0);
	if (*endptr != '\0' || value > 0xFFFFFF) {
		shell_error(sh, "Invalid value: %s (must be 0x000000-0xFFFFFF)", argv[2]);
		return -1;
	}
	
	// Write register using new driver API
	struct sensor_value val;
	val.val1 = addr;
	val.val2 = value;
	int ret = sensor_attr_set(bgt60_dev, SENSOR_CHAN_ALL, SENSOR_ATTR_PRIV_START, &val);
	if (ret) {
		shell_error(sh, "Failed to write register 0x%02X: %d", addr, ret);
		return ret;
	}
	
	shell_print(sh, "Register 0x%02X = 0x%06X (written)", addr, value);
	return 0;
}

// Shell command: radar register dump [start] [count]
static int cmd_radar_register_dump(const struct shell *sh, size_t argc, char **argv)
{
	if (!bgt60_dev) {
		shell_error(sh, "Sensor not initialized. Run 'radar init' first.");
		return -1;
	}
	
	// Parse start address (default 0x00)
	uint32_t start = 0x00;
	if (argc >= 2) {
		char *endptr;
		start = strtoul(argv[1], &endptr, 0);
		if (*endptr != '\0' || start > 0x7F) {
			shell_error(sh, "Invalid start address: %s (must be 0x00-0x7F)", argv[1]);
			return -1;
		}
	}
	
	// Parse count (default 16)
	uint32_t count = 16;
	if (argc >= 3) {
		char *endptr;
		count = strtoul(argv[2], &endptr, 0);
		if (*endptr != '\0' || count > 128 || (start + count) > 0x80) {
			shell_error(sh, "Invalid count: %s (start+count must be <= 0x80)", argv[2]);
			return -1;
		}
	}
	
	shell_print(sh, "Register dump from 0x%02X to 0x%02X:", start, start + count - 1);
	shell_print(sh, "Addr    Value     Description");
	shell_print(sh, "------- --------- -----------");
	
	for (uint32_t addr = start; addr < start + count; addr++) {
		struct sensor_value val;
		val.val1 = addr;
		int ret = sensor_attr_get(bgt60_dev, SENSOR_CHAN_ALL, SENSOR_ATTR_PRIV_START, &val);
		
		if (ret) {
			shell_print(sh, "0x%02X    ERROR-%d", addr, ret);
		} else {
			// Add register descriptions for known registers
			const char *desc = "";
			switch (addr) {
				case 0x00: desc = "MAIN"; break;
				case 0x01: desc = "STAT0"; break;
				case 0x02: desc = "STAT1"; break;
				case 0x03: desc = "FSTAT"; break;
				case 0x0E: desc = "CSP_D_2 (VGA gain)"; break;
				case 0x10: desc = "ADC0"; break;
				case 0x11: desc = "ADC1"; break;
				default:   desc = ""; break;
			}
			shell_print(sh, "0x%02X    0x%06X  %s", addr, val.val1, desc);
		}
	}
	
	return 0;
}

// Shell command: radar power on
static int cmd_radar_power_on(const struct shell *sh, size_t argc, char **argv)
{
	// Configure GPIO if not already done
	if (!gpio_is_ready_dt(&radar_crystal)) {
		shell_error(sh, "Radar crystal control GPIO not ready!");
		return -1;
	}
	
	int ret = gpio_pin_configure_dt(&radar_crystal, GPIO_OUTPUT);
	if (ret < 0) {
		shell_error(sh, "Failed to configure radar crystal GPIO: %d", ret);
		return ret;
	}
	
	// For ground-side MOSFET: LOW = ON (MOSFET conducts to ground)
	ret = gpio_pin_set_dt(&radar_crystal, 0);
	if (ret < 0) {
		shell_error(sh, "Failed to set radar crystal GPIO: %d", ret);
		return ret;
	}
	
	radar_powered = true;
	shell_print(sh, "✓ Radar crystal powered ON (P1.08 = LOW, MOSFET conducting)");
	
	// Wait a bit for crystal to stabilize
	k_sleep(K_MSEC(100));
	shell_print(sh, "✓ Crystal stabilization delay complete");
	
	return 0;
}

// Shell command: radar power off
static int cmd_radar_power_off(const struct shell *sh, size_t argc, char **argv)
{
	// Configure GPIO if not already done
	if (!gpio_is_ready_dt(&radar_crystal)) {
		shell_error(sh, "Radar crystal control GPIO not ready!");
		return -1;
	}
	
	int ret = gpio_pin_configure_dt(&radar_crystal, GPIO_OUTPUT);
	if (ret < 0) {
		shell_error(sh, "Failed to configure radar crystal GPIO: %d", ret);
		return ret;
	}
	
	// For ground-side MOSFET: HIGH = OFF (MOSFET blocks, no ground path)
	ret = gpio_pin_set_dt(&radar_crystal, 1);
	if (ret < 0) {
		shell_error(sh, "Failed to set radar crystal GPIO: %d", ret);
		return ret;
	}
	
	radar_powered = false;
	shell_print(sh, "✓ Radar crystal powered OFF (P1.08 = HIGH, MOSFET blocking)");
	
	// Mark radar as not initialized since we powered it off
	radar_initialized = false;
	
	return 0;
}

// Shell command: radar power status
static int cmd_radar_power_status(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "Radar Power Status:");
	shell_print(sh, "  Crystal power: %s", radar_powered ? "ON" : "OFF");
	shell_print(sh, "  Pin P1.08:     %s (MOSFET %s)", 
	           radar_powered ? "LOW" : "HIGH",
	           radar_powered ? "conducting" : "blocking");
	shell_print(sh, "  Initialized:   %s", radar_initialized ? "Yes" : "No");
	
	if (!radar_powered && radar_initialized) {
		shell_print(sh, "⚠ WARNING: Radar marked as initialized but power is OFF!");
	}
	
	return 0;
}

// Shell command: radar continuous [delay_ms] [count]
static int cmd_radar_continuous(const struct shell *sh, size_t argc, char **argv)
{
	if (!radar_initialized) {
		shell_error(sh, "Radar not initialized! Run 'radar init' first.");
		return -1;
	}
	
	int delay_ms = 1000;  // Default 1 second
	int max_count = 20;   // Default 20 measurements
	
	if (argc > 1) {
		delay_ms = atoi(argv[1]);
		if (delay_ms < 100) {
			delay_ms = 100;  // Minimum 100ms
		}
	}
	
	if (argc > 2) {
		max_count = atoi(argv[2]);
		if (max_count < 1) {
			max_count = 1;  // Minimum 1 measurement
		}
		if (max_count > 1000) {
			max_count = 1000;  // Maximum 1000 measurements
		}
	}
	
	shell_print(sh, "Taking %d measurements every %dms (skip=%d).", max_count, delay_ms, current_skip);
	
	int measurement_count = 0;
	while (measurement_count < max_count) {
		measurement_count++;
		
		// Read sensor data
		int ret = sensor_read(&iodev, &ctx, sensor_buffer, sizeof(sensor_buffer));
		if (ret < 0) {
			shell_error(sh, "sensor_read() failed: %d", ret);
			k_sleep(K_MSEC(delay_ms));
			continue;
		}
		
		// Process radar data for distance measurement (no debug output in continuous mode)
		float distance = process_radar_distance(sensor_buffer, BUF_SIZE, current_skip, true);
		
		if (distance >= 0.0f) {
			// Convert to millimeters and centimeters for integer display
			int distance_mm = (int)(distance * 1000);
			shell_print(sh, "%3d: Distance: %d.%03d m (%d.%d cm)", measurement_count,
			           distance_mm / 1000, distance_mm % 1000, distance_mm / 10, distance_mm % 10);
		} else {
			shell_print(sh, "%3d: No signal detected", measurement_count);
		}
		
		k_sleep(K_MSEC(delay_ms));
	}
	
	shell_print(sh, "Continuous measurements stopped.");
	return 0;
}


// Shell command subgroups
SHELL_STATIC_SUBCMD_SET_CREATE(sub_radar_power,
	SHELL_CMD(on, NULL, "Turn on radar crystal power", cmd_radar_power_on),
	SHELL_CMD(off, NULL, "Turn off radar crystal power", cmd_radar_power_off),
	SHELL_CMD(status, NULL, "Show power status", cmd_radar_power_status),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_radar_config,
	SHELL_CMD(skip, NULL, "Get/set skip bins [value]", cmd_radar_config_skip),
	SHELL_CMD(gain, NULL, "Get/set VGA gain [dB: 0,5,10,15,20,25,30]", cmd_radar_config_gain),
	SHELL_CMD(hpf, NULL, "Get/set high pass filter [kHz: 20,40,80,140,160]", cmd_radar_config_hpf),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_radar_export,
	SHELL_CMD(raw, NULL, "Export raw sample data", cmd_radar_export_raw),
	SHELL_CMD(fft, NULL, "Export FFT data", cmd_radar_export_fft),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_radar_register,
	SHELL_CMD(read, NULL, "Read register <address>", cmd_radar_register_read),
	SHELL_CMD(write, NULL, "Write register <address> <value>", cmd_radar_register_write),
	SHELL_CMD(dump, NULL, "Dump registers [start] [count]", cmd_radar_register_dump),
	SHELL_SUBCMD_SET_END
);

// Shell command group
SHELL_STATIC_SUBCMD_SET_CREATE(sub_radar,
	SHELL_CMD(init, NULL, "Initialize BGT60 radar", cmd_radar_init),
	SHELL_CMD(measure, NULL, "Take radar measurement [skip_bins]", cmd_radar_measure),
	SHELL_CMD(continuous, NULL, "Continuous measurements [delay_ms] [count]", cmd_radar_continuous),
	SHELL_CMD(status, NULL, "Show radar status", cmd_radar_status),
	SHELL_CMD(power, &sub_radar_power, "Power control commands", NULL),
	SHELL_CMD(config, &sub_radar_config, "Configuration commands", NULL),
	SHELL_CMD(export, &sub_radar_export, "Export data commands", NULL),
	SHELL_CMD(register, &sub_radar_register, "Register access commands", NULL),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(radar, &sub_radar, "BGT60 radar commands", NULL);

int main(void)
{
	// Initialize radar crystal power control - ensure it's OFF by default
	if (gpio_is_ready_dt(&radar_crystal)) {
		gpio_pin_configure_dt(&radar_crystal, GPIO_OUTPUT);
		gpio_pin_set_dt(&radar_crystal, 1);  // HIGH = OFF (MOSFET blocks)
		printk("✓ Radar crystal power initialized: OFF\n");
	} else {
		printk("⚠ Radar crystal GPIO not ready!\n");
	}
	
	printk("\n==========================================\n");
	printk("BGT60 Radar Shell Interface (Sensor API)\n");
	printk("==========================================\n");
	printk("Using BGT60 sensor driver with RTIO\n");
	printk("Commands:\n");
	printk("  radar init               - Initialize BGT60\n");
	printk("  radar measure [skip]     - Take measurement (skip=bins to skip, default %d)\n", current_skip);
	printk("  radar continuous [ms] [n] - Take n measurements (default 20 @ 1000ms)\n");
	printk("  radar status             - Show status\n");
	printk("  radar power on           - Turn on radar crystal (P1.08 HIGH, MOSFET on)\n");
	printk("  radar power off          - Turn off radar crystal (P1.08 LOW, MOSFET off)\n");
	printk("  radar power status       - Show power status\n");
	printk("  radar config skip        - Get/set skip bins\n");
	printk("  radar config gain        - Get/set VGA gain (0-30 dB)\n");
	printk("  radar config hpf         - Get/set high pass filter (20-160 kHz)\n");
	printk("  radar export raw         - Export raw sample data\n");
	printk("  radar export fft         - Export FFT data\n");
	printk("  radar register read      - Read register value\n");
	printk("  radar register write     - Write register value\n");
	printk("  radar register dump      - Dump multiple registers\n");
	printk("==========================================\n");
	
	return 0;
}