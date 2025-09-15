import serial
import time
from collections import Counter

def is_likely_protocol(data):
    """
    Analyzes a byte array to determine if it looks like structured protocol data.
    Returns True if it finds signs of repetition and structure, False if it looks random.
    """
    if len(data) < 20:
        return False  # Not enough data to analyze

    # Check 1: Check for frequent repeating patterns (e.g., packet headers/command bytes)
    # Count the frequency of each byte value
    byte_counts = Counter(data)
    most_common_byte, most_common_count = byte_counts.most_common(1)[0]
    
    # If a single byte value appears too frequently (e.g., >30%), it's likely filler (0x00, 0xFF) or an error.
    if most_common_count / len(data) > 0.3:
        return False

    # Check 2: Check for predictable sequences (e.g., incrementing counters, stable values)
    # Look for a high number of consecutive byte pairs that are identical (suggests multi-byte values changing slowly)
    identical_pair_count = 0
    for i in range(len(data) - 1):
        if data[i] == data[i+1]:
            identical_pair_count += 1
    # If many consecutive bytes are the same, it's a good sign (e.g., RPM value bytes)
    if identical_pair_count / len(data) > 0.05: # More than 5% of bytes are part of a repeated pair
        return True

    # Check 3: Simple entropy check - structured data often has lower entropy than random data
    # For our purposes, if it passed the pair check, it's promising.
    return False

def find_baudrate(port):
    """
    Attempts to find the correct baud rate for a given serial port.
    Connects at each baud rate, reads data, and checks if it looks valid.
    """
    # Common baud rates to try. Focus on higher ones common in modern devices.
    common_baudrates = [115200, 57600, 38400, 19200, 9600, 230400, 460800, 250000]
    
    print(f"Attempting to detect baud rate on {port}...")
    print("Ensure the  are powered ON and connected.")
    print("-" * 50)
    
    found_valid_baud = None
    
    for baud in common_baudrates:
        try:
            # Configure the serial port. Critical settings:
            # - bytesize=8, parity='N', stopbits=1 are the most common defaults.
            # - timeout=1: wait for 1 second for read to complete
            # - xonxoff=False, rtscts=False: disable software/hardware flow control
            with serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
                xonxoff=False,
                rtscts=False
            ) as ser:
                # Flush any old data from the buffers
                ser.reset_input_buffer()
                # Give the system a moment to start receiving data
                time.sleep(0.1)
                # Read a substantial chunk of data to analyze
                data = ser.read(500)  # Read up to 500 bytes
                
                if len(data) > 0:
                    print(f"Trying {baud:>7} baud... Received {len(data):>3} bytes.", end=' ')
                    # Analyze the data to see if it looks like a real protocol
                    if is_likely_protocol(data):
                        print("-> SUCCESS: Pattern detected! <-")
                        found_valid_baud = baud
                        # Print a hex preview of the first 50 bytes
                        print(f"      Data preview: {data[:50].hex(' ')}")
                        print("-" * 50)
                        # Don't break, let's see if other baud rates also "work"
                    else:
                        print("-> Junk data")
                else:
                    print(f"Trying {baud:>7} baud... No data received.")
                    
        except serial.SerialException as e:
            print(f"Trying {baud:>7} baud... Port error: {e}")
        except PermissionError:
            print(f"Trying {baud:>7} baud... Permission denied. Is the port free?")
        except Exception as e:
            print(f"Trying {baud:>7} baud... Unexpected error: {e}")
    
    # Summary
    print("\n" + "="*50)
    if found_valid_baud:
        print(f"[+] Likely baud rate found: {found_valid_baud}")
        print("[+] Next step: Use this baud rate to capture and analyze data.")
    else:
        print("[-] No likely baud rate found.")
        print("[-] Troubleshooting:")
        print("    - Check your wiring (RX -> Yellow, GND -> Brown).")
        print("    - Is the powered on and communicating?")
        print("    - Try a different USB-TTL adapter.")
        print("    - The protocol might use a less common baud rate.")

if __name__ == "__main__":
    # ****************** IMPORTANT CONFIGURATION ******************
    # Replace 'COM5' with your actual serial port!
    # On Windows: It's usually 'COM3', 'COM4', etc. Check Device Manager.
    # On Linux/macOS: It's usually '/dev/ttyUSB0', '/dev/tty.usbserial-*', etc.
    YOUR_SERIAL_PORT = 'COM5'
    # ****************************************************************
    
    find_baudrate(YOUR_SERIAL_PORT)