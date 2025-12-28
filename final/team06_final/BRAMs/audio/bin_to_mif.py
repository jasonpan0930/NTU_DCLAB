#!/usr/bin/env python3
"""
Convert binary files to MIF (Memory Initialization File) format for Quartus.
"""

import os
import sys

def bin_to_mif(bin_file, mif_file, width=8):
    """
    Convert a binary file to MIF format.
    
    Args:
        bin_file: Path to input binary file
        mif_file: Path to output MIF file
        width: Data width in bits (default: 8, can be 8 or 16)
    """
    # Read binary file
    with open(bin_file, 'rb') as f:
        data = f.read()
    
    # Write MIF file
    with open(mif_file, 'w') as f:
        if width == 8:
            # 8-bit mode: one byte per address
            depth = len(data)
            f.write(f"DEPTH = {depth};\n")
            f.write(f"WIDTH = {width};\n")
            f.write("ADDRESS_RADIX = HEX;\n")
            f.write("DATA_RADIX = HEX;\n")
            f.write("CONTENT\n")
            f.write("BEGIN\n")
            
            # Write data
            for addr, byte_val in enumerate(data):
                f.write(f"{addr:08X} : {byte_val:02X};\n")
        
        elif width == 16:
            # 16-bit mode: two bytes per address (little-endian)
            # If odd number of bytes, pad with 0
            byte_count = len(data)
            depth = (byte_count + 1) // 2  # Round up
            f.write(f"DEPTH = {depth};\n")
            f.write(f"WIDTH = {width};\n")
            f.write("ADDRESS_RADIX = HEX;\n")
            f.write("DATA_RADIX = HEX;\n")
            f.write("CONTENT\n")
            f.write("BEGIN\n")
            
            # Write data: combine two bytes into one 16-bit word
            for i in range(0, byte_count, 2):
                addr = i // 2
                if i + 1 < byte_count:
                    # Two bytes available
                    word = data[i] | (data[i + 1] << 8)
                else:
                    # Only one byte available (last byte if odd count)
                    word = data[i]
                f.write(f"{addr:08X} : {word:04X};\n")
        
        else:
            raise ValueError(f"Unsupported width: {width}. Only 8 or 16 supported.")
        
        f.write("END;\n")
    
    print(f"Converted {bin_file} ({len(data)} bytes) -> {mif_file} (WIDTH={width})")

if __name__ == "__main__":
    # Get script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Convert bgm.bin
    bgm_bin = os.path.join(script_dir, "bgm.bin")
    bgm_mif = os.path.join(script_dir, "bgm.mif")
    if os.path.exists(bgm_bin):
        bin_to_mif(bgm_bin, bgm_mif, width=16)
    else:
        print(f"Error: {bgm_bin} not found")
    
    # Convert killed.bin
    killed_bin = os.path.join(script_dir, "killed.bin")
    killed_mif = os.path.join(script_dir, "killed.mif")
    if os.path.exists(killed_bin):
        bin_to_mif(killed_bin, killed_mif, width=16)
    else:
        print(f"Error: {killed_bin} not found")

