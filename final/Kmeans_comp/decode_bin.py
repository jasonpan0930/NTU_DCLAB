"""
Decode paletted image from binary file
- Reads palette from HEX file (RGB565 format)
- Reads pixel indices from binary file (16-bit per pixel)
- Converts RGB565 to RGB888
- Generates output image
"""

"""
# Default (auto-detects format)
python decode_bin.py

# Specify files and dimensions
python decode_bin.py <bin_file> <palette_file> <output_file> [width] [height]

# Force 16-bit mode (if needed)
python decode_bin.py <bin_file> <palette_file> <output_file> --16bit [width] [height]
"""

import struct
import numpy as np
from PIL import Image


def rgb565_to_rgb888(rgb565_value):
    """Convert RGB565 (16-bit) to RGB888 (24-bit)"""
    # Extract color components from RGB565
    # Format: RRRRR GGGGGG BBBBB (5-6-5 bits)
    r = ((rgb565_value >> 11) & 0x1F) << 3  # 5 bits for red
    g = ((rgb565_value >> 5) & 0x3F) << 2   # 6 bits for green
    b = (rgb565_value & 0x1F) << 3          # 5 bits for blue
    
    # Scale to full 8-bit range
    r = (r * 255) // 248  # Scale from 0-248 to 0-255
    g = (g * 255) // 252  # Scale from 0-252 to 0-255
    b = (b * 255) // 248  # Scale from 0-248 to 0-255
    
    return (r, g, b)


def load_palette(palette_file):
    """Load palette from HEX file (RGB565 format)"""
    palette = []
    with open(palette_file, 'r') as f:
        for line in f:
            line = line.strip()
            if line:
                # Read RGB565 value as hex
                rgb565 = int(line, 16)
                # Convert to RGB888
                rgb = rgb565_to_rgb888(rgb565)
                palette.append(rgb)
    return palette


def decode_image(bin_file, palette_file, output_file, width=None, height=None, force_16bit=False):
    """Decode paletted image from binary file
    
    Args:
        bin_file: Path to binary file containing pixel indices
        palette_file: Path to palette HEX file (RGB565 format)
        output_file: Path to save decoded image
        width: Image width (optional, will be inferred if not provided)
        height: Image height (optional, will be inferred if not provided)
        force_16bit: Force 16-bit pixel format (default: False, auto-detect)
    """
    
    # Load palette
    print(f"Loading palette from {palette_file}...")
    palette = load_palette(palette_file)
    print(f"Loaded {len(palette)} colors in palette")
    
    # Read binary file
    print(f"Reading binary file {bin_file}...")
    with open(bin_file, 'rb') as f:
        binary_data = f.read()
    
    file_size = len(binary_data)
    print(f"Binary file size: {file_size} bytes")
    
    # Determine pixel size
    if force_16bit:
        pixel_size = 2
        print("Using 16-bit indices (2 bytes per pixel) - forced")
    else:
        # Auto-detect based on file size and dimensions
        if width is not None and height is not None:
            expected_pixels = width * height
            if file_size == expected_pixels:
                pixel_size = 1
                print("Using 8-bit indices (1 byte per pixel)")
            elif file_size == expected_pixels * 2:
                pixel_size = 2
                print("Using 16-bit indices (2 bytes per pixel)")
            else:
                # Default to 16-bit as user specified
                pixel_size = 2
                print(f"Warning: File size doesn't match expected size. Using 16-bit format.")
        else:
            # Try to infer from file size
            # If each pixel is 16 bits (2 bytes), then:
            num_pixels_16bit = file_size // 2
            # Check if it matches common dimensions
            if file_size == 1280 * 720:
                width, height = 1280, 720
                print("Assuming 8-bit indices (1 byte per pixel)")
                pixel_size = 1
            else:
                # Default to 16-bit as user specified
                pixel_size = 2
                print(f"Assuming 16-bit indices (2 bytes per pixel)")
                if width is None and height is None:
                    # Try to infer dimensions from 16-bit assumption
                    # Common aspect ratios
                    if num_pixels_16bit == 640 * 360:
                        width, height = 640, 360
                    elif num_pixels_16bit == 1280 * 720:
                        width, height = 1280, 720
                    else:
                        # Default to 1280x720 and adjust
                        width, height = 1280, 720
                        print(f"Warning: Could not determine dimensions. Using default {width}x{height}")
                        print(f"Note: File has {num_pixels_16bit} pixels (16-bit assumption)")
    
    # Set default dimensions if not provided
    if width is None:
        width = 1280
    if height is None:
        height = 720
    
    # Read pixel indices
    pixels = []
    if pixel_size == 1:
        # 8-bit indices
        for i in range(0, file_size):
            if i < len(binary_data):
                index = binary_data[i]
                pixels.append(index)
    else:
        # 16-bit indices (little-endian)
        for i in range(0, file_size, 2):
            if i + 1 < len(binary_data):
                # Read as little-endian 16-bit unsigned integer
                index = struct.unpack('<H', binary_data[i:i+2])[0]
                pixels.append(index)
    
    num_pixels = len(pixels)
    print(f"Read {num_pixels} pixel indices")
    
    # Verify dimensions
    if width * height != num_pixels:
        print(f"Warning: Dimensions {width}x{height} = {width*height} don't match pixel count {num_pixels}")
        # Try to recalculate
        height = num_pixels // width
        if width * height != num_pixels:
            print(f"Recalculated height: {height}, total pixels: {width * height}")
    
    # Convert indices to RGB colors using palette
    print("Converting indices to RGB colors...")
    rgb_pixels = []
    out_of_range_count = 0
    for idx in pixels:
        if idx < len(palette):
            rgb_pixels.append(palette[idx])
        else:
            # Out of range index, use black
            out_of_range_count += 1
            if out_of_range_count <= 10:  # Only show first 10 warnings
                print(f"Warning: Index {idx} out of palette range (max: {len(palette)-1}), using black")
            rgb_pixels.append((0, 0, 0))
    
    if out_of_range_count > 10:
        print(f"Warning: {out_of_range_count - 10} more indices out of range (total: {out_of_range_count})")
    if out_of_range_count > 0:
        print(f"Note: This might indicate incorrect pixel format (8-bit vs 16-bit)")
    
    # Reshape to image dimensions
    print(f"Reshaping to {width}x{height}...")
    image_array = np.array(rgb_pixels, dtype=np.uint8).reshape(height, width, 3)
    
    # Create and save image
    print(f"Saving decoded image to {output_file}...")
    image = Image.fromarray(image_array, 'RGB')
    image.save(output_file)
    print(f"Successfully saved decoded image to {output_file}")


if __name__ == "__main__":
    import sys
    
    # Default file paths
    bin_file = "background/output/background.bin"
    palette_file = "background/output/background_palette.HEX"
    output_file = "background/output/background_decoded.png"
    
    # Allow command line arguments
    force_16bit = False
    width = None
    height = None
    
    if len(sys.argv) >= 2:
        bin_file = sys.argv[1]
    if len(sys.argv) >= 3:
        palette_file = sys.argv[2]
    if len(sys.argv) >= 4:
        output_file = sys.argv[3]
    if len(sys.argv) >= 5:
        if sys.argv[4].lower() == '--16bit':
            force_16bit = True
            if len(sys.argv) >= 6:
                width = int(sys.argv[5])
            if len(sys.argv) >= 7:
                height = int(sys.argv[6])
        else:
            width = int(sys.argv[4])
            if len(sys.argv) >= 6:
                height = int(sys.argv[5])
    
    # Check for --16bit flag in any position
    if '--16bit' in sys.argv:
        force_16bit = True
    
    decode_image(bin_file, palette_file, output_file, width, height, force_16bit)

