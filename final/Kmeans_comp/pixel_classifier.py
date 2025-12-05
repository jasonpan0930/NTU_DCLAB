"""
Pixel Classifier
Reads PNG images and classifies each pixel:
- Transparent (alpha == 0) → "00"
- Closer to black → "11"
- Closer to red → "10"
Outputs to folder_path/output/
"""

import sys
import os
import numpy as np
from PIL import Image

# ----------------------------------------------------
SUPPORTED = (".png", ".jpg", ".jpeg", ".bmp")
# ----------------------------------------------------


def classify_pixel(r, g, b, a):
    """
    Classify a single pixel based on transparency and color distance.
    
    Args:
        r, g, b, a: Red, green, blue, alpha values (0-255)
    
    Returns:
        Binary value: 0 (00) if transparent, 3 (11) if closer to black, 2 (10) if closer to red
    """
    # Check transparency first
    if a == 0:
        return 0  # "00" in binary
    
    # Convert to int to avoid uint8 overflow issues
    r_int = int(r)
    
    # Calculate distance to black (0, 0, 0)
    dist_to_black = r_int
    
    # Calculate distance to red (255, 0, 0)
    dist_to_red = abs(r_int - 255)
    
    # Return code based on which is closer
    if dist_to_black < dist_to_red:
        return 3  # "11" in binary
    else:
        return 2  # "10" in binary


def write_mif(filename, data_list):
    """
    Write data to a MIF (Memory Initialization File) format.
    
    Args:
        filename: Output MIF file path
        data_list: List of values (0, 2, or 3) to write
    """
    depth = len(data_list)
    with open(filename, "w") as f:
        f.write("WIDTH=2;\n")
        f.write(f"DEPTH={depth};\n")
        f.write("ADDRESS_RADIX=HEX;\n")
        f.write("DATA_RADIX=HEX;\n\n")
        f.write("CONTENT BEGIN\n")
        
        for addr, data in enumerate(data_list):
            f.write(f"{addr:06X} : {data:01X};\n")
        
        f.write("END;\n")


def read_mif(filename):
    """
    Read data from a MIF (Memory Initialization File) format.
    
    Args:
        filename: Input MIF file path
    
    Returns:
        List of byte values (0-255) read from the file
    """
    data_list = []
    with open(filename, "r") as f:
        in_content = False
        for line in f:
            line = line.strip()
            if line == "CONTENT BEGIN":
                in_content = True
                continue
            if line == "END;":
                break
            if in_content and ":" in line:
                # Parse line like "000000 : 03;"
                parts = line.split(":")
                if len(parts) == 2:
                    data_hex = parts[1].strip().rstrip(";")
                    data_value = int(data_hex, 16)
                    data_list.append(data_value)
    
    return data_list


def rebuild_image_from_mif(mif_path, width, height, output_path=None):
    """
    Rebuild an image from a MIF file.
    
    Args:
        mif_path: Path to MIF file
        width: Image width
        height: Image height
        output_path: Optional path to save rebuilt image. If None, auto-generates.
    
    Returns:
        PIL Image object
    """
    pixel_codes = read_mif(mif_path)
    
    if len(pixel_codes) != width * height:
        raise ValueError(f"MIF file has {len(pixel_codes)} pixels, but expected {width * height} (width={width}, height={height})")
    
    # Create RGBA image array
    image_array = np.zeros((height, width, 4), dtype=np.uint8)
    
    # Map codes to colors
    # 0 (00) = yellow (255, 255, 0, 255) - transparent pixels shown as yellow
    # 2 (10) = red (255, 0, 0, 255)
    # 3 (11) = black (0, 0, 0, 255)
    
    idx = 0
    for y in range(height):
        for x in range(width):
            code = pixel_codes[idx]
            if code == 0:  # Transparent -> Yellow
                image_array[y, x] = [255, 255, 0, 255]
            elif code == 2:  # Red
                image_array[y, x] = [255, 0, 0, 255]
            elif code == 3:  # Black
                image_array[y, x] = [0, 0, 0, 255]
            else:
                # Unknown code, use yellow
                image_array[y, x] = [255, 255, 0, 255]
            idx += 1
    
    # Create PIL image
    img = Image.fromarray(image_array, 'RGBA')
    
    # Save if output path provided
    if output_path:
        img.save(output_path)
    
    return img


def process_image(input_path, output_folder):
    """
    Process a PNG image and classify each pixel.
    
    Args:
        input_path: Path to input image file
        output_folder: Folder to save output files
    
    Returns:
        Tuple of (mif_path, rebuild_path) for the generated files
    """
    # Load image with alpha channel
    img = Image.open(input_path).convert("RGBA")
    width, height = img.size
    
    # Convert to numpy array
    pixels = np.array(img)
    
    # Process each pixel (row by row, left to right)
    pixel_codes = []
    for y in range(height):
        for x in range(width):
            r, g, b, a = pixels[y, x]
            code = classify_pixel(r, g, b, a)
            pixel_codes.append(code)
    
    # Generate output filenames
    base_name = os.path.splitext(os.path.basename(input_path))[0]
    mif_path = os.path.join(output_folder, f"{base_name}.mif")
    rebuild_path = os.path.join(output_folder, f"{base_name}_rebuild.png")
    
    # Write MIF file
    write_mif(mif_path, pixel_codes)
    
    # Rebuild image from MIF for verification
    rebuild_image_from_mif(mif_path, width, height, rebuild_path)
    
    # Print statistics
    count_00 = pixel_codes.count(0)
    count_10 = pixel_codes.count(2)
    count_11 = pixel_codes.count(3)
    total = len(pixel_codes)
    
    print(f"  Generated: {mif_path}")
    print(f"  Generated: {rebuild_path}")
    print(f"  Statistics: Transparent={count_00} ({100*count_00/total:.1f}%), "
          f"Red={count_10} ({100*count_10/total:.1f}%), "
          f"Black={count_11} ({100*count_11/total:.1f}%)")
    
    return mif_path, rebuild_path


def main():
    """
    Main function to process all images in a folder.
    """
    if len(sys.argv) < 2:
        print("Usage: python pixel_classifier.py <folderpath>")
        print("  Processes all images in folderpath and outputs to folderpath/output/")
        return
    
    folder = sys.argv[1]
    output_folder = os.path.join(folder, "output")
    os.makedirs(output_folder, exist_ok=True)
    
    # Find all supported image files
    files = [f for f in os.listdir(folder) if f.lower().endswith(SUPPORTED)]
    if not files:
        print("No image files found in folder.")
        return
    
    print(f"Processing {len(files)} image(s) in {folder}...")
    print(f"Output folder: {output_folder}\n")
    
    for fname in files:
        print(f"Processing: {fname}")
        path = os.path.join(folder, fname)
        
        try:
            process_image(path, output_folder)
        except Exception as e:
            print(f"  Error processing {fname}: {e}")
            continue
        print()
    
    print("All done!")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

