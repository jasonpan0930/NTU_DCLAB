import sys
import os
import numpy as np
from PIL import Image

# ----------------------------------------------------
# 設定
SUPPORTED = (".png", ".jpg", ".jpeg", ".bmp")
# ----------------------------------------------------

def load_palette_as_rgb888(palette_path):
    """讀取調色盤並確保轉換回 24-bit RGB 空間進行精確比對"""
    palette_rgb = []
    with open(palette_path, "r") as f:
        for line in f:
            hex_val = line.strip()
            if not hex_val: continue
            # 支援 RGB565 HEX (4位) 或 RGB888 HEX (6位)
            if len(hex_val) == 4:
                val = int(hex_val, 16)
                r = ((val >> 11) & 0x1F) << 3
                g = ((val >> 5) & 0x3F) << 2
                b = (val & 0x1F) << 3
                palette_rgb.append((r, g, b))
            else:
                r = int(hex_val[0:2], 16)
                g = int(hex_val[2:4], 16)
                b = int(hex_val[4:6], 16)
                palette_rgb.append((r, g, b))
    return np.array(palette_rgb, dtype=np.int32)

def write_bin(filename, byte_list):
    """輸出原始二進位檔案 (Raw Binary)"""
    with open(filename, "wb") as f:
        f.write(bytearray(byte_list))

def write_mif(filename, byte_list):
    """輸出 BRAM 用的 MIF 檔案"""
    depth = len(byte_list)
    with open(filename, "w") as f:
        f.write("WIDTH=8;\n")
        f.write(f"DEPTH={depth};\n")
        f.write("ADDRESS_RADIX=HEX;\n")
        f.write("DATA_RADIX=HEX;\n\n")
        f.write("CONTENT BEGIN\n")
        for addr, data in enumerate(byte_list):
            f.write(f"{addr:06X} : {data:02X};\n")
        f.write("END;\n")

def main():
    if len(sys.argv) < 3:
        print("Usage: python sync_map_to_palette.py <palette_file> <folderpath>")
        return

    palette_file = sys.argv[1]
    folder = sys.argv[2]
    output_folder = os.path.join(folder, "sync_output")
    os.makedirs(output_folder, exist_ok=True)

    # 載入調色盤
    palette_np = load_palette_as_rgb888(palette_file)
    print(f"Loaded palette with {len(palette_np)} colors.")

    files = [f for f in os.listdir(folder) if f.lower().endswith(SUPPORTED)]
    
    for fname in files:
        print(f"Processing: {fname}...")
        path = os.path.join(folder, fname)
        img = Image.open(path).convert("RGBA")
        w, h = img.size
        pixels = np.array(img).reshape(-1, 4)

        labels = np.zeros(len(pixels), dtype=np.uint8)
        mask_trans = (pixels[:, 3] == 0)
        mask_nontrans = ~mask_trans

        # 映射邏輯同步：透明=0, 非透明=調色盤[1:]最近色 + 1
        if np.any(mask_nontrans):
            opaque = pixels[mask_nontrans][:, :3].astype(np.int32)
            # 與調色盤 index 1~255 比對
            diff = opaque[:, None, :] - palette_np[1:, :]
            dist = np.sum(diff**2, axis=2)
            nearest = np.argmin(dist, axis=1) + 1
            labels[mask_nontrans] = nearest

        # 檔名處理
        base_name = os.path.splitext(fname)[0]

        # 1. 輸出 MIF
        out_mif = os.path.join(output_folder, f"{base_name}.mif")
        write_mif(out_mif, labels.tolist())

        # 2. 輸出 BIN (新增)
        out_bin = os.path.join(output_folder, f"{base_name}.bin")
        write_bin(out_bin, labels.tolist())
        
        # 3. 輸出預覽圖
        rebuilt = palette_np[labels].astype(np.uint8).reshape(h, w, 3)
        out_preview = os.path.join(output_folder, f"{base_name}_preview.png")
        Image.fromarray(rebuilt).save(out_preview)

    print(f"\nAll done! Output files (.mif, .bin, .png) are in: {output_folder}")

if __name__ == "__main__":
    main()
