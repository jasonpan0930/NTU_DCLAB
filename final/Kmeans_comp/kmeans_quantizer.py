"""
K-means Quantizer with Transparent Support (Folder Version)
--------------------------------------------------------------
功能：
1. 對 folderpath 中所有圖片產生各自的 index map
2. palette[0] = 白色 (255,255,255)
3. 透明像素 (A==0) → index=0
4. 非透明像素 → K-means（K-1 clusters）
5. palette_rest 不得含白色 → 自動改 254,254,254
6. palette 使用固定 RGB565（16-bit）
7. K 預設為 256（你可自行修改）
8. 輸出到 folderpath/output/
9. 額外輸出：
     - xxx.bin（Binary raw data）
     - xxx.mif（BRAM Memory Initialization File）
     - xxx_palette.mif（BRAM 用 16-bit palette）
"""

import sys
import os
import numpy as np
from PIL import Image
from sklearn.cluster import KMeans

# ----------------------------------------------------
K = 256
CHUNK_SIZE = 16
SUPPORTED = (".png", ".jpg", ".jpeg", ".bmp")
# ----------------------------------------------------


def rgb565(r, g, b):
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)


def write_intel_hex(filename, byte_list):
    with open(filename, "w") as f:
        addr = 0
        for i in range(0, len(byte_list), CHUNK_SIZE):
            chunk = byte_list[i:i + CHUNK_SIZE]
            count = len(chunk)

            addr_hi = (addr >> 8) & 0xFF
            addr_lo = addr & 0xFF
            record = [count, addr_hi, addr_lo, 0x00] + chunk
            checksum = (-sum(record)) & 0xFF

            line = ":" + "".join("{:02X}".format(x) for x in record) + "{:02X}".format(checksum)
            f.write(line + "\n")

            addr += count
        f.write(":00000001FF\n")


def write_bin(filename, byte_list):
    with open(filename, "wb") as f:
        f.write(bytearray(byte_list))


def write_mif(filename, byte_list):
    depth = len(byte_list)
    with open(filename, "w") as f:
        f.write(f"WIDTH=8;\n")
        f.write(f"DEPTH={depth};\n")
        f.write("ADDRESS_RADIX=HEX;\n")
        f.write("DATA_RADIX=HEX;\n\n")
        f.write("CONTENT BEGIN\n")

        for addr, data in enumerate(byte_list):
            f.write(f"{addr:06X} : {data:02X};\n")

        f.write("END;\n")


# ★★★ 新增：palette 專用的 16-bit MIF ★★★
def write_palette_mif(filename, palette_rgb565):
    depth = len(palette_rgb565)  # =256
    with open(filename, "w") as f:
        f.write("WIDTH=16;\n")
        f.write(f"DEPTH={depth};\n")
        f.write("ADDRESS_RADIX=HEX;\n")
        f.write("DATA_RADIX=HEX;\n\n")
        f.write("CONTENT BEGIN\n")

        for addr, data in enumerate(palette_rgb565):
            f.write(f"{addr:04X} : {data:04X};\n")

        f.write("END;\n")


# ----------------------------------------------------


def main():
    if len(sys.argv) < 2:
        print("Usage: python kmeans_quantizer_transparent.py <folderpath>")
        return

    folder = sys.argv[1]
    output_folder = os.path.join(folder, "output")
    os.makedirs(output_folder, exist_ok=True)

    files = [f for f in os.listdir(folder) if f.lower().endswith(SUPPORTED)]
    if not files:
        print("No image files found in folder.")
        return

    for idx_file, fname in enumerate(files):
        print(f"\nProcessing: {fname}")
        path = os.path.join(folder, fname)

        img = Image.open(path).convert("RGBA")
        w, h = img.size
        pixels = np.array(img).reshape(-1, 4)

        mask_trans = (pixels[:, 3] == 0)
        mask_nontrans = ~mask_trans

        opaque_pixels = pixels[mask_nontrans][:, :3].astype(np.int32)

        print("Running K-means...")
        km = KMeans(n_clusters=K - 1, random_state=0).fit(opaque_pixels)
        centers = km.cluster_centers_

        palette = [(255, 255, 255)]  # index 0

        for (r, g, b) in centers:
            R = int(r)
            G = int(g)
            B = int(b)
            if R == 255 and G == 255 and B == 255:
                R = G = B = 254
            palette.append((R, G, B))

        palette_np_uint8 = np.array(palette, dtype=np.uint8)
        palette_np = np.array(palette, dtype=np.int32)

        labels = np.zeros(len(pixels), dtype=np.uint8)
        labels[mask_trans] = 0

        opaque = pixels[mask_nontrans][:, :3].astype(np.int32)

        diff = opaque[:, None, :] - palette_np[1:, :]
        dist = np.sum(diff * diff, axis=2)
        nearest = np.argmin(dist, axis=1) + 1

        labels[mask_nontrans] = nearest

        # ------------------------------
        # Output: Intel HEX
        # ------------------------------
        out_hex = os.path.join(output_folder, f"{os.path.splitext(fname)[0]}.HEX")
        write_intel_hex(out_hex, labels.tolist())

        # ------------------------------
        # Output: Binary
        # ------------------------------
        out_bin = os.path.join(output_folder, f"{os.path.splitext(fname)[0]}.bin")
        write_bin(out_bin, labels.tolist())

        # ------------------------------
        # Output: MIF (index data)
        # ------------------------------
        out_mif = os.path.join(output_folder, f"{os.path.splitext(fname)[0]}.mif")
        write_mif(out_mif, labels.tolist())

        # ------------------------------
        # Output: Palette (HEX)
        # ------------------------------
        palette_path = os.path.join(output_folder, f"{os.path.splitext(fname)[0]}_palette.HEX")
        palette_rgb565 = []
        with open(palette_path, "w") as f:
            for (r, g, b) in palette:
                val = rgb565(r, g, b)
                palette_rgb565.append(val)
                f.write(f"{val:04X}\n")

        # ------------------------------
        # ★★★ Output: Palette MIF ★★★
        # ------------------------------
        palette_mif_path = os.path.join(output_folder, f"{os.path.splitext(fname)[0]}_palette.mif")
        write_palette_mif(palette_mif_path, palette_rgb565)

        # ------------------------------
        # Preview image
        # ------------------------------
        rebuilt = palette_np_uint8[labels].reshape(h, w, 3)
        out_preview = os.path.join(output_folder, f"{os.path.splitext(fname)[0]}_preview.png")
        Image.fromarray(rebuilt).save(out_preview)

        print(f"Generated:")
        print(f"  {out_hex}")
        print(f"  {out_bin}")
        print(f"  {out_mif}")
        print(f"  {palette_path}")
        print(f"  {palette_mif_path}")
        print(f"  {out_preview}")

    print("\nAll done!")


if __name__ == "__main__":
    main()
