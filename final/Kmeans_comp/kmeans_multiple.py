"""
build_palette.py
多張圖片共用 RGB565 調色盤（資料夾輸入版）
------------------------------------------------------------
使用方式：
    python build_palette.py folderpath

輸入：
    folderpath/ 裡所有 PNG / JPG / BMP 會被讀取

輸出：
    folderpath/output/palette.HEX
    folderpath/output/image_0.HEX
    folderpath/output/image_1.HEX
    folderpath/output/preview_0.png
    folderpath/output/preview_1.png

規則：
1. palette[0] = 白色，用於透明
2. A==0 → index=0
3. 非透明像素 → K-means（K-1 clusters）
4. palette_rest 不得包含白色 → 強制改成 254,254,254
5. palette 使用 RGB565，固定 16-bit
"""

import sys
import os
import numpy as np
from PIL import Image
from sklearn.cluster import KMeans

# ---------------------------------------------------------
# 你可以自行修改的參數
# ---------------------------------------------------------
K = 256        # palette 總色數（含 index=0 的白色），剩下 K-1 給 K-means
CHUNK_SIZE = 16  # Intel HEX 每行 bytes
# ---------------------------------------------------------


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
            rectype = 0x00

            record = [count, addr_hi, addr_lo, rectype] + chunk
            checksum = (-sum(record)) & 0xFF

            hexline = ":" + "".join("{:02X}".format(x) for x in record) + "{:02X}".format(checksum)
            f.write(hexline + "\n")

            addr += count

        f.write(":00000001FF\n")  # EOF


def main():
    if len(sys.argv) < 2:
        print("Usage: python build_palette.py folderpath")
        return

    folder = sys.argv[1]
    output_folder = os.path.join(folder, "output")

    # 建立 output 目錄
    os.makedirs(output_folder, exist_ok=True)

    # 檔案清單
    supported = (".png", ".jpg", ".jpeg", ".bmp")
    image_files = [f for f in os.listdir(folder) if f.lower().endswith(supported)]

    if not image_files:
        print("No images found in folder.")
        return

    # ---------------------------------------------------------
    # 1. 讀取所有圖片
    # ---------------------------------------------------------
    print("Loading images...")

    all_pixels = []
    images_rgba = []
    sizes = []

    for fname in image_files:
        path = os.path.join(folder, fname)

        img = Image.open(path).convert("RGBA")
        w, h = img.size
        sizes.append((w, h))

        arr = np.array(img)
        images_rgba.append(arr)

        flat = arr.reshape(-1, 4)
        opaque = flat[flat[:, 3] != 0]
        rgb_opaque = opaque[:, :3]
        all_pixels.append(rgb_opaque)

    all_pixels = np.concatenate(all_pixels, axis=0)

    # ---------------------------------------------------------
    # 2. K-means
    # ---------------------------------------------------------
    print("Running K-means on combined images...")

    km = KMeans(n_clusters=K - 1, random_state=0).fit(all_pixels)
    centers = km.cluster_centers_

    # ---------------------------------------------------------
    # 3. palette
    # ---------------------------------------------------------
    palette = [(255, 255, 255)]  # index 0 = 白色透明鍵

    for (r, g, b) in centers:
        R = int(r)
        G = int(g)
        B = int(b)

        if R == 255 and G == 255 and B == 255:
            R, G, B = 254, 254, 254

        palette.append((R, G, B))

    palette_np = np.array(palette, dtype=np.uint8)

    # ---------------------------------------------------------
    # 4. 分別建立每張圖的 index map
    # ---------------------------------------------------------
    print("Encoding individual images...")

    for i, arr in enumerate(images_rgba):
        w, h = sizes[i]
        flat = arr.reshape(-1, 4)

        index_map = np.zeros(len(flat), dtype=np.uint8)

        trans_mask = (flat[:, 3] == 0)
        index_map[trans_mask] = 0

        opaque_mask = ~trans_mask
        opaque_rgb = flat[opaque_mask][:, :3]

        diff = opaque_rgb[:, None, :] - palette_np[1:, :]
        dist = np.sum(diff * diff, axis=2)
        nearest = np.argmin(dist, axis=1) + 1

        index_map[opaque_mask] = nearest

        # 輸出 image_N.HEX
        outfile = os.path.join(output_folder, f"image_{i}.HEX")
        write_intel_hex(outfile, index_map.tolist())

        # 輸出 preview
        rebuilt = palette_np[index_map].reshape(h, w, 3)
        Image.fromarray(rebuilt).save(os.path.join(output_folder, f"preview_{i}.png"))

    # ---------------------------------------------------------
    # 5. 輸出 palette.HEX
    # ---------------------------------------------------------
    palette_path = os.path.join(output_folder, "palette.HEX")

    with open(palette_path, "w") as f:
        for (r, g, b) in palette:
            f.write("{:04X}\n".format(rgb565(r, g, b)))

    print("Done!")
    print(f"Output written to: {output_folder}")


if __name__ == "__main__":
    main()
