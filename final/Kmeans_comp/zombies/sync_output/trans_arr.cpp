#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <string>

// --- 設定參數 ---
const int WIDTH = 90;   // 對應 SystemVerilog 的 ZOMBIE_SIZE_X (也是 MIF 的 DEPTH)
const int HEIGHT = 132;  // 對應 SystemVerilog 的 ZOMBIE_SIZE_Y
const int DATA_WIDTH = 16; // [15:8]=Max, [7:0]=Min

const int TOTAL_SIZE = WIDTH * HEIGHT;

// 定義透明色 (0x00)
const unsigned char TRANSPARENT_PIXEL = 0x00;

int main() {
    const std::string input_filename = "zombie_0.9x.bin";
    const std::string output_filename = "zombie_0.9x_bounds.mif";

    // 1. 讀取 Binary 檔案
    std::ifstream infile(input_filename, std::ios::binary);
    if (!infile) {
        std::cerr << "Error: Cannot open input file " << input_filename << std::endl;
        return 1;
    }

    std::vector<unsigned char> pixel_data(TOTAL_SIZE);
    infile.read(reinterpret_cast<char*>(pixel_data.data()), TOTAL_SIZE);
    if (infile.gcount() != TOTAL_SIZE) {
        std::cerr << "Warning: File size mismatch. Read " << infile.gcount() << " bytes.\n";
    }
    infile.close();

    // 2. 準備寫入 MIF 檔案
    std::ofstream mif(output_filename);
    if (!mif) {
        std::cerr << "Error: Cannot create output file " << output_filename << std::endl;
        return 1;
    }

    std::cout << "Generating Quartus MIF file: " << output_filename << " ...\n";

    // 3. 寫入 MIF Header (Quartus 格式)
    mif << "DEPTH = " << WIDTH << ";\n";       // 記憶體深度 (102 words)
    mif << "WIDTH = " << DATA_WIDTH << ";\n";  // 資料寬度 (16 bits)
    mif << "ADDRESS_RADIX = DEC;\n";           // 地址用十進位表示 (0..101)
    mif << "DATA_RADIX = HEX;\n";              // 資料用十六進位表示
    mif << "CONTENT\n";
    mif << "BEGIN\n";

    // 4. 掃描並計算每一列的 Bounds
    for (int x = 0; x < WIDTH; ++x) {
        // 預設值: Min = 255 (0xFF), Max = 0 (0x00) -> Combined = 0x00FF
        // 這表示該列沒有任何實體像素
        int current_min = 0xFF;
        int current_max = 0x00;
        bool first_opaque_found = false;

        for (int y = 0; y < HEIGHT; ++y) {
            // 計算索引 (Row-Major: y * width + x)
            int index = y * WIDTH + x;
            unsigned char pixel = pixel_data[index];

            // 尋找非透明像素 (實體)
            if (pixel != TRANSPARENT_PIXEL) {
                // 更新 Max (總是更新，因為 Y 遞增)
                current_max = y;

                // 更新 Min (只在第一次遇到時更新)
                if (!first_opaque_found) {
                    current_min = y;
                    first_opaque_found = true;
                }
            }
        }

        // 5. 組合 16-bit 數據: [15:8] = Max, [7:0] = Min
        unsigned int combined_data = (current_max << 8) | (current_min & 0xFF);

        // 6. 寫入 MIF 格式: Address : Data;
        mif << x << " : " 
            << std::hex << std::uppercase << std::setw(4) << std::setfill('0') 
            << combined_data 
            << ";\n";
    }

    // 7. 寫入 MIF Footer
    mif << "END;\n";
    mif.close();

    std::cout << "Done! File saved to " << output_filename << std::endl;
    return 0;
}
