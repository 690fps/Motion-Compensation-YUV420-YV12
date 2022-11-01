// MotionCompensation.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <fstream>
#include <filesystem>
#include <execution>
#include <chrono>

using namespace std;
using namespace std::chrono;

int blockSize = 32;
int width = 352;
int height = 288;

typedef vector<uint8_t> byteVec;

byteVec getBlock(const byteVec& buffer, int x, int y, int stride, int compSize) {
    byteVec out;
    if (!(x >= 0 && x + compSize-1 < width && y >= 0 && y + compSize-1 < height)) {
        return out;
    }
    for (int i = 0; i < compSize; i++) {
        auto start = buffer.begin() + (y+i) * stride + x;
        out.insert(out.end(), start, start + compSize);
    }
    return out;
};

const uint8_t& getPixel(const byteVec& buffer, int x, int y, int stride) {
    return buffer[y*stride + x];
}

void processBlock(const byteVec inputCur[], const byteVec inputRef[], byteVec outPred[], vector<int16_t> outDiff[], int x, int y, int stride) {
    unsigned long minAbsDif = ULONG_MAX;
    int bestX = -1;
    int bestY = -1;
    byteVec curBlock = getBlock(inputCur[0], x, y, stride, blockSize);
    byteVec refBlock;

    //find prediction MV
    int minX = max(0, x - blockSize);
    int minY = max(0, y - blockSize);
    int maxX = min(stride - blockSize, x + blockSize);
    int maxY = min(height - blockSize, y + blockSize);
    for (int j = minY; j <= maxY; j++) {
        for (int i = minX; i <= maxX; i++) {
            unsigned long dif = 0;
            for (int jj = 0; jj < blockSize; jj++) {
                for (int ii = 0; ii < blockSize; ii++) {
                    dif += abs(getPixel(inputCur[0], x+ii, y+jj, stride) - getPixel(inputRef[0], i+ii, j+jj, stride));
                }
            }
            if (dif < minAbsDif) {
                minAbsDif = dif;
                bestX = i;
                bestY = j;
            }
        }
    }

    //use MV to get prediction and difference
    if (bestX >= 0 && bestY >= 0) {
        printf("\tBlock\t(%d, \t%d)\tMV => (%d, %d) \n", x, y, bestX - x, bestY - y);
        for (int comp = 0; comp < 3; comp++) {
            int subs =  comp ? 1 : 0;
            int compX = x >> subs;
            int compY = y >> subs;
            int compBestX = bestX >> subs;
            int compBestY = bestY >> subs;
            int compSize = blockSize >> subs;
            int compStride = stride >> subs;
            byteVec bestBlock = getBlock(inputRef[comp], compBestX, compBestY, compStride, compSize);
            curBlock = getBlock(inputCur[comp], compX, compY, compStride, compSize);

            //write prediction block
            for (int line = 0; line < compSize; line++) {
                auto outBuff = outPred[comp].begin() + (compY + line) * compStride + compX;
                auto begin = bestBlock.begin() + line * compSize;
                copy(begin, begin + compSize, outBuff);
            }
            //write difference block
            vector<int16_t> differenceBlock;
            for (int i = 0; i < bestBlock.size(); i++) {
                differenceBlock.push_back(curBlock[i] - bestBlock[i] + 255); //255 is offset to avoid negative values in YUV viewer
            }
            for (int line = 0; line < compSize; line++) {
                auto outBuff = outDiff[comp].begin() + (compY + line) * compStride + compX;
                auto begin = differenceBlock.begin() + line * compSize;
                copy(begin, begin + compSize, outBuff);
            }
        }
    } else {
        printf("Error searching motion vector\n");
    }
    return;
}

void error(string text) {
    printf("%s", text.c_str());
    getchar();
    exit(1);
}

int main()
{
    int frames = 10;

    int lumaSize = width * height;
    int chromaSize = lumaSize >> 2;
    //std::string path1 = filesystem::current_path().string() + "/akiyo_cif.yuv";
    string path1 = filesystem::current_path().string() + "/foreman_cif.yuv";
    ifstream file(path1, std::ios::in | std::ios::binary | std::ios::ate);
    if (!file)
        error("Error opening file");
    auto end = file.tellg();
    file.seekg(0, ios::beg);
    auto fileSize = size_t(end - file.tellg());
    if (fileSize < (lumaSize + 2*chromaSize) * frames)
        error("Wrong YV12 file size");
    ofstream outFilePred(filesystem::current_path().string() + "/prediction_8b.yuv", ios::binary);
    if (!outFilePred)
        error("Error creating output file prediction_8b.yuv");
    ofstream outFileDiff(filesystem::current_path().string() + "/difference_16b.yuv", ios::binary);
    if (!outFileDiff)
        error("Error creating output file difference_16b.yuv");

    byteVec current[3];
    current[0].resize(lumaSize);
    current[1].resize(chromaSize);
    current[2].resize(chromaSize);
    byteVec reference[3];
    reference[0].resize(lumaSize);
    reference[1].resize(chromaSize);
    reference[2].resize(chromaSize);
    byteVec prediction[3];
    prediction[0].resize(lumaSize);
    prediction[1].resize(chromaSize);
    prediction[2].resize(chromaSize);
    vector<int16_t> difference[3];
    difference[0].resize(lumaSize);
    difference[1].resize(chromaSize);
    difference[2].resize(chromaSize);

    vector<pair<int,int>> blocks;
    for (int y = 0; y < height; y += blockSize) {
        for (int x = 0; x < width; x += blockSize) {
            blocks.push_back(make_pair(x, y));
        }
    }

    for (int frame = 0; frame < frames; frame++) {
        printf("\nProcessing Frame %d\n", frame);
        printf("\tBlock (X,Y) => Prediction MV\n");
        for (int comp = 0; comp < 3; comp++) {
            if(!file.read((char *) current[comp].data(), comp ? chromaSize : lumaSize))
                error("Error reading data");
        }
        if (frame == 0) { //use first frame as reference
            reference[0] = current[0];
            reference[1] = current[1];
            reference[2] = current[2];
        }
        auto start = chrono::high_resolution_clock::now();
        for_each(
            execution::par_unseq,
            blocks.begin(),
            blocks.end(),
            [&](auto&& blockXY) {
                processBlock(current, reference, prediction, difference, blockXY.first, blockXY.second, width);
            }
        );
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(stop - start);
        printf("Frame processed in: %d ms \n", duration);

        for (int comp = 0; comp < 3; comp++) {
            outFilePred.write((char*)& prediction[comp][0], prediction[comp].size() * sizeof(uint8_t));
        }
        for (int comp = 0; comp < 3; comp++) {
            outFileDiff.write((char*)& difference[comp][0], difference[comp].size() * sizeof(int16_t));
        }
    }

    file.close();
    outFileDiff.close();
    outFilePred.close();

    printf("Process finished\n\n");
    getchar();
    return 0;

}
