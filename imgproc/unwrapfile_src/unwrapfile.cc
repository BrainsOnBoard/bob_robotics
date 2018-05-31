// C++ includes
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <string>

// GeNN robotics includes
#include "third_party/path.h"

// local includes
#include "processfiles.h"

enum FileType {
    skip,
    image,
    video
};

int main(int argc, char** argv)
{
    if (argc == 1) {
        cout << "usage: punwrap [--no-sound] [file(s)]" << endl;
        return 0;
    }

    FileType ftype[argc - 1];
    bool anyvideo;
    std::string ext;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--no-sound") == 0) {
            copysound = false;
            ftype[i - 1] = skip;
            continue;
        }

        filesystem::path inputFile(argv[i]);
        if (!inputFile.exists()) {
            cerr << "Error: File " << inputFile.str() << " does not exist" << endl;
            return 1;
        }

        // Get extension and convert to lower case
        ext = inputFile.extension();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

        if (ext == ".mp4") {
            anyvideo = true;
            ftype[i - 1] = video;
        } else if (ext == ".jpg" || ext == ".jpeg" || ext == ".jpe")
            ftype[i - 1] = image;
        else {
            cerr << "Warning : Only JPEG files and MP4 videos are supported -- skipping " << argv[i] << endl;
            ftype[i - 1] = skip;
        }
    }

    if (copysound && anyvideo && !filesystem::path(FFMPEG_PATH).exists()) {
        cerr << "Warning: ffmpeg not found, sound will not be copied for videos" << endl;
        copysound = false;
    }

    for (int i = 0; i < argc - 1; i++) {
        if (ftype[i] == image)
            processjpeg(argv[i + 1]);
        else if (ftype[i] == video)
            processmp4(argv[i + 1]);
    }

    return 0;
}
