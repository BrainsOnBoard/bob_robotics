// C++ includes
#include <cstdlib>
#include <iostream>

// GeNN robotics includes
#include "os/filesystem.h"

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
    string ext;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--no-sound") == 0) {
            copysound = false;
            ftype[i - 1] = skip;
            continue;
        }

        if (!OS::FileSystem::fileExists(argv[i])) {
            cerr << "Error: File " << argv[i] << " does not exist" << endl;
            return 1;
        }

        ext = OS::FileSystem::getExtension(argv[i]);
        if (ext == ".MP4") {
            anyvideo = true;
            ftype[i - 1] = video;
        } else if (ext == ".JPG" || ext == ".JPEG" || ext == ".JPE")
            ftype[i - 1] = image;
        else {
            cerr << "Warning : Only JPEG files and MP4 videos are supported -- skipping " << argv[i] << endl;
            ftype[i - 1] = skip;
        }
    }

    if (copysound && anyvideo && !OS::FileSystem::fileExists(FFMPEG_PATH)) {
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
