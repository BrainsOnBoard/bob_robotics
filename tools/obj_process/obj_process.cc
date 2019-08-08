// BoB robotics includes
#include "common/logging.h"
#include "common/macros.h"
#include "third_party/path.h"

// Standard C++ includes
#include <algorithm>
#include <fstream>
#include <map>
#include <sstream>
#include <vector>

#ifdef __GNUC__
#include <fcntl.h>
#include <ext/stdio_filebuf.h>
#endif

//----------------------------------------------------------------------------
// Anonymous namespace
//----------------------------------------------------------------------------
namespace
{
template<unsigned int N>
void readVector(std::istringstream &stream, float(&vector)[N])
{
    // Read components and push back
    float x;
    for(unsigned int i = 0; i < N; i++) {
        stream >> x;
        vector[i] = x;
    }
}

bool getRemappedIndex(const std::map<int, int> &indices, const std::string &indexString,
                      std::vector<int> &remappedIndices)
{
    // If index isn't found, return false
    const auto index = indices.find(stoi(indexString));
    if (index == indices.cend()) {
        return false;
    }
    // Otherwise, add remapped index to vector and return true
    else {
        remappedIndices.push_back(index->second);
        return true;
    }
}

void findBounds(std::istream &inputObjFile)
{
    // Initialise bounds
    float minBound[3]{
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max() };
    float maxBound[3]{
        std::numeric_limits<float>::min(),
        std::numeric_limits<float>::min(),
        std::numeric_limits<float>::min() };

    LOGI << "1/1 - Finding bounds:";
    std::string lineString;
    std::string commandString;
    for(size_t l = 0; std::getline(inputObjFile, lineString); l++) {
        // Entirely skip comment or empty lines
        if (lineString[0] == '#' || lineString.empty()) {
            continue;
        }

        // Wrap line in stream for easier parsing
        std::istringstream lineStream(lineString);

        // Read command from first token
        lineStream >> commandString;

        // If line is a position
        if (commandString == "v") {
            // Read position
            float position[3];
            readVector(lineStream, position);

            // Update bounds
            for (unsigned int i = 0; i < 3; i++) {
                minBound[i] = std::min(minBound[i], position[i]);
                maxBound[i] = std::max(maxBound[i], position[i]);
            }
        }
        // Otherwise, if we've hit the faces section there should be no more vertices so break;
        else if(commandString == "f") {
            break;
        }
    }

    LOGI << "\tMin: (" << minBound[0] << ", " << minBound[1] << ", " << minBound[2] << ")";
    LOGI << "\tMax: (" << maxBound[0] << ", " << maxBound[1] << ", " << maxBound[2] << ")";
}

void copyPositions(const float(&min)[3], const float(&max)[3],
                   std::istream &inputObjFile, std::ofstream &outputObjFile,
                   std::map<int, int> &positionIndices)
{
    LOGI << "1/3 - Copy positions:";
    std::string lineString;
    std::string commandString;
    int originalPositionID = 1;
    int remappedPositionID = 1;
    while (std::getline(inputObjFile, lineString)) {
        // Entirely skip comment or empty lines
        if (lineString[0] == '#' || lineString.empty()) {
            continue;
        }

        // Wrap line in stream for easier parsing
        std::istringstream lineStream(lineString);

        // Read command from first token
        lineStream >> commandString;

        // If line is a position
        if (commandString == "v") {
            // Read position
            float position[3];
            readVector(lineStream, position);

            // If position is within bounds
            if (position[0] >= min[0] && position[1] >= min[1] && position[2] >= min[2] &&
                position[0] < max[0] && position[1] < max[1] && position[2] < max[2])
            {
                // Copy vertex to output file
                outputObjFile << lineString << std::endl;

                // Add mapping between original adn remapped ID to map
                positionIndices.insert(std::make_pair(originalPositionID, remappedPositionID));

                // Incrememnt remapped vertex ID
                remappedPositionID++;
            }

            // Remap original vertex ID
            originalPositionID++;
        }
        else if (commandString == "mtllib" || commandString == "o") {
            outputObjFile << lineString << std::endl;
        }
        // Otherwise, if we've hit the faces section there should be no more vertices so break;
        else if(commandString == "f") {
            break;
        }
    }


    LOGI << "\t" << remappedPositionID - 1 << "/" << originalPositionID - 1 << " vertices";
}

void findFaces(std::istream &inputObjFile,
               const std::map<int, int> &positionIndices,
               std::map<int, int> &texCoordIndices, std::map<int, int> &normalIndices)
{
    LOGI << "2/3 - Reading faces to find tex coords and normals:";
    std::string lineString;
    std::string commandString;
    std::string faceIndexString;
    std::string indexString;
    std::vector<int> facePositionIndices;
    std::vector<int> faceTexCoordIndices;
    std::vector<int> faceNormalIndices;
    int totalFaces = 0;
    int facesInBounds = 0;
    while (std::getline(inputObjFile, lineString)) {
        // Entirely skip comment or empty lines
        if (lineString[0] == '#' || lineString.empty()) {
            continue;
        }

        // Wrap line in stream for easier parsing
        std::istringstream lineStream(lineString);

        // Read command from first token
        lineStream >> commandString;

        if (commandString == "f") {
            // Read indices i.e. P/T[/N] into string
            facePositionIndices.clear();
            faceTexCoordIndices.clear();
            faceNormalIndices.clear();
            while(lineStream >> faceIndexString) {
                // Convert into stream for processing
                std::istringstream faceIndexStream(faceIndexString);

                // Extract indices of position, tex coordinate and normal
                // **NOTE** obj indices start from 1
                std::getline(faceIndexStream, indexString, '/');
                facePositionIndices.push_back(stoi(indexString));
                std::getline(faceIndexStream, indexString, '/');
                faceTexCoordIndices.push_back(stoi(indexString));
                if(std::getline(faceIndexStream, indexString, '/')) {
                    faceNormalIndices.push_back(stoi(indexString));
                }
            }

            // If all of the face position indices are included in the map
            if (std::all_of(facePositionIndices.cbegin(), facePositionIndices.cend(),
                [&positionIndices](int pos){ return (positionIndices.find(pos) != positionIndices.cend()); }))
            {
                // Add indices of texture coordinates to maps
                // **NOTE** at this point, remapped ids are zero to be filled during next pass
                std::transform(faceTexCoordIndices.cbegin(), faceTexCoordIndices.cend(), std::inserter(texCoordIndices, texCoordIndices.end()),
                               [](int id){ return std::make_pair(id, 0); });

                // If any face normals were found, add indices of normals to maps
                // **NOTE** at this point, remapped ids are zero to be filled during next pass
                if(!faceNormalIndices.empty()) {
                    std::transform(faceNormalIndices.cbegin(), faceNormalIndices.cend(), std::inserter(normalIndices, normalIndices.end()),
                                [](int id){ return std::make_pair(id, 0); });
                }

                // Increment number of faces in bounds
                facesInBounds++;
            }

            // Increment face count
            totalFaces++;
        }
    }
    LOGI << "\t" << facesInBounds << "/" << totalFaces << " faces";
    LOGI << "\t" << texCoordIndices.size() << " tex coords";
    LOGI << "\t" << normalIndices.size() << " normals";
}

void completeCopy(std::istream &inputObjFile, std::ofstream &outputObjFile,
                  const std::map<int, int> &positionIndices,
                  std::map<int, int> &texCoordIndices, std::map<int, int> &normalIndices)
{
    LOGI << "3/3 - Copying remaining geometry:";
    std::string lineString;
    std::string commandString;
    std::string faceIndexString;
    std::string indexString;
    int originalTexCoordID = 1;
    int originalNormalID = 1;
    int remappedTexCoordID = 1;
    int remappedNormalID = 1;
    std::vector<int> facePositionIndices;
    std::vector<int> faceTexCoordIndices;
    std::vector<int> faceNormalIndices;
    while (std::getline(inputObjFile, lineString)) {
        // Entirely skip comment or empty lines
        if (lineString[0] == '#' || lineString.empty()) {
            continue;
        }

        // Wrap line in stream for easier parsing
        std::istringstream lineStream(lineString);

        // Read command from first token
        lineStream >> commandString;

        // If line is a texture coordinate
        if (commandString == "vt") {
            // If texture coord should be includes
            auto texCoord = texCoordIndices.find(originalTexCoordID);
            if (texCoord != texCoordIndices.cend()) {
                // Update mapping with new texture coord id
                texCoord->second = remappedTexCoordID;

                // Write texture coord to output
                outputObjFile << lineString << std::endl;

                // Increment remapped texture coord id
                remappedTexCoordID++;
            }

            // Increment original texture coord id
            originalTexCoordID++;
        }
        // Otherwise, if line is a vertex normal
        else if (commandString == "vn") {
            // If normal should be includes
            auto normal = normalIndices.find(originalNormalID);
            if (normal != normalIndices.cend()) {
                // Update mapping with new normal id
                normal->second = remappedNormalID;

                // Write normal to output
                outputObjFile << lineString << std::endl;

                // Increment remapped normal id
                remappedNormalID++;
            }

            // Increment original normal id
            originalNormalID++;
        }
        // Otherwise, if line is a face
        else if (commandString == "f") {
            // Read indices i.e. P/T[/N] into string
            facePositionIndices.clear();
            faceTexCoordIndices.clear();
            faceNormalIndices.clear();
            bool validFace = true;
            while(lineStream >> faceIndexString) {
                // Convert into stream for processing
                std::istringstream faceIndexStream(faceIndexString);

                // Update mapping with remapped position index
                std::getline(faceIndexStream, indexString, '/');
                if(!getRemappedIndex(positionIndices, indexString, facePositionIndices)) {
                    validFace = false;
                    break;
                }

                // Update mapping with remapped texture coordinate index
                std::getline(faceIndexStream, indexString, '/');
                if(!getRemappedIndex(texCoordIndices, indexString, faceTexCoordIndices)) {
                    validFace = false;
                    break;
                }

                // If normals are present, update mapping with remapped normal index
                if(std::getline(faceIndexStream, indexString, '/')) {
                    if(!getRemappedIndex(normalIndices, indexString, faceNormalIndices)) {
                        validFace = false;
                        break;
                    }
                }
            }

            // If a valid face has been parsed
            if(validFace) {
                // Check all sizes match
                BOB_ASSERT(facePositionIndices.size() == faceTexCoordIndices.size());
                BOB_ASSERT(faceNormalIndices.empty() || faceTexCoordIndices.size() == faceNormalIndices.size());

                // Write new face
                outputObjFile << "f ";
                for(size_t i = 0; i < facePositionIndices.size(); i++) {
                    if(faceNormalIndices.empty()) {
                        outputObjFile << facePositionIndices[i] << "/" << faceTexCoordIndices[i];
                    }
                    else {
                        outputObjFile << facePositionIndices[i] << "/" << faceTexCoordIndices[i] << "/" << faceNormalIndices[i];
                    }

                    // If this isn't the last face vertex, add whitespace
                    if(i != (facePositionIndices.size() - 1)) {
                        outputObjFile << " ";
                    }
                }

                outputObjFile << std::endl;
            }
        }
        // Otherwise, if this is a line
        else if(commandString == "l") {
            facePositionIndices.clear();
            bool validLine = true;
            do {
                // Read index into string
                lineStream >> faceIndexString;

                // Add remapped position index to vector
                if(!getRemappedIndex(positionIndices, faceIndexString, facePositionIndices)) {
                    validLine = false;
                    break;
                }
            } while (!lineStream.eof());

            // If a valid line has been parsed
            if(validLine) {
                 // Write new line
                outputObjFile << "l ";
                for(size_t i = 0; i < facePositionIndices.size(); i++) {
                    outputObjFile << facePositionIndices[i];

                    // If this isn't the last line vertex, add whitespace
                    if(i != (facePositionIndices.size() - 1)) {
                        outputObjFile << " ";
                    }
                }
                outputObjFile << std::endl;
            }
        }
        // Otherwise, if command has already been handled by copyPositions, ignore
        else if (commandString == "mtllib" || commandString == "o" || commandString == "v") {
        }
        // Otherwise, copy line directly
        else {
            outputObjFile << lineString << std::endl;
        }
    }
}
}   // Anonymous namespace

int main(int argc, char **argv)
{

    if(argc < 2) {
        LOGF << "At least one argument (object filename) required";
        return EXIT_FAILURE;
    }
    else {
#ifdef __GNUC__
        // Open file using POSIX API
        int fd = open(argv[1], O_RDONLY);
        if(fd == -1) {
            LOGF << "Cannot open obj file: " << argv[1];
            return EXIT_FAILURE;
        }

        // Advise the kernel of our access pattern.
        // https://stackoverflow.com/questions/17925051/fast-textfile-reading-in-c
        posix_fadvise(fd, 0, 0, POSIX_FADV_SEQUENTIAL);

        // **HACK** apply terrifying GCC hack to build a std::istream from a POSIX handle
        // https://stackoverflow.com/questions/2746168/how-to-construct-a-c-fstream-from-a-posix-file-descriptor
        __gnu_cxx::stdio_filebuf<char> filebuf(fd, std::ios::in);
        std::istream inputObjFile(&filebuf);
#else
         // Open obj file
        std::ifstream inputObjFile(argv[1]);
#endif
        if(!inputObjFile.good()) {
            LOGF << "Cannot open obj file: " << argv[1];
            return EXIT_FAILURE;
        }

        // If only one argument is passed, find bounds of model
        if(argc == 2) {
            findBounds(inputObjFile);
            return EXIT_SUCCESS;
        }
        // Otherwise
        else if(argc == 8) {
            // Create an output path for object
            const auto inputPath = filesystem::path(argv[1]).make_absolute();
            const auto outputPath = inputPath.parent_path() / ("output_" + inputPath.filename());

            // Open output file
            std::ofstream outputObjFile(outputPath.str());

            // Parse bounds
            const float min[3]{ strtof(argv[2], nullptr), strtof(argv[3], nullptr), strtof(argv[4], nullptr) };
            const float max[3]{ strtof(argv[5], nullptr), strtof(argv[6], nullptr), strtof(argv[7], nullptr) };

            // Copy positions withing bounds to output file
            std::map<int, int> positionIndices;
            copyPositions(min, max, inputObjFile, outputObjFile,
                        positionIndices);

            // Rewind
            inputObjFile.clear();
            inputObjFile.seekg(0);

            // Find the faces required for these vertices
            std::map<int, int> texCoordIndices;
            std::map<int, int> normalIndices;
            findFaces(inputObjFile, positionIndices,
                    texCoordIndices, normalIndices);

            // Rewind
            inputObjFile.clear();
            inputObjFile.seekg(0);

            // Complete copy of geometry to output file
            completeCopy(inputObjFile, outputObjFile,
                        positionIndices, texCoordIndices, normalIndices);
            return EXIT_SUCCESS;
        }
        else {
            LOGF << "Object filename, minX, minY, minZ, maxX, maxY, maxZ arguments required";
            return EXIT_FAILURE;
        }
    }
}
