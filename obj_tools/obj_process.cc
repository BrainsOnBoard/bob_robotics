// Standard C++ includes
#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>

// Standard C includes
#include <cassert>

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

void copyPositions(const float(&min)[3], const float(&max)[3], 
                   std::ifstream &inputObjFile, std::ofstream &outputObjFile,
                   std::map<int, int> &positionIndices)
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

    std::cout << "1/3 - Copy positions:" << std::endl;
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

            // Update bounds
            for (unsigned int i = 0; i < 3; i++) {
                minBound[i] = std::min(minBound[i], position[i]);
                maxBound[i] = std::max(maxBound[i], position[i]);
            }

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
    }
   

    std::cout << "\t" << originalPositionID - 1 << " vertices" << std::endl;
    std::cout << "\tMin: (" << minBound[0] << ", " << minBound[1] << ", " << minBound[2] << ")" << std::endl;
    std::cout << "\tMax: (" << maxBound[0] << ", " << maxBound[1] << ", " << maxBound[2] << ")" << std::endl;

}

void findTexCoordsAndNormals(std::ifstream &inputObjFile,
                             const std::map<int, int> &positionIndices,
                             std::map<int, int> &texCoordIndices, std::map<int, int> &normalIndices)
{
    std::cout << "2/3 - Reading faces to find tex coords and normals:" << std::endl;
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
            facePositionIndices.clear();
            faceTexCoordIndices.clear();
            faceNormalIndices.clear();
            do {
                // Read indices i.e. P/T/N into string
                lineStream >> faceIndexString;

                // Convert into stream for processing
                std::istringstream faceIndexStream(faceIndexString);

                // Extract indices of position, tex coordinate and normal
                // **NOTE** obj indices start from 1
                std::getline(faceIndexStream, indexString, '/');
                facePositionIndices.push_back(stoi(indexString));
                std::getline(faceIndexStream, indexString, '/');
                faceTexCoordIndices.push_back(stoi(indexString));
                std::getline(faceIndexStream, indexString, '/');
                faceNormalIndices.push_back(stoi(indexString));
            } while (!lineStream.eof());

            // If all of the face position indices are included in the map
            if (std::all_of(facePositionIndices.cbegin(), facePositionIndices.cend(),
                [&positionIndices](int pos){ return (positionIndices.find(pos) != positionIndices.cend()); }))
            {
                // Add indices of texture coordinates and normals to maps
                // **NOTE** at this point, local ids are zero to be filled during next fass
                std::transform(faceTexCoordIndices.cbegin(), faceTexCoordIndices.cend(), texCoordIndices,
                               [](int id){ return std::make_pair(id, 0); });
                std::transform(faceNormalIndices.cbegin(), faceNormalIndices.cend(), normalIndices,
                               [](int id){ return std::make_pair(id, 0); });
                
                // Increment number of faces in bounds
                facesInBounds++;
            }

            // Increment face count
            totalFaces++;
        }
    }
    std::cout << "\t" << facesInBounds << "/" << totalFaces << " faces" << std::endl;
    std::cout << "\t" << texCoordIndices.size() << " tex coords" << std::endl;
    std::cout << "\t" << normalIndices.size() << " normals" << std::endl;
}

void completeCopy(std::ifstream &inputObjFile, std::ofstream &outputObjFile,
                  const std::map<int, int> &positionIndices,
                  std::map<int, int> &texCoordIndices, std::map<int, int> &normalIndices)
{
    std::cout << "3/3 - Copying remaining geometry:" << std::endl;
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
        
        else if (commandString == "f") {
            facePositionIndices.clear();
            faceTexCoordIndices.clear();
            faceNormalIndices.clear();
            bool validFace = true;
            do {
                // Read indices i.e. P/T/N into string
                lineStream >> faceIndexString;

                // Convert into stream for processing
                std::istringstream faceIndexStream(faceIndexString);

                // Extract indices of position, tex coordinate and normal
                // **NOTE** obj indices start from 1
                std::getline(faceIndexStream, indexString, '/');
                const auto position = positionIndices.find(stoi(indexString));
                if (position == positionIndices.cend()) {
                    validFace = false;
                    break;
                }

                std::getline(faceIndexStream, indexString, '/');
                const int texCoord = stoi(indexString);
                std::getline(faceIndexStream, indexString, '/');
                const int normal = stoi(indexString);
            } while (!lineStream.eof());
        }
        else if (commandString == "mtllib" || commandString == "o" || commandString == "v") {
            // Ignore previously processed commands
        }
        // Copy unhandled
        else {
            outputObjFile << lineString << std::endl;
        }
    }
}
}   // Anonymous namespace

int main(int argc, char **argv)
{
    const char *filename = "3D Model.obj";
    const float min[3]{0.0f, 0.0f, 0.0f};
    const float max[3]{100.0f, 100.0f, 100.0f};


    // Open obj file
    std::ifstream inputObjFile(filename);
    if(!inputObjFile.good()) {
        std::cerr << "Cannot open obj file: " << filename << std::endl;
        return false;
    }
    
    std::ofstream outputObjFile("test.obj");

    // Copy positions withing bounds to output file
    std::map<int, int> positionIndices;
    copyPositions(min, max, inputObjFile, outputObjFile,
                  positionIndices);
    
    std::cout << positionIndices.size() << " vertices within bounds" << std::endl;
    
    // Rewind
    inputObjFile.seekg(0);
    
    // Find the texture coordinates and normals required to alongside faces
    std::map<int, int> texCoordIndices;
    std::map<int, int> normalIndices;
    findTexCoordsAndNormals(inputObjFile, positionIndices,
                            texCoordIndices, normalIndices);

    // Rewind
    inputObjFile.seekg(0);

    // Complete copy of geometry to output file
    completeCopy(inputObjFile, outputObjFile,
                 positionIndices, texCoordIndices, normalIndices);
    return EXIT_SUCCESS;
}