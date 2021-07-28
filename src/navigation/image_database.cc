// BoB robotics includes
#include "common/macros.h"
#include "common/string.h"
#include "imgproc/opencv_unwrap_360.h"
#include "navigation/image_database.h"

// Third-party includes
#include "plog/Log.h"
#include "third_party/tinydir.h"

// TBB
#include <tbb/parallel_for.h>

// Standard C includes
#include <ctime>

// Standard C++ includes
#include <algorithm>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>

using namespace units::literals;
using namespace units::angle;
using namespace units::length;

namespace BoBRobotics {
namespace Navigation {

constexpr const char *ImageDatabase::MetadataFilename;
constexpr const char *ImageDatabase::EntriesFilename;

size_t
Range::size() const
{
    return (separation == 0_mm) ? 1 : (1 + ((end - begin) / separation).to<size_t>());
}

cv::Mat
ImageDatabase::Entry::load(bool greyscale) const
{
    auto img = cv::imread(path.str(), greyscale ? cv::IMREAD_GRAYSCALE : cv::IMREAD_COLOR);
    if (img.empty()) {
        throw std::runtime_error("Could not load image file at " + path.str());
    }
    return img;
}

bool
ImageDatabase::Entry::hasExtraField(const std::string &name) const
{
    return extraFields.find(name) != extraFields.cend();
}

const std::string &
ImageDatabase::Entry::getExtraField(const std::string &name) const
{
    const auto pos = extraFields.find(name);
    BOB_ASSERT(pos != extraFields.cend());

    // This is needed because operator[] is non-const for std::unordered_map
    return pos->second;
}

ImageDatabase::ImageFileWriter::ImageFileWriter(const ImageDatabase &,
                                                std::string imageFormat)
  : m_ImageFormat{ std::move(imageFormat) }
{}

void
ImageDatabase::ImageFileWriter::writeFrame(const cv::Mat &frame, Entry &entry)
{
    filesystem::path path = getCurrentFilenameRoot() + "." + m_ImageFormat;
    BOB_ASSERT(!path.exists()); // Don't overwrite data by default!
    BOB_ASSERT(cv::imwrite(path.str(), frame));

    entry.path = std::move(path);
}

ImageDatabase::VideoFileWriter::VideoFileWriter(const ImageDatabase &database,
                                                const std::pair<const std::string &, const std::string &> &format)
  : m_FileName{ database.getName() + "." + format.first }
{
    const auto path = database.getPath() / m_FileName;
    BOB_ASSERT(!path.exists()); // Don't overwrite by mistake

    const auto &fourcc = format.second;
    BOB_ASSERT(fourcc.size() == 4);
    m_Writer.open(path.str(),
                  cv::VideoWriter::fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]),
                  database.getFrameRate().value(),
                  database.getResolution());
    BOB_ASSERT(m_Writer.isOpened());
}

const std::string &
ImageDatabase::VideoFileWriter::getVideoFileName() const
{
    return m_FileName;
}

void
ImageDatabase::VideoFileWriter::writeFrame(const cv::Mat &frame, Entry &)
{
    m_Writer.write(frame);
}

ImageDatabase::GridRecorder::GridRecorder(ImageDatabase &imageDatabase,
                                          const Range &xrange,
                                          const Range &yrange,
                                          const Range &zrange,
                                          degree_t heading,
                                          std::string imageFormat,
                                          std::vector<std::string> extraFieldNames)
  : Recorder<ImageFileWriter>(imageDatabase, false, std::move(imageFormat),
                              std::move(extraFieldNames))
  , m_Heading(heading)
  , m_Begin(xrange.begin, yrange.begin, zrange.begin)
  , m_Separation(xrange.separation, yrange.separation, zrange.separation)
  , m_Size({ xrange.size(), yrange.size(), zrange.size() })
  , m_Current({ 0, 0, 0 })
{
    BOB_ASSERT(!imageDatabase.isRoute());

    // Save some extra, grid-specific metadata
    this->m_YAML << "grid"
                 << "{"
                 << "beginAtMM"
                 << "[:" << m_Begin[0]() << m_Begin[1]() << m_Begin[2]() << "]"
                 << "separationMM"
                 << "[:" << m_Separation[0]() << m_Separation[1]() << m_Separation[2]() << "]"
                 << "size"
                 << "[:" << (int) m_Size[0] << (int) m_Size[1] << (int) m_Size[2] << "]"
                 << "}";
}

std::string
ImageDatabase::GridRecorder::getCurrentFilenameRoot() const
{
    // Convert to integers
    std::array<int, 3> iposition;
    const auto position = getPosition(m_Current);
    std::transform(position.begin(), position.end(), iposition.begin(), [](auto mm) {
        return static_cast<int>(units::math::round(mm));
    });

    // Make filename
    std::ostringstream ss;
    ss << "image_" << std::setw(7) << std::setfill('0') << std::showpos << std::internal
       << std::setw(7) << iposition[0] << "_"
       << std::setw(7) << iposition[1] << "_"
       << std::setw(7) << iposition[2];
    return (this->getImageDatabase().getPath() / ss.str()).str();
}

Vector3<millimeter_t>
ImageDatabase::GridRecorder::getPosition(const std::array<size_t, 3> &gridPosition) const
{
    BOB_ASSERT(gridPosition[0] < m_Size[0] && gridPosition[1] < m_Size[1]
                && gridPosition[2] < m_Size[2]);
    Vector3<millimeter_t> position;
    for (size_t i = 0; i < position.size(); i++) {
        position[i] = (m_Separation[i] * gridPosition[i]) + m_Begin[i];
    }
    return position;
}

std::vector<Vector3<millimeter_t>>
ImageDatabase::GridRecorder::getPositions() const
{
    std::vector<Vector3<millimeter_t>> positions;
    positions.reserve(maximumSize());

    for (size_t x = 0; x < sizeX(); x++) {
        for (size_t y = 0; y < sizeY(); y++) {
            for (size_t z = 0; z < sizeZ(); z++) {
                positions.emplace_back(getPosition({ x, y, z }));
            }
        }
    }
    return positions;
}

size_t
ImageDatabase::GridRecorder::maximumSize() const
{
    return sizeX() * sizeY() * sizeZ();
}

size_t
ImageDatabase::GridRecorder::sizeX() const
{
    return m_Size[0];
}

size_t
ImageDatabase::GridRecorder::sizeY() const
{
    return m_Size[1];
}

size_t
ImageDatabase::GridRecorder::sizeZ() const
{
    return m_Size[2];
}

std::tm
getCurrentTime()
{
    auto timer = time(nullptr);
    return *std::localtime(&timer);
}

ImageDatabase::ImageDatabase()
  : ImageDatabase{ getCurrentTime() }
{
}

ImageDatabase::ImageDatabase(const std::tm &creationTime)
  : ImageDatabase{ &creationTime, Path::getNewPath(creationTime), false }
{
}

ImageDatabase::ImageDatabase(const char *databasePath, bool overwrite)
  : ImageDatabase(filesystem::path(databasePath), overwrite)
{}

ImageDatabase::ImageDatabase(const std::string &databasePath, bool overwrite)
  : ImageDatabase(filesystem::path(databasePath), overwrite)
{}

ImageDatabase::ImageDatabase(filesystem::path databasePath, bool overwrite)
  : ImageDatabase{ nullptr, std::move(databasePath), overwrite }
{
}

ImageDatabase::ImageDatabase(const std::tm *creationTime,
                             filesystem::path databasePath,
                             bool overwrite)
  : m_Path{ std::move(databasePath) }
  , m_CreationTime{}
{
    m_CreationTime.tm_isdst = -1;

    LOGI << "Using image database at " << m_Path;

    // If we're making a new database then we need a creation time
    if (overwrite && m_Path.exists()) {
        m_CreationTime = creationTime ? *creationTime : getCurrentTime();

        LOG_WARNING << "Database already exists; overwriting";
        filesystem::remove_all(m_Path);
        BOB_ASSERT(filesystem::create_directory(m_Path));
        return;
    }

    // Allow for loading a standalone video file as an image database
    if (m_Path.is_file()) {
        auto ext = m_Path.extension();
        for (char &c : ext) {
            c = ::tolower(c);
        }

        m_VideoFilePath = m_Path;
        m_Path = m_Path.parent_path();

        // Populate m_Entries with empty entries
        cv::VideoCapture cap{ m_VideoFilePath.str() };
        BOB_ASSERT(cap.isOpened());
        m_Entries.resize(static_cast<size_t>(cap.get(cv::CAP_PROP_FRAME_COUNT)));
    } else if (m_Path.exists()) {
        BOB_ASSERT(m_Path.is_directory());
    }

    // Try to read metadata from YAML file
    loadMetadata();

    // Try to read entries from CSV file
    if (!loadCSV()) {
        LOGW << "Could not find CSV file";
        if (m_VideoFilePath.empty()) {
            if (!readDirectoryEntries()) {
                // Make sure we have a directory to save into
                filesystem::create_directory(m_Path);

                /*
                 * Now that we *know* it's an empty database, it makes sense to
                 * assign a creation time.
                 */
                m_CreationTime = creationTime ? *creationTime : getCurrentTime();
            }
        } else {
            // Populate m_Entries with empty entries
            cv::VideoCapture cap{ m_VideoFilePath.str() };
            BOB_ASSERT(cap.isOpened());
            m_Entries.resize(static_cast<size_t>(cap.get(cv::CAP_PROP_FRAME_COUNT)));
        }
    }
}

bool
ImageDatabase::loadCSV()
{
    const auto entriesPath = m_Path / EntriesFilename;
    std::ifstream entriesFile(entriesPath.str());
    if (entriesFile.fail()) {
        return false;
    }
    entriesFile.exceptions(std::ios::badbit);

    std::string line;
    if (!std::getline(entriesFile, line)) {
        // ...then it's an empty file
        return false;
    }

    // Read field names, using comma as separator and trimming whitespace
    std::vector<std::string> fields;
    strSplit(line, ',', fields);
    std::for_each(fields.begin(), fields.end(), strTrim);
    const size_t numFields = fields.size();

    constexpr std::array<const char *, 8> defaultFieldNames{
        "X [mm]", "Y [mm]", "Z [mm]", "Heading [degrees]", "Filename", "Grid X",
        "Grid Y", "Grid Z"
    };

    /*
     * Go through field names, figuring out which are standard ones and which
     * are extra, user-defined ones
     */
    std::array<int, defaultFieldNames.size()> fieldNameIdx;
    std::fill(fieldNameIdx.begin(), fieldNameIdx.end(), -1);
    std::unordered_map<size_t, std::string> extraFieldNames;
    for (size_t i = 0; i < fields.size(); i++) {
        const auto def = std::find(defaultFieldNames.begin(), defaultFieldNames.end(), fields[i]);
        if (def != defaultFieldNames.end()) {
            // If it's a default one, save the column number for later parsing
            fieldNameIdx[std::distance(defaultFieldNames.begin(), def)] = i;
        } else {
            extraFieldNames.emplace(i, std::move(fields[i]));
        }
    }

    // Sanity check the file: we need the first four columns
    const auto validIdx = [](int idx) {
        return idx != -1;
    };
    BOB_ASSERT(std::all_of(fieldNameIdx.cbegin(), fieldNameIdx.cbegin() + 4, validIdx));

    /*
     * The Filename column is required for image-type databases, but doesn't
     * make sense for video-type ones.
     */
    BOB_ASSERT(validIdx(fieldNameIdx[4]) == m_VideoFilePath.empty());

    // Assume it's a grid if we have any of the Grid fields...
    if (!hasMetadata()) {
        m_IsRoute = !std::any_of(fieldNameIdx.cbegin() + 5, fieldNameIdx.cend(), validIdx);
    }

    // ...but we require *all* Grid fields for it to be valid
    if (!m_IsRoute) {
        BOB_ASSERT(std::all_of(fieldNameIdx.cbegin() + 5, fieldNameIdx.cend(), validIdx));
    }

    // Read data line by line
    while (std::getline(entriesFile, line)) {
        // Ignore empty lines
        strTrim(line);
        if (line.empty()) {
            continue;
        }

        // Use comma as delimiter
        strSplit(line, ',', fields);
        BOB_ASSERT(fields.size() == numFields);
        std::for_each(fields.begin(), fields.end(), strTrim);

        // Get values for any user-defined fields
        std::unordered_map<std::string, std::string> extraFields;
        for (size_t i = 0; i < fields.size(); i++) {
            const auto pos = extraFieldNames.find(i);
            if (pos != extraFieldNames.cend()) {
                extraFields.emplace(pos->second, std::move(fields[i]));
            }
        }

        // Get the value for a given default field
        const auto getDefaultField = [&fields, &fieldNameIdx](size_t i) {
            return fields[fieldNameIdx[i]];
        };

        // Get grid position for grid databases
        std::array<size_t, 3> gridPosition{0, 0, 0};
        if (!m_IsRoute) {
            gridPosition[0] = static_cast<size_t>(std::stoul(getDefaultField(5)));
            gridPosition[1] = static_cast<size_t>(std::stoul(getDefaultField(6)));
            gridPosition[2] = static_cast<size_t>(std::stoul(getDefaultField(7)));
        }

        // Save details to vector
        Entry entry{
            { millimeter_t(std::stod(getDefaultField(0))),
              millimeter_t(std::stod(getDefaultField(1))),
              millimeter_t(std::stod(getDefaultField(2))) },
            degree_t(std::stod(getDefaultField(3))),
            m_VideoFilePath.empty() ? m_Path / getDefaultField(4) : filesystem::path{},
            gridPosition,
            std::move(extraFields)
        };
        m_Entries.emplace_back(std::move(entry));
    }

    return true;
}

bool
ImageDatabase::readDirectoryEntries()
{
    if (!m_Path.exists()) {
        return false;
    }

    // For reading contents of directory
    tinydir_dir dir{};
    BOB_ASSERT(tinydir_open(&dir, m_Path.str().c_str()) == 0);
    for (; dir.has_next; BOB_ASSERT(tinydir_next(&dir) == 0)) {
        tinydir_file file;
        if (tinydir_readfile(&dir, &file)) {
            tinydir_close(&dir);
            throw std::runtime_error("Could not read from directory");
        }
        if (file.is_dir) {
            continue;
        }

        filesystem::path fileName = file.path;
        auto ext = fileName.extension();
        for (char &c : ext) {
            c = ::tolower(c);
        }
        if (ext == "jpg" || ext == "jpeg" || ext == "png") {
            // Save details to vector
            m_Entries.emplace_back();
            m_Entries.back().path = std::move(fileName);
        }
    }
    tinydir_close(&dir);

    // Try to sort by number in file name and fall back on alphabetic comparison
    const auto byName = [](const auto &e1, const auto &e2) {
        return ImageDatabase::fileNameCompare(e1.path.filename(), e2.path.filename());
    };
    std::sort(m_Entries.begin(), m_Entries.end(), byName);

    return !m_Entries.empty();
}

const std::tm &
ImageDatabase::getCreationTime() const
{
    // Check that we actually have a creation time
    BOB_ASSERT(m_CreationTime.tm_year != 0);
    return m_CreationTime;
}

ImageDatabase::GridRecorder
ImageDatabase::getGridRecorder(const Range &xrange, const Range &yrange,
                               const Range &zrange, degree_t heading,
                               std::string imageFormat,
                               std::vector<std::string> extraFieldNames)
{
    return ImageDatabase::GridRecorder{ *this, xrange, yrange, zrange, heading,
                                        std::move(imageFormat),
                                        std::move(extraFieldNames) };
}

ImageDatabase::RouteRecorder<ImageDatabase::ImageFileWriter>
ImageDatabase::getRouteRecorder(std::string imageFormat,
                                std::vector<std::string> extraFieldNames)
{
    return ImageDatabase::RouteRecorder<ImageFileWriter>{
        *this, std::move(imageFormat), std::move(extraFieldNames)
    };
}

ImageDatabase::RouteRecorder<ImageDatabase::VideoFileWriter>
ImageDatabase::getRouteVideoRecorder(const cv::Size &resolution,
                                     units::frequency::hertz_t fps,
                                     std::vector<std::string> extraFieldNames)
{
    return getRouteVideoRecorder(resolution, fps, "avi", "MJPG", std::move(extraFieldNames));
}

ImageDatabase::RouteRecorder<ImageDatabase::VideoFileWriter>
ImageDatabase::getRouteVideoRecorder(const cv::Size &resolution,
                                     units::frequency::hertz_t fps,
                                     const std::string &extension,
                                     const std::string &codec,
                                     std::vector<std::string> extraFieldNames)
{
    m_Resolution = resolution;
    m_FrameRate = fps;

    const std::pair<const std::string &, const std::string &> format{ extension, codec };
    RouteRecorder<VideoFileWriter> recorder{
        *this,
        format,
        std::move(extraFieldNames)
    };
    recorder.getMetadataWriter() << "videoFile" << recorder.getVideoFileName()
                                 << "frameRate" << m_FrameRate.value();

    return recorder;
}

//! Get the path of the directory corresponding to this ImageDatabase
const filesystem::path &
ImageDatabase::getPath() const
{
    return m_Path;
}

//! Get one Entry from the database
const ImageDatabase::Entry &
ImageDatabase::operator[](size_t i) const
{
    return m_Entries[i];
}

//! Start iterator for the database entries
std::vector<ImageDatabase::Entry>::const_iterator
ImageDatabase::begin() const
{
    return m_Entries.cbegin();
}

//! End iterator for the database entries
std::vector<ImageDatabase::Entry>::const_iterator
ImageDatabase::end() const
{
    return m_Entries.cend();
}

//! Number of entries in this database
size_t
ImageDatabase::size() const
{
    return m_Entries.size();
}

//! Check if there are any entries in this database
bool
ImageDatabase::empty() const
{
    return m_Entries.empty();
}

//! Check if the database is non-empty and a route-type database
bool
ImageDatabase::isRoute() const
{
    return !empty() && m_IsRoute;
}

//! Check if the database is non-empty and a grid-type database
bool
ImageDatabase::isGrid() const
{
    return !empty() && !m_IsRoute;
}

//! Load all of the images in this database into memory and return
std::vector<cv::Mat>
ImageDatabase::loadImages(const cv::Size &size, size_t frameSkip,
                          bool greyscale) const
{
    std::vector<cv::Mat> images;
    loadImages(images, size, frameSkip, greyscale);
    return images;
}

//! Load all of the images in this database into the specified std::vector<>
void
ImageDatabase::loadImages(std::vector<cv::Mat> &images, const cv::Size &size,
                          size_t frameSkip, bool greyscale) const
{
    size_t oldSize = images.size();
    images.resize(oldSize + m_Entries.size() / frameSkip);

    if (size == cv::Size{}) {
        forEachImage(
                [&images, oldSize](size_t i, const cv::Mat &img) {
                    img.copyTo(images[i + oldSize]);
                },
                frameSkip,
                greyscale);
    } else {
        forEachImage(
                [&images, oldSize, size](size_t i, const cv::Mat &img) {
                    cv::resize(img, images[i + oldSize], size);
                },
                frameSkip,
                greyscale);
    }
}

units::frequency::hertz_t
ImageDatabase::getFrameRate() const
{
    BOB_ASSERT(m_FrameRate != hertz_t{ 0 });
    return m_FrameRate;
}

//! Access the metadata for this database via OpenCV's persistence API
cv::FileNode
ImageDatabase::getMetadata() const
{
    BOB_ASSERT(hasMetadata());
    return m_MetadataYAML->operator[]("metadata");
}

//! Get the (directory) name of this database
std::string
ImageDatabase::getName() const
{
    return m_Path.filename();
}

//! Get the resolution of saved images
cv::Size
ImageDatabase::getResolution() const
{
    BOB_ASSERT(!m_Resolution.empty());
    return m_Resolution;
}

//! Check if this database has any saved metadata (yet)
bool
ImageDatabase::hasMetadata() const
{
    return static_cast<bool>(m_MetadataYAML);
}

void
ImageDatabase::generateUnwrapCSV(const filesystem::path &destination,
                                 size_t frameSkip) const
{
    const auto src = m_Path / EntriesFilename;
    if (!src.exists()) {
        return;
    }

    std::ifstream ifs;
    ifs.exceptions(std::ios::badbit);
    ifs.open(src.str());

    std::string line;
    if (!std::getline(ifs, line)) {
        // ...then it's an empty file
        return;
    }

    std::ofstream ofs;
    ofs.exceptions(std::ios::badbit | std::ios::failbit);
    ofs.open((destination / EntriesFilename).str());

    // Copy headers; if the source is a video file we need to append file names
    ofs << line;
    if (!m_VideoFilePath.empty()) {
        ofs << ",Filename";
    }
    ofs << "\n";

    for (size_t i = 0; i < m_Entries.size() / frameSkip; i++) {
        BOB_ASSERT(std::getline(ifs, line));
        BOB_ASSERT(!line.empty());

        ofs << line;
        if (!m_VideoFilePath.empty()) {
            ofs << ",image" << i << ".jpg";
        }
        ofs << "\n";

        // Skip the requested number of lines
        for (size_t j = 1; j < frameSkip; j++) {
            BOB_ASSERT(std::getline(ifs, line));
        }
    }
}

/**!
 *  \brief Unwrap all the panoramic images in this database into a new
 *         folder, creating a new database.
 */
void
ImageDatabase::unwrap(const filesystem::path &destination,
                      const cv::Size &unwrapRes, size_t frameSkip,
                      bool greyscale) const
{
    // Check that the database doesn't already exist
    BOB_ASSERT(!(destination / EntriesFilename).exists());

    BOB_ASSERT(frameSkip != 0);
    BOB_ASSERT(m_NeedsUnwrapping);

    // Create object for unwrapping images
    std::string camName;
    getMetadata()["camera"]["name"] >> camName;
    ImgProc::OpenCVUnwrap360 unwrapper(getResolution(), unwrapRes, camName);
    filesystem::create_directory(destination);

    // Generate new CSV file from the old (if it exists)
    generateUnwrapCSV(destination, frameSkip);

    /*
     * Create new metadata (YAML) file.
     *
     * There is no way to edit a persistence file in OpenCV, so we have
     * to do it ourselves in this ugly way.
     */
    {
        std::string line;
        std::ofstream ofs((destination / MetadataFilename).str());
        ofs.exceptions(std::ios::badbit | std::ios::failbit);
        std::ifstream ifs((m_Path / MetadataFilename).str());
        BOB_ASSERT(!ifs.fail());
        ifs.exceptions(std::ios::badbit);

        /*
         * Match expressions of the form: (whitespace)(key name): (value)
         *
         * This obvs isn't a proper YAML parser but it should be good enough.
         */
        const std::regex regex{ "^(\\s*)(\\w+):.*" };
        std::smatch match;
        while (std::getline(ifs, line)) {
            if (std::regex_match(line, match, regex)) {
                const auto &whitespace = match[1];
                const auto &key = match[2];

                // The new database won't need unwrapping anymore
                if (key == "needsUnwrapping") {
                    ofs << whitespace << "needsUnwrapping: 0\n";
                    continue;
                }

                // Set this field if converting to greyscale
                if (greyscale && key == "isGreyscale") {
                    ofs << whitespace << "isGreyscale: 1\n";
                    continue;
                }

                // Update the resolution
                if (key == "resolution") {
                    ofs << whitespace << "resolution: [ " << unwrapRes.width
                        << ", " << unwrapRes.height << " ]\n";
                    continue;
                }

                // The new database won't have a video file
                if (key == "videoFile") {
                    continue;
                }
            }

            ofs << line << "\n";
        }

        // Save this value
        ofs << "  frameSkip: " << frameSkip << "\n";

        // Append info about the unwrapping object, indenting appropriately
        cv::FileStorage fs(".yml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
        fs << "unwrapper" << unwrapper;
        std::stringstream ss;
        ss << fs.releaseAndGetString();
        std::getline(ss, line);
        std::getline(ss, line);
        while (std::getline(ss, line)) {
            ofs << "  " << line << "\n";
        }
    }

    // Finally, unwrap all images and save to new folder
    forEachImage([&](size_t i, const cv::Mat &image) {
        /*
         * *If* forEachImage runs in parallel, then each thread will need its own
         * copy of these scratch variables.
         */
        static thread_local cv::Mat unwrapped;
        static thread_local std::string outPath;

        unwrapper.unwrap(image, unwrapped);
        if (m_Entries[i].path.empty()) {
            outPath = "image" + std::to_string(i) + ".jpg";
        } else {
            outPath = m_Entries[i * frameSkip].path.filename();
        }

        BOB_ASSERT(cv::imwrite((destination / outPath).str(), unwrapped));
    }, frameSkip, greyscale);
}

bool
ImageDatabase::fileNameCompare(const std::string &fn1, const std::string &fn2)
{
    // Find file names like (string)(number)(extension), e.g. "image45.png"
    std::regex regex{ R"(^([^0-9]*)([0-9]+)\.(.+)$)" };

    /*
     * If the prefixes (e.g. "image" in "image45.png") and the file extensions
     * match...
     */
    std::smatch m1, m2;
    if (std::regex_match(fn1, m1, regex) && std::regex_match(fn2, m2, regex)
        && m1[1] == m2[1] && m1[3] == m2[3]) {

        // ...then use the numbers for comparison
        return std::stoul(m1[2]) < std::stoul(m2[2]);
    }

    // ...else fall back on alphabetic comparison
    return fn1 < fn2;
}

void
ImageDatabase::loadMetadata()
{
    const auto metadataPath = m_Path / MetadataFilename;
    const bool metadataPresent = metadataPath.exists();
    if (!metadataPresent) {
        LOG_WARNING << "No " << MetadataFilename << " file found";
        m_IsRoute = true;
        m_MetadataYAML.reset();
    } else {
        std::ifstream ifs{ metadataPath.str() };
        ifs.exceptions(std::ios::badbit | std::ios::failbit);

        std::stringstream ss;
        ss << "%YAML:1.0\n"
           << ifs.rdbuf();

        // Parse metadata file
        m_MetadataYAML = std::make_unique<cv::FileStorage>(ss.str(), cv::FileStorage::READ | cv::FileStorage::MEMORY);

        // What type of database is it?
        std::string dbtype;
        const auto metadata = getMetadata();
        metadata["type"] >> dbtype;
        if (dbtype == "route") {
            m_IsRoute = true;
        } else if (dbtype == "grid") {
            m_IsRoute = false;
        } else {
            throw std::runtime_error("Invalid database type \"" + dbtype + "\"");
        }

        // Check whether images are panoramic or not
        metadata["needsUnwrapping"] >> m_NeedsUnwrapping;

        // Get image resolution
        std::vector<int> size(2);
        metadata["camera"]["resolution"] >> size;
        m_Resolution = { size[0], size[1] };

        // These will only be set if database was recorded as a video file
        std::string videoFileName;
        metadata["videoFile"] >> videoFileName;
        if (!videoFileName.empty()) {
            m_VideoFilePath = m_Path / videoFileName;
        }

        double fps = 0;
        metadata["frameRate"] >> fps;
        m_FrameRate = hertz_t{ fps };

        std::string time;
        metadata["time"] >> time;
        if (!time.empty()) {
            std::istringstream ss{ time };
            ss >> std::get_time(&m_CreationTime, "%Y-%m-%d %H:%M:%S");
        }
    }
}

void
ImageDatabase::addNewEntries(std::vector<ImageDatabase::Entry> &newEntries,
                             const std::vector<std::string> &extraFieldNames)
{
    if (newEntries.empty()) {
        LOG_WARNING << "No new entries added, nothing will be written";
        return;
    }

    // Reload metadata, in case it's changed
    loadMetadata();

    const std::string path = (m_Path / EntriesFilename).str();
    LOG_INFO << "Writing entries to " << path << "...";

    // Move new entries into this object's vector
    m_Entries.reserve(m_Entries.size() + newEntries.size());
    for (auto &&e : newEntries) {
        m_Entries.emplace_back(std::move(e));
    }

    // Write image entries info to CSV file
    std::ofstream os;
    os.exceptions(std::ios::badbit | std::ios::failbit);
    os.open(path);
    os << "X [mm], Y [mm], Z [mm], Heading [degrees]";
    if (m_VideoFilePath.empty()) {
        os << ", Filename";
    }
    if (!m_IsRoute) {
        os << ", Grid X, Grid Y, Grid Z";
    }
    for (const auto &name : extraFieldNames) {
        os << ", " << name;
    }
    os << "\n";

    for (auto &e : m_Entries) {
        // These fields are always written...
        os << e.position[0]() << ", " << e.position[1]() << ", "
           << e.position[2]() << ", " << e.heading();

        // ...this is only written if we're not saving as a video
        if (m_VideoFilePath.empty()) {
            os << ", " << e.path.filename();
        }

        // ...and these are only written if it's a grid database
        if (!m_IsRoute) {
            os << ", " << e.gridPosition[0] << ", " << e.gridPosition[1]
               << ", " << e.gridPosition[2];
        }

        // Write any extra user-specified field values
        for (const auto &name : extraFieldNames) {
            os << ", " << e.extraFields[name];
        }

        os << "\n";
    }
}
} // Navigation
} // BoB robotics
