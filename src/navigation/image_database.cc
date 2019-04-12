// BoB robotics includes
#include "common/assert.h"
#include "common/logging.h"
#include "imgproc/opencv_unwrap_360.h"
#include "navigation/image_database.h"

// Standard C includes
#include <ctime>

// Standard C++ includes
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace units::literals;
using namespace units::angle;
using namespace units::length;

namespace BoBRobotics {
namespace Navigation {

constexpr Range::Range(const std::pair<millimeter_t, millimeter_t> beginAndEnd,
                       const millimeter_t separation)
  : begin(beginAndEnd.first)
  , end(beginAndEnd.second)
  , separation(separation)
{
    if (begin == end) {
        BOB_ASSERT(separation == 0_mm);
    } else {
        BOB_ASSERT(begin < end);
        BOB_ASSERT(separation > 0_mm);
    }
}

constexpr Range::Range(const millimeter_t value)
  : Range({ value, value }, 0_mm)
{}

size_t
Range::size() const
{
    return (separation == 0_mm) ? 1 : (1 + ((end - begin) / separation).to<size_t>());
}

cv::Mat
ImageDatabase::Entry::load() const
{
    BOB_ASSERT(path.exists());
    return cv::imread(path.str());
}

cv::Mat
ImageDatabase::Entry::loadGreyscale() const
{
    BOB_ASSERT(path.exists());
    return cv::imread(path.str(), cv::IMREAD_GRAYSCALE);
}

ImageDatabase::Recorder::~Recorder()
{
    if (m_Recording) {
        save();
    }
}

cv::FileStorage &
ImageDatabase::Recorder::getMetadataWriter()
{
    return m_YAML;
}

void
ImageDatabase::Recorder::abortSave()
{
    m_Recording = false;
}

void
ImageDatabase::Recorder::save()
{
    // Write metadata to file
    {
        m_YAML << "}";

        const auto path = (m_ImageDatabase.m_Path / MetadataFilename).str();
        LOG_INFO << "Writing metadata to " << path << "...";
        std::ofstream os(path);
        os << m_YAML.releaseAndGetString();
    }

    m_ImageDatabase.addNewEntries(m_NewEntries);
    m_Recording = false;
}

size_t
ImageDatabase::Recorder::size() const
{
    return m_NewEntries.size();
}

std::string
ImageDatabase::Recorder::getImageFormat() const
{
    return m_ImageFormat;
}

ImageDatabase::Recorder::Recorder(ImageDatabase &imageDatabase,
                                  const bool isRoute,
                                  const std::string &imageFormat)
  : m_ImageDatabase(imageDatabase)
  , m_ImageFormat(imageFormat)
  , m_Recording(true)
  , m_YAML(".yml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY)
{
    // Set this property of the ImageDatabase
    imageDatabase.m_IsRoute = isRoute;

    // Get current date and time
    std::time_t now = std::time(nullptr);
    char timeStr[sizeof("0000-00-00 00:00:00")];
    BOB_ASSERT(0 != std::strftime(timeStr, sizeof(timeStr), "%F %T", std::localtime(&now)));

    // Write some metadata; users can add extra
    m_YAML << "metadata"
           << "{"
           << "time" << timeStr
           << "type" << (isRoute ? "route" : "grid");
}

void
ImageDatabase::Recorder::addEntry(const std::string &filename,
                                  const cv::Mat &image,
                                  const Vector3<millimeter_t> &position,
                                  const degree_t heading,
                                  const std::array<size_t, 3> &gridPosition)
{
    BOB_ASSERT(m_Recording);
    m_ImageDatabase.writeImage(filename, image);
    m_NewEntries.emplace_back(Entry{
            position, heading, m_ImageDatabase.m_Path / filename, gridPosition });
}

ImageDatabase::GridRecorder::GridRecorder(ImageDatabase &imageDatabase,
                                          const Range &xrange,
                                          const Range &yrange,
                                          const Range &zrange,
                                          degree_t heading,
                                          const std::string &imageFormat)
  : Recorder(imageDatabase, false, imageFormat)
  , m_Heading(heading)
  , m_Begin(xrange.begin, yrange.begin, zrange.begin)
  , m_Separation(xrange.separation, yrange.separation, zrange.separation)
  , m_Size({ xrange.size(), yrange.size(), zrange.size() })
  , m_Current({ 0, 0, 0 })
{
    BOB_ASSERT(!imageDatabase.isRoute());

    // Save some extra, grid-specific metadata
    m_YAML << "grid"
           << "{"
           << "beginAtMM"
           << "[:" << m_Begin[0]() << m_Begin[1]() << m_Begin[2]() << "]"
           << "separationMM"
           << "[:" << m_Separation[0]() << m_Separation[1]() << m_Separation[2]() << "]"
           << "size"
           << "[:" << (int) m_Size[0] << (int) m_Size[1] << (int) m_Size[2] << "]"
           << "}";
}

//! Get the physical position represented by grid coordinates
Vector3<millimeter_t>
ImageDatabase::GridRecorder::getPosition(const std::array<size_t, 3> &gridPosition) const
{
    BOB_ASSERT(gridPosition[0] < m_Size[0] && gridPosition[1] < m_Size[1] && gridPosition[2] < m_Size[2]);
    Vector3<millimeter_t> position;
    for (size_t i = 0; i < position.size(); i++) {
        position[i] = (m_Separation[i] * gridPosition[i]) + m_Begin[i];
    }
    return position;
}

//! Get a vector of all possible positions for this grid
std::vector<Vector3<millimeter_t>>
ImageDatabase::GridRecorder::getPositions()
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

//! Save a new image into the database
void
ImageDatabase::GridRecorder::record(const cv::Mat &image)
{
    BOB_ASSERT(m_Current[2] < sizeZ());
    record(m_Current, image);

    if (++m_Current[0] == sizeX()) {
        m_Current[0] = 0;
        if (++m_Current[1] == sizeY()) {
            m_Current[1] = 0;
            m_Current[2]++;
        }
    }
}

//! Save a new image into the database at the specified coordinates
void
ImageDatabase::GridRecorder::record(const std::array<size_t, 3> &gridPosition, const cv::Mat &image)
{
    const auto position = getPosition(gridPosition);
    const std::string filename = ImageDatabase::getFilename(position, getImageFormat());
    addEntry(filename, image, position, m_Heading, { gridPosition[0], gridPosition[1], gridPosition[2] });
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

ImageDatabase::RouteRecorder::RouteRecorder(ImageDatabase &imageDatabase,
                                            const std::string &imageFormat)
  : Recorder(imageDatabase, true, imageFormat)
{
    BOB_ASSERT(!imageDatabase.isGrid());
}

void
ImageDatabase::RouteRecorder::record(const Vector3<millimeter_t> &position,
                                     degree_t heading,
                                     const cv::Mat &image)
{
    const std::string filename = ImageDatabase::getFilename(size(), getImageFormat());
    addEntry(filename, image, position, heading);
}

ImageDatabase::ImageDatabase(const char *databasePath, bool overwrite)
    : ImageDatabase(filesystem::path(databasePath), overwrite)
{}

ImageDatabase::ImageDatabase(const std::string &databasePath, bool overwrite)
    : ImageDatabase(filesystem::path(databasePath), overwrite)
{}

ImageDatabase::ImageDatabase(const filesystem::path &databasePath, bool overwrite)
    : m_Path(databasePath)
{
    if (overwrite && databasePath.exists()) {
        LOG_WARNING << "Database already exists; overwriting";
        filesystem::remove_all(databasePath);
    }

    // If we don't have any entries, it's an empty database
    const auto entriesPath = m_Path / EntriesFilename;
    if (!entriesPath.exists()) {
        // Make sure we have a directory to save into
        filesystem::create_directory(m_Path);
        return;
    }

    // Try to read metadata from YAML file
    loadMetadata();

    // Read in entries from CSV file
    std::ifstream entriesFile(entriesPath.str());
    std::string line, field;
    std::vector<std::string> fields;
    std::getline(entriesFile, line); // skip first line
    while (std::getline(entriesFile, line)) {
        std::stringstream lineStream(line);
        fields.clear();
        while (std::getline(lineStream, field, ',')) {
            // Trim whitespace from left (not C++ at its prettiest!)
            field.erase(field.begin(), std::find_if(field.begin(), field.end(), [](int ch) {
                return !std::isspace(ch);
            }));
            fields.push_back(field);
        }
        if (fields.size() < 5) {
            break;
        }

        std::array<size_t, 3> gridPosition;
        if (fields.size() >= 8) {
            if (!hasMetadata()) {
                // Infer that it is a grid
                m_IsRoute = false;
            }

            gridPosition[0] = static_cast<size_t>(std::stoul(fields[5]));
            gridPosition[1] = static_cast<size_t>(std::stoul(fields[6]));
            gridPosition[2] = static_cast<size_t>(std::stoul(fields[7]));
        } else {
            gridPosition = { 0, 0, 0 };
        }

        // Save details to vector
        Entry entry{
            { millimeter_t(std::stod(fields[0])),
                millimeter_t(std::stod(fields[1])),
                millimeter_t(std::stod(fields[2])) },
            degree_t(std::stod(fields[3])),
            m_Path / fields[4],
            gridPosition
        };
        m_Entries.push_back(entry);
    }
}

//! Get the path of the directory corresponding to this ImageDatabase
const filesystem::path &
ImageDatabase::getPath() const
{
    return m_Path;
}

//! Get one Entry from the database
const ImageDatabase::Entry &ImageDatabase::operator[](size_t i) const
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
ImageDatabase::getImages() const
{
    std::vector<cv::Mat> images;
    getImages(images);
    return images;
}

//! Load all of the images in this database into the specified std::vector<>
void
ImageDatabase::getImages(std::vector<cv::Mat> &images) const
{
    images.reserve(size());
    std::transform(begin(), end(), std::back_inserter(images), [](const auto &entry) {
        return entry.load();
    });
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

//! Start recording a grid of images
ImageDatabase::GridRecorder
ImageDatabase::getGridRecorder(const Range &xrange,
                               const Range &yrange,
                               const Range &zrange,
                               degree_t heading,
                               const std::string &imageFormat)
{
    return GridRecorder(*this, xrange, yrange, zrange, heading, imageFormat);
}

//! Start recording a route
ImageDatabase::RouteRecorder
ImageDatabase::getRouteRecorder(const std::string &imageFormat)
{
    return RouteRecorder(*this, imageFormat);
}

//! Get the resolution of saved images
cv::Size
ImageDatabase::getResolution() const
{
    BOB_ASSERT(hasMetadata());
    return m_Resolution;
}

//! Check if this database has any saved metadata (yet)
bool
ImageDatabase::hasMetadata() const
{
    return static_cast<bool>(m_MetadataYAML);
}

/**!
 *  \brief Unwrap all the panoramic images in this database into a new
 *         folder, creating a new database.
 */
void
ImageDatabase::unwrap(const filesystem::path &destination, const cv::Size &unwrapRes)
{
    // Check that the database doesn't already exist
    BOB_ASSERT(!(destination / EntriesFilename).exists());

    // Create object for unwrapping images
    std::string camName;
    getMetadata()["camera"]["name"] >> camName;
    ImgProc::OpenCVUnwrap360 unwrapper(getResolution(), unwrapRes, camName);
    filesystem::create_directory(destination);

    // Copy entries (CSV) file
    filesystem::copy_file(m_Path / EntriesFilename, destination / EntriesFilename);

    // Create new metadata (YAML) file
    {
        /*
            * There is no way to edit a persistence file in OpenCV, so we have
            * to do it ourselves in this ugly way.
            *
            * First, copy all fields, except for changing needsUnwrapping to false.
            */
        std::string line;
        std::ofstream ofs((destination / MetadataFilename).str());
        for (std::ifstream ifs((m_Path / MetadataFilename).str()); std::getline(ifs, line);) {
            const auto pos = line.find("needsUnwrapping:");
            if (pos != std::string::npos) {
                ofs << std::string(pos, ' ') << "needsUnwrapping: 0\n";
            } else {
                ofs << line << "\n";
            }
        }

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
    cv::Mat unwrapped(unwrapRes, CV_8UC3);
    std::string outPath;
    for (auto &entry : m_Entries) {
        unwrapper.unwrap(entry.load(), unwrapped);
        outPath = (destination / entry.path.filename()).str();
        LOG_INFO << "Writing to " << outPath;
        BOB_ASSERT(cv::imwrite(outPath, unwrapped));
    }
}

//! Get a filename for a route-type database
std::string
ImageDatabase::getFilename(const size_t routeIndex,
                           const std::string &imageFormat)
{
    std::ostringstream ss;
    ss << "image_" << std::setw(5) << std::setfill('0') << routeIndex << "." << imageFormat;
    return ss.str();
}

//! Get a filename for a grid-type database
std::string
ImageDatabase::getFilename(const Vector3<millimeter_t> &position,
                           const std::string &imageFormat)
{
    // Convert to integers
    std::array<int, 3> iposition;
    std::transform(position.begin(), position.end(), iposition.begin(), [](auto mm) {
        return static_cast<int>(units::math::round(mm));
    });

    // Make filename
    std::ostringstream ss;
    ss << "image_" << std::setw(7) << std::setfill('0') << std::showpos << std::internal
       << std::setw(7) << iposition[0] << "_"
       << std::setw(7) << iposition[1] << "_"
       << std::setw(7) << iposition[2] << "." << imageFormat;
    return ss.str();
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
        std::ifstream ifs(metadataPath.str());
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

        // Get image resolution
        std::vector<int> size(2);
        metadata["camera"]["resolution"] >> size;
        m_Resolution = { size[0], size[1] };
    }
}

void
ImageDatabase::writeImage(const std::string &filename, const cv::Mat &image) const
{
    const filesystem::path path = m_Path / filename;
    BOB_ASSERT(!path.exists()); // Don't overwrite data by default!
    cv::imwrite(path.str(), image);
}

void
ImageDatabase::addNewEntries(std::vector<Entry> &newEntries)
{
    if (newEntries.empty()) {
        LOG_WARNING << "No new entries added, nothing will be written";
        return;
    }

    const std::string path = (m_Path / EntriesFilename).str();
    LOG_INFO << "Writing entries to " << path << "...";

    // Move new entries into this object's vector
    m_Entries.reserve(m_Entries.size() + newEntries.size());
    for (auto &&e : newEntries) {
        m_Entries.emplace_back(std::move(e));
    }

    // Write image entries info to CSV file
    std::ofstream os(path);
    if (m_IsRoute) {
        os << "X [mm], Y [mm], Z [mm], Heading [degrees], Filename" << std::endl;
        for (auto &e : m_Entries) {
            writeEntry(os, e);
            os << std::endl;
        }
    } else {
        os << "X [mm], Y [mm], Z [mm], Heading [degrees], Filename, Grid X, Grid Y, Grid Z" << std::endl;
        for (auto &e : m_Entries) {
            writeEntry(os, e);
            os << ", " << e.gridPosition[0] << ", " << e.gridPosition[1] << ", " << e.gridPosition[2];
            os << std::endl;
        }
    }

    // Reload metadata, in case it's changed
    loadMetadata();
}

void
ImageDatabase::writeEntry(std::ofstream &os, const ImageDatabase::Entry &e)
{
    os << e.position[0]() << ", " << e.position[1]() << ", "
       << e.position[2]() << ", " << e.heading() << ", " << e.path.filename();
}
} // Navigation
} // BoB robotics
