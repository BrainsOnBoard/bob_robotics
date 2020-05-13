#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "common/pose.h"

// Third-party includes
#include "third_party/path.h"
#include "third_party/units.h"

// OpenCV includes
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <array>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace BoBRobotics {
namespace Navigation {
using namespace units::literals;

//------------------------------------------------------------------------
// BoBRobotics::Navigation::Range
//------------------------------------------------------------------------
//! A range of values in millimetres
struct Range
{
    using millimeter_t = units::length::millimeter_t;
    millimeter_t begin, end, separation;

    constexpr Range(const std::pair<millimeter_t, millimeter_t> beginAndEnd,
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

    constexpr Range(const millimeter_t value)
      : Range({ value, value }, 0_mm)
    {}

    constexpr Range()
      : begin{ millimeter_t{ std::numeric_limits<double>::quiet_NaN() } }
      , end{ millimeter_t{ std::numeric_limits<double>::quiet_NaN() } }
      , separation{ 0_mm }
    {}

    size_t size() const;
};

//------------------------------------------------------------------------
// BoBRobotics::Navigation::ImageDatabase
//------------------------------------------------------------------------
//! An interface for reading from and writing to folders of images
class ImageDatabase
{
    using degree_t = units::angle::degree_t;
    using millimeter_t = units::length::millimeter_t;

public:
    //! The metadata for an entry in an ImageDatabase
    struct Entry
    {
        Vector3<millimeter_t> position;
        degree_t heading;
        filesystem::path path;
        std::array<size_t, 3> gridPosition; //! For grid-type databases, indicates the x,y,z grid position

        cv::Mat load() const;
        cv::Mat loadGreyscale() const;
    };

    //! Base class for GridRecorder and RouteRecorder
    class Recorder {
    public:
        ~Recorder();

        //! Get an object for writing additional metadata to
        cv::FileStorage &getMetadataWriter();

        //! Don't save new metadata when this class is destroyed
        void abortSave();

        //! Save new metadata
        void save();

        //! Current number of *new* entries for the ImageDatabase
        size_t size() const;

        //! Get the format in which images will be saved
        std::string getImageFormat() const;

    private:
        ImageDatabase &m_ImageDatabase;
        const std::string m_ImageFormat;
        bool m_Recording;
        std::vector<Entry> m_NewEntries;

    protected:
        cv::FileStorage m_YAML;

        Recorder(ImageDatabase &imageDatabase,
                 const bool isRoute,
                 const std::string &imageFormat);

        void addEntry(const std::string &filename,
                      const cv::Mat &image,
                      const Vector3<millimeter_t> &position,
                      const degree_t heading,
                      const std::array<size_t, 3> &gridPosition = { 0, 0, 0 });
    };

    //! For recording a grid of images at a fixed heading
    class GridRecorder : public Recorder {
    public:
        GridRecorder(ImageDatabase &imageDatabase, const Range &xrange, const Range &yrange,
                     const Range &zrange = Range(0_mm), degree_t heading = 0_deg,
                     const std::string &imageFormat = "png");

        //! Get the physical position represented by grid coordinates
        Vector3<millimeter_t> getPosition(const std::array<size_t, 3> &gridPosition) const;

        //! Get a vector of all possible positions for this grid
        std::vector<Vector3<millimeter_t>> getPositions();

        //! Save a new image into the database
        void record(const cv::Mat &image);

        //! Save a new image into the database at the specified coordinates
        void record(const std::array<size_t, 3> &gridPosition,
                    const cv::Mat &image);

        size_t maximumSize() const;
        size_t sizeX() const;
        size_t sizeY() const;
        size_t sizeZ() const;

    private:
        const degree_t m_Heading;
        const Vector3<millimeter_t> m_Begin, m_Separation;
        const std::array<size_t, 3> m_Size;
        std::array<size_t, 3> m_Current;
    };

    //! For saving images recorded along a route
    class RouteRecorder : public Recorder {
    public:
        RouteRecorder(ImageDatabase &imageDatabase, const std::string &imageFormat = "png");

        //! Save a new image taken at the specified pose
        void record(const Vector3<millimeter_t> &position, degree_t heading, const cv::Mat &image);
    };

    ImageDatabase(const char *databasePath, bool overwrite = false);
    ImageDatabase(const std::string &databasePath, bool overwrite = false);
    ImageDatabase(const filesystem::path &databasePath, bool overwrite = false);

    //! Get the path of the directory corresponding to this ImageDatabase
    const filesystem::path &getPath() const;

    //! Get one Entry from the database
    const Entry &operator[](size_t i) const;

    //! Start iterator for the database entries
    std::vector<Entry>::const_iterator begin() const;

    //! End iterator for the database entries
    std::vector<Entry>::const_iterator end() const;

    //! Number of entries in this database
    size_t size() const;

    //! Check if there are any entries in this database
    bool empty() const;

    //! Check if the database is non-empty and a route-type database
    bool isRoute() const;

    //! Check if the database is non-empty and a grid-type database
    bool isGrid() const;

    //! Load all of the images in this database into memory and return
    std::vector<cv::Mat> getImages() const;

    //! Load all of the images in this database into the specified std::vector<>
    void getImages(std::vector<cv::Mat> &images) const;

    //! Access the metadata for this database via OpenCV's persistence API
    cv::FileNode getMetadata() const;

    //! Get the (directory) name of this database
    std::string getName() const;

    //! Start recording a grid of images
    GridRecorder getGridRecorder(const Range &xrange, const Range &yrange,
                                 const Range &zrange = Range(0_mm),
                                 degree_t heading = 0_deg,
                                 const std::string &imageFormat = "png");
    //! Start recording a route
    RouteRecorder getRouteRecorder(const std::string &imageFormat = "png");

    //! Get the resolution of saved images
    cv::Size getResolution() const;

    //! Check if this database has any saved metadata (yet)
    bool hasMetadata() const;

    /**!
     *  \brief Unwrap all the panoramic images in this database into a new
     *         folder, creating a new database.
     */
    void unwrap(const filesystem::path &destination, const cv::Size &unwrapRes);

    //! Get a filename for a route-type database
    static std::string getFilename(const size_t routeIndex,
                                   const std::string &imageFormat = "png");

    //! Get a filename for a grid-type database
    static std::string getFilename(const Vector3<millimeter_t> &position,
                                   const std::string &imageFormat = "png");

private:
    const filesystem::path m_Path;
    std::vector<Entry> m_Entries;
    std::unique_ptr<cv::FileStorage> m_MetadataYAML;
    cv::Size m_Resolution;
    bool m_IsRoute;
    static constexpr const char *MetadataFilename = "database_metadata.yaml";
    static constexpr const char *EntriesFilename = "database_entries.csv";

    void loadMetadata();
    void writeImage(const std::string &filename, const cv::Mat &image) const;
    void addNewEntries(std::vector<Entry> &newEntries);
    void writeEntry(std::ofstream &os, const Entry &e);
}; // ImageDatabase
} // Navigation
} // BoB robotics
