#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "common/path.h"
#include "common/pose.h"
#include "common/string.h"

// Third-party includes
#include "plog/Log.h"
#include "third_party/optional.hpp"
#include "third_party/path.h"
#include "third_party/units.h"

// OpenCV includes
#include <opencv2/opencv.hpp>

// TBB
#include <tbb/parallel_for_each.h>

// Standard C includes
#include <ctime>

// Standard C++ includes
#include <array>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <sstream>
#include <unordered_map>
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

    Range(const std::pair<millimeter_t, millimeter_t> beginAndEnd,
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

    Range(const millimeter_t value)
      : Range({ value, value }, 0_mm)
    {}

    Range()
      : begin{ millimeter_t{ std::numeric_limits<double>::quiet_NaN() } }
      , end{ millimeter_t{ std::numeric_limits<double>::quiet_NaN() } }
      , separation{ 0_mm }
    {}

    size_t size() const;
};

enum class DatabaseOptions
{
    Read = 1,
    Write = 2,
    Overwrite = 6
};

//------------------------------------------------------------------------
// BoBRobotics::Navigation::ImageDatabase
//------------------------------------------------------------------------
//! An interface for reading from and writing to folders of images
class ImageDatabase
{
    using degree_t = units::angle::degree_t;
    using millimeter_t = units::length::millimeter_t;
    using hertz_t = units::frequency::hertz_t;

public:
    static constexpr const char *MetadataFilename = "database_metadata.yaml";
    static constexpr const char *DefaultEntriesFilename = "database_entries.csv";

    //! The metadata for an entry in an ImageDatabase
    struct Entry
    {
        Pose3<millimeter_t, degree_t> pose;
        filesystem::path path;
        std::array<size_t, 3> gridPosition; //! For grid-type databases, indicates the x,y,z grid position
        std::unordered_map<std::string, std::string> extraFields;

        Entry();
        Entry(const Pose3<millimeter_t, degree_t> &_pose,
              filesystem::path _path,
              const std::array<size_t, 3> &_gridPosition,
              std::unordered_map<std::string, std::string> _extraFields);
        cv::Mat load(bool greyscale = true) const;
        bool hasExtraField(const std::string &name) const;
        const std::string &getExtraField(const std::string &name) const;
    };

    class FrameWriter {
    public:
        virtual ~FrameWriter() = default;
        virtual void writeFrame(const cv::Mat &frame, Entry &entry,
                                const std::function<std::string()> &getFileName) = 0;
    };

    class ImageFileWriter
      : public FrameWriter {
    public:
        ImageFileWriter(const ImageDatabase &, std::string imageFormat);
        void writeFrame(const cv::Mat &frame, Entry &entry,
                        const std::function<std::string()> &getFileName) override;

    private:
        const std::string m_ImageFormat;
    };

    class VideoFileWriter
      : public FrameWriter {
    public:
        VideoFileWriter(const ImageDatabase &, const std::string& extension, std::string codec);
        void writeFrame(const cv::Mat &frame, Entry &entry, const std::function<std::string()> &getFileName) override;
        const std::string &getVideoFileName() const;

    private:
        cv::VideoWriter m_Writer;
        const std::string m_FileName;
    };

    //! Base class for GridRecorder and RouteRecorder
    class Recorder
    {
    public:
        ~Recorder()
        {
            if (m_Recording) {
                save();
            }
        }

        //! Get an object for writing additional metadata to
        cv::FileStorage &getMetadataWriter() { return m_YAML; }

        //! Don't save new metadata when this class is destroyed
        void abortSave() { m_Recording = false; }

        //! Save new metadata
        void save()
        {
            // Write metadata to file
            {
                m_YAML << "}";

                const auto path = (m_ImageDatabase.m_Path / MetadataFilename).str();
                LOG_INFO << "Writing metadata to " << path << "...";
                std::ofstream ofs(path);
                ofs.exceptions(std::ios::badbit | std::ios::failbit);
                ofs << m_YAML.releaseAndGetString();
            }

            m_ImageDatabase.addNewEntries(m_NewEntries, m_ExtraFieldNames);
            m_Recording = false;
        }

        void setSaveImages(bool saveImages)
        {
            m_SaveImages = saveImages;
        }

        //! Current number of *new* entries for the ImageDatabase
        size_t size() const { return m_NewEntries.size(); }

    private:
        ImageDatabase &m_ImageDatabase;
        const std::vector<std::string> m_ExtraFieldNames;
        std::vector<Entry> m_NewEntries;
        std::unique_ptr<FrameWriter> m_Writer;
        bool m_Recording, m_SaveImages;

    protected:
        cv::FileStorage m_YAML;

        Recorder(ImageDatabase &imageDatabase,
                 bool isRoute,
                 std::unique_ptr<FrameWriter> writer,
                 std::vector<std::string> extraFieldNames)
          : m_ImageDatabase(imageDatabase)
          , m_ExtraFieldNames(std::move(extraFieldNames))
          , m_Writer(std::move(writer))
          , m_Recording(true)
          , m_SaveImages(true)
          , m_YAML(".yml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY)
        {
            BOB_ASSERT(!imageDatabase.m_ReadOnly);

            // Set this property of the ImageDatabase
            imageDatabase.m_IsRoute = isRoute;

            // Get current date and time
            const auto &creationTime = imageDatabase.getCreationTime();
            LOGW_IF(creationTime.tm_year < 122) << "Creation time of database is before 2022. If you want decent metadata you should fix your clock.";

            std::stringstream ss;
            ss << std::setfill('0')
               << std::setw(4) << (creationTime.tm_year + 1900) << "-"
               << std::setw(2) << (creationTime.tm_mon + 1) << "-"
               << std::setw(2) << creationTime.tm_mday << " "
               << std::setw(2) << creationTime.tm_hour << ":"
               << std::setw(2) << creationTime.tm_min << ":"
               << std::setw(2) << creationTime.tm_sec;

            // Write some metadata; users can add extra
            m_YAML << "metadata"
                   << "{"
                   << "time" << ss.str()
                   << "project_git_commit" << BOB_PROJECT_GIT_COMMIT
                   << "bob_robotics_git_commit" << BOB_ROBOTICS_GIT_COMMIT
#ifdef BOB_IS_EXPERIMENT
                   << "is_experiment" << 1
#else
                   << "is_experiment" << 0
#endif
                   << "type" << (isRoute ? "route" : "grid");
        }

        void addEntry(const cv::Mat &image, Entry entry,
                      const std::function<std::string()> &getFileName)
        {
            BOB_ASSERT(entry.extraFields.size() == m_ExtraFieldNames.size());

            if (m_SaveImages) {
                m_Writer->writeFrame(image, entry, getFileName);
            }
            m_NewEntries.emplace_back(std::move(entry));
        }

        template<class... Ts>
        void addEntry(const cv::Mat &image,
                      const std::function<std::string()> &getFileName,
                      const Pose3<millimeter_t, degree_t> &pose,
                      const std::array<size_t, 3> &gridPosition,
                      Ts&&... extraFieldValues)
        {
            // **NOTE** for reasons I can't quite comprehend, "BOB_ASSERT(sizeof...(extraFieldValues) == m_ExtraFieldNames.size());"
            // results in "error C2065: 'extraFieldValues': undeclared identifier" on Visual Studio
            constexpr size_t numExtraFields = sizeof...(extraFieldValues);
            BOB_ASSERT(numExtraFields == m_ExtraFieldNames.size());
            BOB_ASSERT(m_Recording);

            Entry newEntry{
                pose,
                "",
                gridPosition,
                {}
            };

            if (m_SaveImages) {
                m_Writer->writeFrame(image, newEntry, getFileName);
            }
            m_NewEntries.emplace_back(std::move(newEntry));

            setExtraFields(std::forward<Ts>(extraFieldValues)...);
        }

        const ImageDatabase &getImageDatabase() const
        {
            return m_ImageDatabase;
        }

        // Other strings
        template<class... Ts>
        void setExtraFields(std::string value, Ts&&... otherValues)
        {
            const auto &key = *(m_ExtraFieldNames.cend() - 1 - sizeof...(otherValues));
            m_NewEntries.back().extraFields.emplace(key, std::move(value));
            setExtraFields(std::forward<Ts>(otherValues)...);
        }

        // String literals
        template<class... Ts>
        void setExtraFields(const char *value, Ts&&... otherValues)
        {
            setExtraFields(std::string{ value }, std::forward<Ts>(otherValues)...);
        }

        // Float values
        template<class... Ts>
        void setExtraFields(float value, Ts&&... otherValues)
        {
            setExtraFields(writePreciseString(value), std::forward<Ts>(otherValues)...);
        }

        // Double values
        template<class... Ts>
        void setExtraFields(double value, Ts&&... otherValues)
        {
            setExtraFields(writePreciseString(value), std::forward<Ts>(otherValues)...);
        }

        // Other things
        template<class T, class... Ts>
        void setExtraFields(T value, Ts&&... otherValues)
        {
            setExtraFields(std::to_string(value), std::forward<Ts>(otherValues)...);
        }


        // Stop condition
        void setExtraFields() const
        {}
    };

    //! For recording a grid of images at a fixed heading
    class GridRecorder
      : public Recorder {
    public:
        GridRecorder(ImageDatabase &imageDatabase, const Range &xrange,
                     const Range &yrange, const Range &zrange = Range(0_mm),
                     degree_t heading = 0_deg,
                     std::string imageFormat = "png",
                     std::vector<std::string> extraFieldNames = {});

        //! Get the physical position represented by grid coordinates
        Vector3<millimeter_t> getPosition(const std::array<size_t, 3> &gridPosition) const;

        //! Get a vector of all possible positions for this grid
        std::vector<Vector3<millimeter_t>> getPositions() const;

        /**!
         * \brief Save a new image into the database with values for extra
         *        fields, if used
         */
        template<class... Ts>
        void record(const cv::Mat &image, Ts &&... extraFieldValues)
        {
            BOB_ASSERT(m_Current[2] < sizeZ());
            record(m_Current, image, std::forward<Ts>(extraFieldValues)...);

            if (++m_Current[0] == sizeX()) {
                m_Current[0] = 0;
                if (++m_Current[1] == sizeY()) {
                    m_Current[1] = 0;
                    m_Current[2]++;
                }
            }
        }

        /**!
         * \brief Save a new image into the database at the specified
         *        coordinates with values for extra fields, if used
         */
        template<class... Ts>
        void record(const std::array<size_t, 3> &gridPosition,
                    const cv::Mat &image, Ts&&... extraFieldValues)
        {
            const auto getFileName = [this]() {
                // Convert to integers
                std::array<int, 3> iposition;
                const auto position = getPosition(m_Current);
                std::transform(position.begin(), position.end(), iposition.begin(),
                    [](auto mm) {
                        return static_cast<int>(units::math::round(mm));
                    });

                // Make filename
                std::ostringstream ss;
                ss << "image_" << std::setw(7) << std::setfill('0') << std::showpos << std::internal
                   << std::setw(7) << iposition[0] << "_"
                   << std::setw(7) << iposition[1] << "_"
                   << std::setw(7) << iposition[2];
                return (this->getImageDatabase().getPath() / ss.str()).str();
            };

            const auto position = getPosition(gridPosition);
            constexpr degree_t nan{ NAN };
            this->addEntry(image, getFileName, { position, { m_Heading, nan, nan } },
                           gridPosition, std::forward<Ts>(extraFieldValues)...);
        }

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
    class RouteRecorder
      : public Recorder
    {
    public:
        RouteRecorder(ImageDatabase &imageDatabase, std::unique_ptr<FrameWriter> writer, std::vector<std::string> extraFieldNames)
          : Recorder(imageDatabase, true, std::move(writer), std::move(extraFieldNames))
        {
            BOB_ASSERT(!imageDatabase.isGrid());
        }

        /**!
         * \brief Save a new image taken at the specified pose with values for
         *        extra fields, if used
         */
        template<class... Ts>
        void record(const Pose3<millimeter_t, degree_t> &pose,
                    const cv::Mat &image, Ts &&...extraFieldValues)
        {
            this->addEntry(image, std::bind(&RouteRecorder::getFileName, this),
                           pose, { 0, 0, 0 }, std::forward<Ts>(extraFieldValues)...);
        }

        void record(const cv::Mat &image, Entry entry)
        {
            this->addEntry(image, entry, std::bind(&RouteRecorder::getFileName, this));
        }

    private:
        std::string getFileName()
        {
            std::ostringstream ss;
            ss << "image" << std::setw(5) << std::setfill('0') << this->size() + 1;
            return (this->getImageDatabase().getPath() / ss.str()).str();
        }
    };

    ImageDatabase();
    ImageDatabase(filesystem::path databasePath, DatabaseOptions options = DatabaseOptions::Read);
    ImageDatabase(filesystem::path databasePath, std::string entriesFileName);
    ImageDatabase(const std::tm &creationTime);

    //! Get the path of the directory corresponding to this ImageDatabase
    const filesystem::path &getPath() const;

    const std::vector<Entry> &getEntries() const;

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
    std::vector<cv::Mat> readImages(const cv::Size &size = {}, size_t frameSkip = 1,
                                    bool greyscale = true) const;

    //! Load all of the images in this database into the specified std::vector<>
    void readImages(std::vector<cv::Mat> &images, const cv::Size &size = {},
                    size_t frameSkip = 1, bool greyscale = true) const;

    //! Whether the database consists of panoramic images which need unwrapping
    std::experimental::optional<bool> needsUnwrapping() const;

    //! Access the metadata for this database via OpenCV's persistence API
    cv::FileNode getMetadata() const;

    //! Get the (directory) name of this database
    std::string getName() const;

    //! Get the time at which this database was created
    const std::tm &getCreationTime() const;

    //! Start recording a grid of images
    std::unique_ptr<GridRecorder> createGridRecorder(const Range &xrange,
                                                     const Range &yrange,
                                                     const Range &zrange = Range(0_mm),
                                                     degree_t heading = 0_deg,
                                                     std::string imageFormat = "png",
                                                     std::vector<std::string> extraFieldNames = {});

    //! Start recording a route
    std::unique_ptr<RouteRecorder> createRouteRecorder(std::string imageFormat = "png",
                                                       std::vector<std::string> extraFieldNames = {});

    /**!
     * \brief Start recording a route, saving images into video file using
     *        default AVI/MJPEG format.
     */
    std::unique_ptr<RouteRecorder> createVideoRouteRecorder(const cv::Size &resolution,
                                                            hertz_t fps,
                                                            std::vector<std::string> extraFieldNames = {});

    //! Start recording a route, saving images into video file with a custom codec
    std::unique_ptr<RouteRecorder> createVideoRouteRecorder(const cv::Size &resolution,
                                                            hertz_t fps,
                                                            const std::string &extension,
                                                            const std::string &codec,
                                                            std::vector<std::string> extraFieldNames = {});

    hertz_t getFrameRate() const;

    //! Get the resolution of saved images
    cv::Size getResolution() const;

    //! Check if this database has any saved metadata (yet)
    bool hasMetadata() const;

    //! Check if database contains a video file cf. multiple image files
    bool isVideoType() const;

    template<class Func>
    void forEachImage(const Func& func, const std::vector<std::pair<size_t, size_t>>& idx, bool greyscale = true) const
    {
        // If database consists of individual image files...
        if (m_VideoFilePath.empty()) {
            const auto load = [&](const auto &pair) {
                func(pair.first, m_Entries[pair.second].load(greyscale));
            };

            tbb::parallel_for_each(idx, load);
            return;
        }

        // ...otherwise we have a video file
        cv::VideoCapture cap{ m_VideoFilePath.str() };
        BOB_ASSERT(cap.isOpened());

        cv::Mat img;
        size_t curFrame = 0;

        // Skip to first frame
        for (; curFrame < m_FrameNumbers[0]; curFrame++) {
            BOB_ASSERT(cap.grab());
        }

        for (const auto &pair : idx) {
            const auto nextFrame = m_FrameNumbers[pair.second];

            /*
             * It is possible to explicitly jump to a given frame with OpenCV,
             * but that turns out to be reeeeeeeaaaally slow. Instead we just
             * grab the frames one by one, discarding those we don't want.
             * Note: This assumes that the range of values increases monotonically!
             */
            BOB_ASSERT(nextFrame >= curFrame);
            for (; curFrame <= nextFrame && cap.grab(); curFrame++)
                ;

            // Copy the grabbed frame into img
            BOB_ASSERT(cap.retrieve(img));

            if (greyscale) {
                cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
            }

            func(pair.first, img);
        }
    }

    template<class Func, class Container>
    void forEachImage(const Func &func, const Container &container,
                      bool greyscale = true) const
    {
        std::vector<std::pair<size_t, size_t>> idx;
        size_t i = 0;
        for (auto it = container.begin(); it != container.end(); ++it) {
            idx.emplace_back(i++, *it);
        }

        forEachImage(func, idx, greyscale);
    }

    template<class Func>
    void forEachImage(const Func &func, size_t frameSkip = 1,
                      bool greyscale = true) const
    {
        BOB_ASSERT(frameSkip > 0);

        std::vector<std::pair<size_t, size_t>> idx;
        idx.reserve(size());
        for (size_t i = 0; i < size(); i++) {
            idx.emplace_back(i, i * frameSkip);
        }

        forEachImage(func, idx, greyscale);
    }

    /**!
     *  \brief Unwrap all the panoramic images in this database into a new
     *         folder, creating a new database.
     */
    void unwrap(const filesystem::path &destination,
                const cv::Size &unwrapRes,
                size_t frameSkip = 1,
                bool greyscale = false) const;

    //! Return true if fn1 should be sorted before fn2
    static bool fileNameCompare(const std::string &fn1, const std::string &fn2);

private:
    filesystem::path m_Path, m_VideoFilePath;
    const std::string m_EntriesFileName;
    std::vector<Entry> m_Entries;
    std::vector<size_t> m_FrameNumbers;
    std::unique_ptr<cv::FileStorage> m_MetadataYAML;
    cv::Size m_Resolution;
    std::tm m_CreationTime;
    hertz_t m_FrameRate{ 0 };
    bool m_IsRoute;
    bool m_ReadOnly;
    std::experimental::optional<bool> m_NeedsUnwrapping;

    ImageDatabase(const std::tm *creationTime, filesystem::path databasePath,
                  DatabaseOptions options, std::string entriesFileName = DefaultEntriesFilename);

    void generateUnwrapCSV(const filesystem::path &destination, size_t frameSkip) const;
    void loadMetadata();
    bool loadCSV();
    bool readDirectoryEntries();
    void writeImage(const std::string &filename, const cv::Mat &image) const;

    void addNewEntries(std::vector<Entry> &newEntries,
                       const std::vector<std::string> &extraFieldNames);
}; // ImageDatabase
} // Navigation
} // BoB robotics
