// BoB robotics includes
#include "common/pose.h"
#include "common/read_objects.h"
#include "common/sfml_renderer.h"

// Eigen
#include <Eigen/Core>

// OpenCV
#include "opencv2/opencv.hpp"

// Standard C++ includes
#include <array>
#include <chrono>
#include <utility>
#include <thread>

using namespace BoBRobotics;
using namespace units::angle;
using namespace units::length;
using namespace units::literals;
using namespace units::math;
using namespace std::literals;

constexpr millimeter_t gridSize = 1_cm;
constexpr millimeter_t bufferSize = 10_cm;
constexpr std::pair<millimeter_t, millimeter_t> xLimits = { -0.5_m, 0.5_m };
constexpr std::pair<millimeter_t, millimeter_t> yLimits = xLimits;

class ArenaObject
  : public sf::Drawable
{
public:
    template<class VectorArrayType, class MatrixType>
    ArenaObject(const VectorArrayType &original, const MatrixType &resized, const SFMLRenderer<> &renderer)
      : m_GreenShape(original.size())
      , m_RedShape(original.size())
    {
        // Dark green
        m_GreenShape.setFillColor(sf::Color{ 0x00, 0x88, 0x00 });

        // Add each vertex to the shape
        for (size_t i = 0; i < original.size(); i++) {
            m_GreenShape.setPoint(i, renderer.vectorToPixel(original[i]));
        }

        m_RedShape.setFillColor(sf::Color::Red);
        for (size_t i = 0; i < original.size(); i++) {
            m_RedShape.setPoint(i, renderer.vectorToPixel(resized(i, 0), resized(i, 1)));
        }
    }

    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const override
    {
        target.draw(m_RedShape, states);
        target.draw(m_GreenShape, states);
    }

private:
    sf::ConvexShape m_GreenShape, m_RedShape;
};

template<class MatrixType, class VectorArray>
void vectorToEigen(MatrixType &matrix, const VectorArray &vectors)
{
    for (size_t i = 0; i < vectors.size(); i++) {
        matrix(i, 0) = static_cast<millimeter_t>(vectors[i].x()).value();
        matrix(i, 1) = static_cast<millimeter_t>(vectors[i].y()).value();
    }
}

template<class MatrixType, class PointsArray>
void eigenToPoints(PointsArray &points, const MatrixType &matrix)
{
    for (int i = 0; i < matrix.rows(); i++) {
        points[i] = cv::Point2i{ static_cast<int>((matrix(i, 0) - xLimits.first()) / gridSize),
                                 static_cast<int>((matrix(i, 1) - yLimits.first()) / gridSize) };
    }
}

int
main()
{
    SFMLRenderer<> renderer(Vector2<millimeter_t>{ -0.5_m, -0.5_m }, Vector2<millimeter_t>{ 0.5_m, 0.5_m });

    const auto toPixels = [&](const auto &limits) {
        return static_cast<int>((limits.second - limits.first) / gridSize);
    };
    cv::Mat arenaObjects(toPixels(xLimits), toPixels(yLimits), CV_8UC1, cv::Scalar{ 0 });

    const auto objects = readObjects("objects.yaml");
    std::vector<cv::Point2i> points;
    std::vector<ArenaObject> objectShapes;
    objectShapes.reserve(objects.size());
    for (auto &object : objects) {
        Eigen::MatrixX2d matrix(object.size(), 2);
        vectorToEigen(matrix, object);

        // Centre the object on the origin
        const Eigen::Vector2d centre = matrix.colwise().mean();
        Eigen::Matrix<double, 2, 2> translation;
        matrix.col(0).array() -= centre[0];
        matrix.col(1).array() -= centre[1];

        // Scale the object so we figure out the buffer zone around it
        const double width = matrix.col(0).maxCoeff() - matrix.col(0).minCoeff();
        const double scale = 1.0 + (bufferSize.value() / width);
        Eigen::Matrix<double, 2, 2> scaleMatrix;
        scaleMatrix << scale, 0,
                       0, scale;
        matrix *= scale;

        // Translate the object back to its origin location
        matrix.col(0).array() += centre[0];
        matrix.col(1).array() += centre[1];

        // Convert to OpenCV points and draw shape on image
        points.clear();
        points.resize(object.size());
        eigenToPoints(points, matrix);
        fillConvexPoly(arenaObjects, points, cv::Scalar{ 0xff });

        objectShapes.emplace_back(object, matrix, renderer);
    }

    while (renderer.isOpen()) {
        renderer.update(objectShapes);
        std::this_thread::sleep_for(20ms);
    }
}
