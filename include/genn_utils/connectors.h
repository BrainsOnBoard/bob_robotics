#pragma once

// BoB robotics includes
#include "../common/macros.h"

// Standard C++ includes
#include <algorithm>
#include <numeric>
#include <random>
#include <stdexcept>
#include <vector>

// Standard C includes
#include <cmath>
#include <cstdint>
//----------------------------------------------------------------------------
// Typedefines
//----------------------------------------------------------------------------
namespace BoBRobotics {
namespace GeNNUtils {
//----------------------------------------------------------------------------
// Functions
//----------------------------------------------------------------------------
// Evaluates continued fraction for incomplete beta function by modified Lentz's method
// Adopted from numerical recipes in C p227
double betacf(double a, double b, double x)
{
    const int maxIterations = 200;
    const double epsilon = 3.0E-7;
    const double fpMin = 1.0E-30;

    const double qab = a + b;
    const double qap = a + 1.0;
    const double  qam = a - 1.0;
    double c = 1.0;

    // First step of Lentz’s method.
    double d = 1.0 - qab * x / qap;
    if (fabs(d) < fpMin) {
        d = fpMin;
    }
    d = 1.0 / d;
    double h = d;
    int m;
    for(m = 1; m <= maxIterations; m++) {
        const double m2 = 2.0 * m;
        const double aa1 = m * (b - m) * x / ((qam + m2) * (a + m2));
        d = 1.0 + aa1 * d;

        // One step (the even one) of the recurrence.
        if(fabs(d) < fpMin)  {
            d = fpMin;
        }
        c = 1.0 + aa1 / c;
        if(fabs(c) < fpMin) {
            c=fpMin;
        }
        d = 1.0 / d;
        h *= d * c;
        const double aa2 = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2));
        d = 1.0 + aa2 * d;

        // Next step of the recurrence (the odd one).
        if (fabs(d) < fpMin) {
            d = fpMin;
        }
        c = 1.0 + aa2 / c;
        if (fabs(c) < fpMin)  {
            c = fpMin;
        }
        d = 1.0 / d;
        const double del = d * c;
        h *= del;

        // Are we done?
        if (fabs(del - 1.0) < epsilon) {
            break;
        }
    }
    if (m > maxIterations) {
        throw std::runtime_error("a or b too big, or MAXIT too small in betacf");
    }
    return h;
}
//----------------------------------------------------------------------------
// Returns the incomplete beta function Ix(a, b)
// Adopted from numerical recipes in C p227
double betai(double a, double b, double x)
{
    if (x < 0.0 || x > 1.0) {
        throw std::runtime_error("Bad x in routine betai");
    }

    // Factors in front of the continued fraction.
    double bt;
    if (x == 0.0 || x == 1.0) {
        bt = 0.0;
    }
    else {
        bt = exp(lgamma(a + b) - lgamma(a) - lgamma(b) + a * log(x) + b * log(1.0 - x));
    }

    // Use continued fraction directly.
    if (x < ((a + 1.0) / (a + b + 2.0))) {
        return bt * betacf(a, b, x) / a;
    }
    // Otherwise, use continued fraction after making the
    // symmetry transformation.
    else {
        return 1.0 - (bt * betacf(b, a, 1.0 - x) / b);
    }
}
//----------------------------------------------------------------------------
// Evaluates inverse CDF of binomial distribution
inline unsigned int binomialInverseCDF(double cdf, unsigned int n, double p)
{
    if(cdf < 0.0 || 1.0 < cdf) {
        throw std::runtime_error("binomialInverseCDF error - CDF < 0 or 1 < CDF");
    }

    // Loop through ks <= n
    for (unsigned int k = 0; k <= n; k++)
    {
        // Use incomplete beta function to evalauate CDF, if it's greater than desired CDF value, return k
        if (betai(n - k, 1 + k, 1.0 - p) > cdf) {
            return k;
        }

    }

    throw std::runtime_error("Invalid CDF parameterse");
}
//----------------------------------------------------------------------------
template<typename IndexType>
void sortRows(unsigned int numPre, unsigned int *rowLength, IndexType *ind, unsigned int maxRowLength)
{
    // Loop through rows and sort indices
    for(unsigned int i = 0; i < numPre; i++) {
        IndexType *indexStart = &ind[i * maxRowLength];
        IndexType *indexEnd = indexStart + rowLength[i];

        std::sort(indexStart, indexEnd);
    }
}
//----------------------------------------------------------------------------
template<typename T>
void printDenseMatrix(unsigned int numPre, unsigned int numPost, T *weights)
{
    for(unsigned int i = 0; i < numPre; i++) {
        for(unsigned int j = 0; j < numPost; j++) {
            LOGI << *(weights++) << ",";
        }
        LOGI << std::endl;
    }
}
//----------------------------------------------------------------------------
template<typename IndexType>
void printRaggedMatrix(unsigned int numPre, const unsigned int *rowLength, const IndexType *ind, unsigned int maxRowLength)
{
    for(unsigned int i = 0; i < numPre; i++) {
        LOGI << i << ":";

        const IndexType *rowInd = &ind[i * maxRowLength];
        for(unsigned int j = 0; j < rowLength[i]; j++) {
            LOGI << rowInd[j] << ",";
        }

        LOGI << std::endl;
    }
}
//----------------------------------------------------------------------------
template <typename Generator, typename IndexType>
void buildFixedNumberPreConnector(unsigned int numPre, unsigned int numPost, unsigned int numConnections,
                                  unsigned int *rowLength, IndexType *ind, unsigned int maxRowLength,
                                  Generator &gen)
{
    // Zero row lengths
    std::fill_n(&rowLength[0], numPre, 0);

    // Generate array of presynaptic indices
    std::vector<unsigned int> preIndices(numPre);
    std::iota(preIndices.begin(), preIndices.end(), 0);

    // Loop through postsynaptic neurons
    for(unsigned int j = 0; j < numPost; j++) {
        // Loop through connections to make
        for(unsigned int c = 1; c <= numConnections; c++) {
            // Create distribution to select from remaining available neurons
            std::uniform_int_distribution<> dis(0, numPre - c);

            // Pick a presynaptic neuron
            const unsigned int i = preIndices[dis(gen)];

            // Add synapse
            ind[(i * maxRowLength) + rowLength[i]++] = (IndexType)j;

            // Swap the last available preindex with the one we have now used
            std::swap(preIndices[i], preIndices[numPre - c]);
        }
    }

    // Sort rows so indices are increasing
    sortRows(numPre, rowLength, ind, maxRowLength);
}
//----------------------------------------------------------------------------
unsigned int calcFixedNumberPreConnectorMaxConnections(unsigned int numPre, unsigned int numPost, unsigned int numConnections)
{
    // Calculate suitable quantile for 0.9999 change when drawing numPre times
    const double quantile = pow(0.9999, 1.0 / (double)numPre);

    return binomialInverseCDF(quantile, numPost, (double)numConnections / (double)numPre);
}
} // GeNNUtils
} // BoBRobotics
