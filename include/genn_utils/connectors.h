#pragma once

// BoB robotics includes
#include "common/assert.h"

// Standard C++ includes
#include <algorithm>
#include <iostream>
#include <numeric>
#include <random>
#include <stdexcept>
#include <vector>

// Standard C includes
#include <cmath>
#include <cstdint>

// GeNN includes
#include "sparseProjection.h"
#include "utils.h"

//----------------------------------------------------------------------------
// Typedefines
//----------------------------------------------------------------------------
namespace BoBRobotics {
namespace GeNNUtils {
typedef void (*AllocateFn)(unsigned int);

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
inline void addSynapseToSparseProjection(unsigned int i, unsigned int j, unsigned int numPre,
                                         SparseProjection &sparseProjection)
{
    // Get index of current end of row in sparse projection
    const unsigned int rowEndIndex = sparseProjection.indInG[i + 1];

    // Also get index of last synapse
    const unsigned int lastSynapseIndex = sparseProjection.indInG[numPre];

    // If these aren't the same (there are existing synapses after this one), shuffle up the indices
    if(rowEndIndex != lastSynapseIndex) {
        std::move_backward(&sparseProjection.ind[rowEndIndex], &sparseProjection.ind[lastSynapseIndex],
                           &sparseProjection.ind[lastSynapseIndex + 1]);
    }

    // Insert new synapse
    sparseProjection.ind[rowEndIndex] = j;

    // Increment all subsequent indices
    std::transform(&sparseProjection.indInG[i + 1], &sparseProjection.indInG[numPre + 1], &sparseProjection.indInG[i + 1],
                   [](unsigned int index)
                   {
                       return (index + 1);
                   });
}
//----------------------------------------------------------------------------
void sortRows(unsigned int numPre, SparseProjection &sparseProjection)
{
    // Loop through rows and sort indices
    for(unsigned int i = 0; i < numPre; i++) {
        const unsigned int rowStartIndex = sparseProjection.indInG[i];
        const unsigned int rowEndIndex = sparseProjection.indInG[i + 1];
        std::sort(&sparseProjection.ind[rowStartIndex], &sparseProjection.ind[rowEndIndex]);
    }
}
//----------------------------------------------------------------------------
template<typename IndexType>
void sortRows(unsigned int numPre, RaggedProjection<IndexType> &sparseProjection)
{
    // Loop through rows and sort indices
    for(unsigned int i = 0; i < numPre; i++) {
        IndexType *indexStart = &sparseProjection.ind[(i * sparseProjection.maxRowLength)];
        IndexType *indexEnd = indexStart + sparseProjection.rowLength[i];

        std::sort(indexStart, indexEnd);
    }
}
//----------------------------------------------------------------------------
template<typename T>
void printDenseMatrix(unsigned int numPre, unsigned int numPost, T *weights)
{
    for(unsigned int i = 0; i < numPre; i++) {
        for(unsigned int j = 0; j < numPost; j++) {
            std::cout << *(weights++) << ",";
        }
        std::cout << std::endl;
    }
}
//----------------------------------------------------------------------------
void printSparseMatrix(unsigned int numPre, const SparseProjection &projection)
{
    for(unsigned int i = 0; i < numPre; i++)
    {
        std::cout << i << ":";

        for(unsigned int j = projection.indInG[i]; j < projection.indInG[i + 1]; j++)
        {
            std::cout << projection.ind[j] << ",";
        }

        std::cout << std::endl;
    }
}
//----------------------------------------------------------------------------
void buildOneToOneConnector(unsigned int numPre, unsigned int numPost,
                            SparseProjection &sparseProjection, AllocateFn allocateFn)
{
    if(numPre != numPost) {
        throw std::runtime_error("One-to-one connector can only be used between two populations of the same size");
    }

    // Allocate SparseProjection arrays
    allocateFn(numPre);

    // Configure synaptic rows
    for(unsigned int i = 0; i < numPre; i++)
    {
        sparseProjection.indInG[i] = i;
        sparseProjection.ind[i] = i;
    }
    sparseProjection.indInG[numPre] = numPre;
}
//----------------------------------------------------------------------------
template <typename Generator>
void buildFixedProbabilityConnector(unsigned int numPre, unsigned int numPost, float probability,
                                    SparseProjection &projection, AllocateFn allocate, Generator &gen)
{
    // Allocate memory for indices
    // **NOTE** RESIZE as this vector is populated by index
    std::vector<unsigned int> tempIndInG;
    tempIndInG.resize(numPre + 1);

    // Reserve a temporary vector to store indices
    std::vector<unsigned int> tempInd;
    tempInd.reserve((unsigned int)((float)(numPre * numPost) * probability));

    // Create RNG to draw probabilities
    std::uniform_real_distribution<> dis(0.0, 1.0);

    // Loop through pre neurons
    for(unsigned int i = 0; i < numPre; i++)
    {
        // Connections from this neuron start at current end of indices
        tempIndInG[i] = tempInd.size();

        // Loop through post neurons
        for(unsigned int j = 0; j < numPost; j++)
        {
            // If there should be a connection here, add one to temporary array
            if(dis(gen) < probability)
            {
                tempInd.push_back(j);
            }
        }
    }

    // Add final index
    tempIndInG[numPre] = tempInd.size();

    // Allocate SparseProjection arrays
    // **NOTE** shouldn't do directly as underneath it may use CUDA or host functions
    allocate(tempInd.size());

    // Copy indices
    std::copy(tempIndInG.begin(), tempIndInG.end(), &projection.indInG[0]);
    std::copy(tempInd.begin(), tempInd.end(), &projection.ind[0]);
}
//----------------------------------------------------------------------------
template <typename Generator, typename IndexType>
void buildFixedProbabilityConnector(unsigned int numPre, unsigned int numPost, float probability,
                                    RaggedProjection<IndexType> &projection, Generator &gen)
{
    const double probabilityReciprocal = 1.0 / std::log(1.0f - probability);

    // Create RNG to draw probabilities
    std::uniform_real_distribution<> dis(0.0, 1.0);

    // Zero row lengths
    std::fill_n(projection.rowLength, numPre, 0);

    // Loop through potential synapses
    const int64_t maxConnections = (int64_t)numPre * (int64_t)numPost;
    for(int64_t s = -1;;) {
        // Skip to next connection
        s += (1 + (int64_t)(std::log(dis(gen)) * probabilityReciprocal));

        // If we haven't skipped past end of matrix
        if(s < maxConnections) {
            // Convert synapse number to pre and post index
            const auto prePost = std::div(s, numPost);

            // Get pointer to start of this presynaptic neuron's connection row
            IndexType *rowIndices = &projection.ind[prePost.quot * projection.maxRowLength];

            // Add synapse
            rowIndices[projection.rowLength[prePost.quot]++] = prePost.rem;
            BOB_ASSERT(projection.rowLength[prePost.quot] <= projection.maxRowLength);
        }
        else {
            break;
        }
    }
}
//----------------------------------------------------------------------------
template <typename Generator>
void buildFixedProbabilityConnector(unsigned int numPre, unsigned int numPost, float probability,
                                    uint32_t *bitfield, Generator &gen)
{
    // **THINK** I'm unsure about this calculation but it's used to allocate the memory and copy it!
    const size_t numWords = ((size_t)numPre * (size_t)numPost) / 32 + 1;

    const double probabilityReciprocal = 1.0 / std::log(1.0f - probability);

    // Create RNG to draw probabilities
    std::uniform_real_distribution<> dis(0.0, 1.0);

    // Zero bitfield
    std::fill_n(bitfield, numWords, 0);

    // Loop through potential synapses
    const int64_t maxConnections = (int64_t)numPre * (int64_t)numPost;
    for(int64_t s = -1;; s++) {
        // Skip to next connection
        s += (1 + (int64_t)(std::log(dis(gen)) * probabilityReciprocal));

        // If we haven't skipped past end of matrix
        if(s < maxConnections) {
            // Convert synapse number to pre and post index
            const auto prePost = std::div(s, numPost);

            const size_t gid = (prePost.quot * (size_t)numPost) + prePost.rem;
            setB(bitfield[gid / 32], gid % 32);
        }
        else {
            break;
        }
    }
}
//----------------------------------------------------------------------------
unsigned int calcFixedProbabilityConnectorMaxConnections(unsigned int numPre, unsigned int numPost, double probability)
{
    // Calculate suitable quantile for 0.9999 change when drawing numPre times
    const double quantile = pow(0.9999, 1.0 / (double)numPre);

    return binomialInverseCDF(quantile, numPost, probability);
}
//----------------------------------------------------------------------------
unsigned int calcFixedProbabilityConnectorMaxSourceConnections(unsigned int numPre, unsigned int numPost, double probability)
{
    // Calculate suitable quantile for 0.9999 change when drawing numPost times
    const double quantile = pow(0.9999, 1.0 / (double)numPost);

    return binomialInverseCDF(quantile, numPre, probability);
}
//----------------------------------------------------------------------------
template <typename Generator>
void buildFixedNumberPreConnector(unsigned int numPre, unsigned int numPost, unsigned int numConnections,
                                  SparseProjection &projection, AllocateFn allocate, Generator &gen)
{
    // Allocate sparse projection
    allocate(numPost * numConnections);

    // Zero all indInG
    std::fill(&projection.indInG[0], &projection.indInG[numPre + 1], 0);

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

            // Add synapse to projection
            addSynapseToSparseProjection(i, j, numPre, projection);

            // Swap the last available preindex with the one we have now used
            std::swap(preIndices[i], preIndices[numPre - c]);
        }
    }

    // Sort rows so indices are increasing
    sortRows(numPre, projection);

    // Check correct number of connections were added
    BOB_ASSERT(projection.indInG[numPre] == projection.connN);
}
//----------------------------------------------------------------------------
unsigned int calcFixedNumberPreConnectorMaxConnections(unsigned int numPre, unsigned int numPost, unsigned int numConnections)
{
    // Calculate suitable quantile for 0.9999 change when drawing numPre times
    const double quantile = pow(0.9999, 1.0 / (double)numPre);

    return binomialInverseCDF(quantile, numPost, (double)numConnections / (double)numPre);
}
//----------------------------------------------------------------------------
template <typename Generator>
void buildFixedNumberTotalWithReplacementConnector(unsigned int numPre, unsigned int numPost, unsigned int numConnections,
                                                   SparseProjection &projection, AllocateFn allocate, Generator &gen)
{
    // Allocate sparse projection
    allocate(numConnections);

    // Initially populate indInG with row lengths
    // **NOTE** we are STARTING at the 2nd row because the first row always starts at zero and
    // FINISHING at second from last row because all remaining connections must go in last row
    unsigned int remainingConnections = numConnections;
    unsigned int matrixSize = numPre * numPost;
    std::generate_n(&projection.indInG[1], numPre - 1,
                    [&remainingConnections, &matrixSize, numPost, &gen]()
                    {
                        const double probability = (double)numPost / (double)matrixSize;

                        // Create distribution to sample row length
                        std::binomial_distribution<unsigned int> rowLengthDist(remainingConnections, probability);

                        // Sample row length;
                        const unsigned int rowLength = rowLengthDist(gen);

                        // Update counters
                        remainingConnections -= rowLength;
                        matrixSize -= numPost;

                        return rowLength;
                    });

    // Insert remaining connections into last row
    projection.indInG[numPre] = remainingConnections;

    // Compute the partial sum so indG now includes offsets
    std::partial_sum(&projection.indInG[1], &projection.indInG[numPre + 1], &projection.indInG[1]);

    // Insert zero for first row's starting offset
    projection.indInG[0] = 0;

    // Create distribution to sample row length
    // **NOTE** these distributions operate on a CLOSED interval hence -1
    std::uniform_int_distribution<unsigned int> postsynapticNeuronDist(0, numPost - 1);

    // Loop through rows
    for(unsigned int i = 0; i < numPre; i++) {
        // Get indices of row start and end
        const unsigned int rowBegin = projection.indInG[i];
        const unsigned int rowEnd = projection.indInG[i + 1];

        // Pick a random postsynaptic neuron to connect each one
        for(unsigned int j = rowBegin; j < rowEnd; j++) {
            projection.ind[j] = postsynapticNeuronDist(gen);
        }
    }

     // Sort rows so indices are increasing
    sortRows(numPre, projection);

    // Check structure is valid
    BOB_ASSERT(projection.indInG[numPre] == numConnections);
}
//----------------------------------------------------------------------------
template <typename Generator>
void buildFixedNumberTotalWithReplacementConnector(unsigned int numPre, unsigned int numPost, size_t numConnections,
                                                   uint32_t *bitfield, Generator &gen)
{
    // Calculate row lengths
    // **NOTE** we are FINISHING at second from last row because all remaining connections must go in last row
    size_t remainingConnections = numConnections;
    size_t matrixSize = (size_t)numPre * (size_t)numPost;
    std::vector<unsigned int> rowLengths(numPre);
    std::generate_n(rowLengths.begin(), numPre - 1,
                    [&remainingConnections, &matrixSize, numPost, &gen]()
                    {
                        const double probability = (double)numPost / (double)matrixSize;

                        // Create distribution to sample row length
                        std::binomial_distribution<size_t> rowLengthDist(remainingConnections, probability);

                        // Sample row length;
                        const size_t rowLength = rowLengthDist(gen);

                        // Update counters
                        remainingConnections -= rowLength;
                        matrixSize -= numPost;

                        return (unsigned int)rowLength;
                    });
    // Insert remaining connections into last row
    rowLengths.back() = (unsigned int)remainingConnections;

    // Create distribution to sample row length
    // **NOTE** these distributions operate on a CLOSED interval hence -1
    std::uniform_int_distribution<size_t> postsynapticNeuronDist(0, numPost - 1);

    // Zero bitfield
    std::fill_n(bitfield, ((size_t)numPre * (size_t)numPost) / 32 + 1, 0);

    // Loop through rows
    for(unsigned int i = 0; i < numPre; i++) {
        // Loop through synapses in row
        for(unsigned int j = 0; j < rowLengths[i]; j++) {
            // Set random bit in this row
            const size_t gid = ((size_t)i * (size_t)numPost) + postsynapticNeuronDist(gen);
            bitfield[gid / 32] |= (1 << (gid % 32));
        }
    }
}
//----------------------------------------------------------------------------
template <typename Generator, typename IndexType>
void buildFixedNumberTotalWithReplacementConnector(unsigned int numPre, unsigned int numPost, size_t numConnections,
                                                   RaggedProjection<IndexType> &projection, Generator &gen)
{
    // Calculate row lengths
    // **NOTE** we are FINISHING at second from last row because all remaining connections must go in last row
    size_t remainingConnections = numConnections;
    size_t matrixSize = (size_t)numPre * (size_t)numPost;
    std::generate_n(&projection.rowLength[0], numPre - 1,
                    [&remainingConnections, &matrixSize, numPost, &gen]()
                    {
                        const double probability = (double)numPost / (double)matrixSize;

                        // Create distribution to sample row length
                        std::binomial_distribution<size_t> rowLengthDist(remainingConnections, probability);

                        // Sample row length;
                        const size_t rowLength = rowLengthDist(gen);

                        // Update counters
                        remainingConnections -= rowLength;
                        matrixSize -= numPost;

                        return (unsigned int)rowLength;
                    });

    // Insert remaining connections into last row
    projection.rowLength[numPre - 1] = (unsigned int)remainingConnections;

    // Create distribution to sample row length
    // **NOTE** these distributions operate on a CLOSED interval hence -1
    std::uniform_int_distribution<unsigned int> postsynapticNeuronDist(0, numPost - 1);

    // Loop through rows
    for(unsigned int i = 0; i < numPre; i++) {
        // Get pointer to first index of row
        IndexType *index = &projection.ind[(i * projection.maxRowLength)];

        if(projection.rowLength[i] > projection.maxRowLength) {
            printf("Row length %u greate than max %u\n", projection.rowLength[i], projection.maxRowLength);
            BOB_ASSERT(false);
        }
        //BOB_ASSERT(projection.rowLength[i] <= projection.maxRowLength);
        
        // Pick a random postsynaptic neuron to connect each one
        for(unsigned int j = 0; j < projection.rowLength[i]; j++) {
            index[j] = postsynapticNeuronDist(gen);
        }
    }

     // Sort rows so indices are increasing
    sortRows(numPre, projection);
}
//----------------------------------------------------------------------------
unsigned int calcFixedNumberTotalWithReplacementConnectorMaxConnections(unsigned int numPre, unsigned int numPost, unsigned int numConnections)
{
    // Calculate suitable quantile for 0.9999 change when drawing numPre times
    const double quantile = pow(0.9999, 1.0 / (double)numPre);

    // There are numConnections connections amongst the numPre*numPost possible connections.
    // Each of the numConnections connections has an independent p=float(numPost)/(numPre*numPost)
    // probability of being selected, and the number of synapses in the sub-row is binomially distributed
    return binomialInverseCDF(quantile, numConnections, (double)numPost / ((double)numPre * (double)numPost));
}
//----------------------------------------------------------------------------
unsigned int calcFixedNumberTotalWithReplacementConnectorMaxSourceConnections(unsigned int numPre, unsigned int numPost, unsigned int numConnections)
{
    // Calculate suitable quantile for 0.9999 change when drawing numPre times
    const double quantile = pow(0.9999, 1.0 / (double)numPost);

    // There are numConnections connections amongst the numPre*numPost possible connections.
    // Each of the numConnections connections has an independent p=float(numPost)/(numPre*numPost)
    // probability of being selected, and the number of synapses in the sub-row is binomially distributed
    return binomialInverseCDF(quantile, numConnections, (double)numPre / ((double)numPre * (double)numPost));
}
} // GeNNUtils
} // BoBRobotics