#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <cmath>
#include <iterator>
#include <numeric>

typedef long double ld;
typedef unsigned int uint;
typedef std::vector<ld>::iterator vec_iter_ld;

/*
Overriding the ostream operator for pretty printing vectors.
 */
template<typename T>
std::ostream &operator<<(std::ostream &os, std::vector<T> vec) {
    os << "[";
    if (vec.size() != 0) {
        std::copy(vec.begin(), vec.end() - 1, std::ostream_iterator<T>(os, " "));
        os << vec.back();
    }
    os << "]";
    return os;
}

/**
 * This class calculates mean and standard deviation of a subvector.
 * This is basically stats computation of a subvector of a window size qual to "lag".
 */
class VectorStats {
public:
    /**
     * Constructor for VectorStats class.
     *
     * @param start - This is the iterator position of the start of the window,
     * @param end   - This is the iterator position of the end of the window,
     */
    VectorStats(vec_iter_ld start, vec_iter_ld end);

    /**
     * This method calculates the mean and standard deviation using STL function.
     * This is the Two-Pass implementation of the Mean & Variance calculation.
     */
    void compute();

    ld mean() {
        return m1;
    }

    ld standard_deviation() {
        return m2;
    }

private:
    vec_iter_ld start;
    vec_iter_ld end;
    ld m1;
    ld m2;
};

/**
 * This is the implementation of the Smoothed Z-Score Algorithm.
 *
 * @param input - input signal
 * @param lag - the lag of the moving window
 * @param threshold - the z-score at which the algorithm signals
 * @param influence - the influence (between 0 and 1) of new signals on the mean and standard deviation
 * @return a hashmap containing the filtered signal and corresponding mean and standard deviation.
 */
std::unordered_map<std::string, std::vector<ld>> z_score_thresholding(std::vector<ld> input, int lag, ld threshold, ld influence);


