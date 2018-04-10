#include "peak_identification.h"

#define DEBUG       0           // Prints additional outputs if 1

using namespace std;


VectorStats::VectorStats(vec_iter_ld start, vec_iter_ld end) {
    this->start = start;
    this->end = end;
    this->compute();
}

void VectorStats::compute() {
    ld sum = std::accumulate(start, end, 0.0);
    uint slice_size = std::distance(start, end);
    ld mean = sum / slice_size;
    std::vector<ld> diff(slice_size);
    std::transform(start, end, diff.begin(), [mean](ld x) { return x - mean; });
    ld sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    ld std_dev = std::sqrt(sq_sum / slice_size);
    
    this->m1 = mean;
    this->m2 = std_dev;
}

unordered_map<string, vector<ld>> z_score_thresholding(vector<ld> input, int lag, ld threshold, ld influence) {
    unordered_map<string, vector<ld>> output;

    if(DEBUG) cout << "THE Z SCORE INPUT SIZE IS " << input.size() << endl;

    uint n = (uint) input.size();
    vector<ld> signals(input.size());
    vector<ld> residuals(input.size());
    vector<ld> filtered_input(input.begin(), input.end());
    vector<ld> filtered_mean(input.size());
    vector<ld> filtered_stddev(input.size());

    if(DEBUG) cout << "BEFORE COMPUTATION, SIGNALS IS " << signals << endl;

    VectorStats lag_subvector_stats(input.begin(), input.begin() + lag);
    filtered_mean[lag - 1] = lag_subvector_stats.mean();
    filtered_stddev[lag - 1] = lag_subvector_stats.standard_deviation();

    if(DEBUG) cout << "BEFORE COMPUTATION, MEAN IS " << filtered_mean << endl;
    if(DEBUG) cout << "BEFORE COMPUTATION, STD DEV IS " << filtered_stddev << endl;

    for (int i = lag; i < n; i++) {
        if (abs(input[i] - filtered_mean[i - 1]) > threshold * filtered_stddev[i - 1]) {
            signals[i] = (input[i] > filtered_mean[i - 1]) ? 1.0 : -1.0;
            filtered_input[i] = influence * input[i] + (1 - influence) * filtered_input[i - 1];
        } else {
            signals[i] = 0.0;
            filtered_input[i] = input[i];
        }
        // Computing the residual which is used later for identifying the biggest peak
        residuals[i] = abs(input[i] - filtered_mean[i - 1]) - threshold * filtered_stddev[i - 1];

        if(DEBUG) cout << "CURRENT SIGNALS IS " << signals[i] << endl;

        VectorStats lag_subvector_stats(filtered_input.begin() + (i - lag), filtered_input.begin() + i);
        filtered_mean[i] = lag_subvector_stats.mean();
        filtered_stddev[i] = lag_subvector_stats.standard_deviation();
    }

    if(DEBUG) cout << "THE TOTAL SIGNALS VECTOR IS " << signals << endl;

    output["output_signals"] = signals;
    output["output_residuals"] = residuals;
    output["filtered_mean"] = filtered_mean;
    output["filtered_stddev"] = filtered_stddev;

    if(DEBUG) cout << "THE Z SCORE OUPUT SIGNAL IS " << output["output_signals"] << endl;
    if(DEBUG) cout << "THE Z SCORE OUPUT RESIDUAL IS " << output["output_residuals"] << endl;
    if(DEBUG) cout << "THE Z SCORE OUPUT MEAN IS " << output["filtered_mean"] << endl;
    if(DEBUG) cout << "THE Z SCORE OUPUT STD DEV IS " << output["filtered_stddev"] << endl;

    return output;
};