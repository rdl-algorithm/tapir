#pragma once


#include <random>

namespace pushbox {

class TruncatedNormalDistribution {
public:
	TruncatedNormalDistribution(const double theMean = 0.0, const double stddev = 1.0 ): mean(theMean), lowerBound(mean-stddev), upperBound(mean+stddev), zeroStdDev(stddev==0), normalDistribution(mean, stddev) {}
	template<class URNG>
	double operator()(URNG& g) {
		if (zeroStdDev) return lowerBound;
		while (true) {
			double result = normalDistribution(g);
			if ( (result >= lowerBound) && (result <= upperBound) ) {
				return result;
			}
		}
	}
	bool hasZeroStdDev() const { return zeroStdDev; }
	double getMean() const { return mean; }
private:
	double mean;
	double lowerBound;
	double upperBound;
	bool zeroStdDev;
	std::normal_distribution<double> normalDistribution;
};


}
