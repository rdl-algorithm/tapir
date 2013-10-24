#ifndef RANDGEN_H
#define RANDGEN_H

class RandGen {
	public:
		RandGen() {}
		~RandGen() {}

		void ranf_array(double aa[], int n);
		void ranf_start(long seed);
		double ranf_arr_cycle();
		double ranf_arr_next();
};
#endif
