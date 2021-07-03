#ifndef PCPG_H_
#define PCPG_H_

#include "../ann-framework/ann.h"


class PCPG : public ANN {
public:

    PCPG();
    void updateOutputs();
private:
	std::vector<double> set;
	std::vector<double> countup;
	std::vector<double> countdown;

	std::vector<double> pcpg_step;
				std::vector<double> setold;
				std::vector<double> countupold;
				std::vector<double> countdownold;
				std::vector<double> diffset ;
				std::vector<double> deltaxdown;
				std::vector<double> deltaxup;
				std::vector<double> xup;
				std::vector<double> xdown;

				std::vector<double> yup;
				std::vector<double> ydown;
				std::vector<double> pcpg_output;
};


#endif /* PCPG_H_ */
