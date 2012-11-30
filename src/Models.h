/*
 * Models.h
 *
 *  Created on: 30 Nov 2012
 *      Author: kent
 */

#ifndef MODELS_H_
#define MODELS_H_

#include <cstdlib>

class Models {
private:


public:
	Models();
	virtual ~Models();

	static struct
	{
		double operator() (double mean, double std) 	//	Normal distributed N(mean, std)
														//	Marsaglia polar method
		{
			double u, v, w, x, s;

			do
			{
				do
				{
					u = 2 * (((double)rand() - ((double)RAND_MAX / 2)) / (double)RAND_MAX);
					v = 2 * (((double)rand() - ((double)RAND_MAX / 2)) / (double)RAND_MAX);
					s = pow(u, 2) + pow(v, 2);
					w = -2 * (log(s)/log(exp(1))) / s;

				} while(w < 0);				//	Prevent imaginary numbers

				x = u * sqrt(w);

			} while (fabs(x) <= 4);			//	4 standard deviations as boundary

			return mean + std * (x / 4.0f);	//	Return values between -1 to 1
		}
	} gaussian;

};

#endif /* MODELS_H_ */
