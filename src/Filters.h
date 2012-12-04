/*
 * Filters.h
 *
 *  Created on: 4 Dec 2012
 *      Author: kent
 */

#ifndef FILTERS_H_
#define FILTERS_H_

namespace Models
{
	class Filters
	{
	public:
		Filters();
		virtual ~Filters();

		static struct
		{
			double operator() (double x, double y) 												//	2D Gaussian filter
			{
				return this->operator ()(x, y, 1, 1, 1);
			}

			double operator() (double x, double y, double stdX, double stdY) 					//	2D Gaussian filter
			{
				return this->operator ()(x, y, stdX, stdY, 1);
			}

			double operator() (double x, double y, double stdX, double stdY, double amplitude) 	//	2D Gaussian filter
			{
				return amplitude * exp(-(1.0/2 * (pow(x / stdX, 2) + pow(y / stdY, 2))));
			}
		} gaussianFilter2D;

	};
} /* namespace Models */
#endif /* FILTERS_H_ */
