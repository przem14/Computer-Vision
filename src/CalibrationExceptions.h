/*
 * CalibrationExceptions.h
 *
 *  Created on: 31-10-2014
 *      Author: Bartosz Lagwa
 */

#ifndef CALIBRATIONEXCEPTIONS_H_
#define CALIBRATIONEXCEPTIONS_H_

#include <exception>

class ImageReadError : public std::exception
{
	virtual const char* what() const noexcept {return "Nie udalo sie wczytac obrazu";};
};


#endif /* CALIBRATIONEXCEPTIONS_H_ */
