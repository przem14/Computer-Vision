#ifndef CALIBRATIONEXCEPTIONS_H_
#define CALIBRATIONEXCEPTIONS_H_

#ifndef _MSC_VER
#define noexcept noexcept
#else
#define noexcept
#endif /* _MSC_VER */

#include <exception>

class ImageReadError : public std::exception
{
public:
    virtual const char* what() const noexcept 
    {
        return "Nie udalo sie wczytac obrazu";
    }
};

class InterruptedByUser : public std::exception
{
public:
    virtual const char* what() const noexcept
    {
        return "Kalibracja przerwana przez uzytkownika";
    }
};

#endif /* CALIBRATIONEXCEPTIONS_H_ */
