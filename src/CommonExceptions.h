#ifndef CALIBRATIONEXCEPTIONS_H_
#define CALIBRATIONEXCEPTIONS_H_

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
        return "Proces przerwany przez uzytkownika";
    }
};

class FramesAmountMatchError : public std::exception
{
public:
    virtual const char* what() const noexcept
    {
        return "Liczba klatek na urzadzeniach nie zgadza sie";
    }
};

#endif /* CALIBRATIONEXCEPTIONS_H_ */
