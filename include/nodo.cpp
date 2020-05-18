#include "nodo.h"


int Nodo::getXPos() const
{
    return xPos;
}

void Nodo::setXPos(int value)
{
    xPos = value;
}

int Nodo::getYPos() const
{
    return yPos;
}

void Nodo::setYPos(int value)
{
    yPos = value;
}
Nodo::Nodo()
{
}


Nodo::Nodo(int xp, int yp, int d, int p)
{xPos=xp; yPos=yp; level=d; priority=p;}

int Nodo::getxPos() const {return xPos;}

int Nodo::getyPos() const {return yPos;}

int Nodo::getPriority() const {return priority;}

void Nodo::updatePriority(const int &xDest, const int &yDest)
{
    priority=level+estimate(xDest, yDest)*10; //A*
}

void Nodo::nextLevel(const int &i, const int &dir) // i: direction
{
    level+=(dir==8?(i%2==0?10:14):10);
}

const int &Nodo::estimate(const int &xDest, const int &yDest) const
{
    static int xd, yd, d;
    xd=xDest-xPos;
    yd=yDest-yPos;

    // Euclidian Distance
    d=static_cast<int>(sqrt(xd*xd+yd*yd));

    // Manhattan distance
    //d=abs(xd)+abs(yd);

    // Chebyshev distance
    //d=max(abs(xd), abs(yd));

    return(d);
}
