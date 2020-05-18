#ifndef NODO_H
#define NODO_H

#include <math.h>

class Nodo
{
    // current position
    int xPos;
    int yPos;
    // total distance already travelled to reach the node
    int level;
    // priority=level+remaining distance estimate
    int priority;  // smaller: higher priority

    public:
        Nodo();
        Nodo(int xp, int yp, int d, int p);

        int getxPos() const;
        int getyPos() const;
        int getLevel() const {return level;}
        int getPriority() const;

        void updatePriority(const int & xDest, const int & yDest);

        // give better priority to going strait instead of diagonally
        void nextLevel(const int & i, const int &dir);

        // Estimation function for the remaining distance to the goal.
        const int & estimate(const int & xDest, const int & yDest) const;
        int getXPos() const;
        void setXPos(int value);
        int getYPos() const;
        void setYPos(int value);
};
#endif // NODO_H
