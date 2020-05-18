#include "ros/ros.h"
#include "planeamiento/apunto.h"
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <stdio.h>
#include <string>
#include <sstream>
//#include "nodo.h"

using namespace std;


int n=10; // horizontal size of the map
int m=10; // vertical size size of the map
//static int map[n][m];
int **map = NULL;
int **closed_nodes_map = NULL;
int **open_nodes_map = NULL;
int **dir_map = NULL;
//static int closed_nodes_map[n][m]; // map of closed (tried-out) nodes
//static int open_nodes_map[n][m]; // map of open (not-yet-tried) nodes
//static int dir_map[n][m]; // map of directions
const int dir=8; // number of possible directions to go at any position
// if dir==4
//static int dx[dir]={1, 0, -1, 0};
//static int dy[dir]={0, 1, 0, -1};
// if dir==8
static int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
static int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};


/*--------------------------------------------------------*/
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
        //void nextLevel(const int & i, const int &dir, Nodo nod);

        // Estimation function for the remaining distance to the goal.
        const int & estimate(const int & xDest, const int & yDest) const;
        int getXPos() const;
        void setXPos(int value);
        int getYPos() const;
        void setYPos(int value);
};



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

/*void Nodo::nextLevel(const int &i, const int &dir, Nodo nod) // i: direction
{
	if ((i==1)&&(::map[nod.getXPos()][nod.getYPos()-1] == 1)){
		level+=500;
		cout << "Esto supongo que es justo el caso que me implica" << endl;
	}
    level+=(dir==8?(i%2==0?10:14):10);
}*/

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
/*--------------------------------------------------------*/






// Determine priority (in the priority queue)
bool operator<(const Nodo & a, const Nodo & b)
{
  return a.getPriority() > b.getPriority();
}

// A-star algorithm.
// The route returned is a string of direction digits.
string pathFind( const int & xStart, const int & yStart,
                 const int & xFinish, const int & yFinish )
{
    static priority_queue<Nodo> pq[2]; // list of open (not-yet-tried) nodes
    static int pqi; // pq index
    static Nodo* n0;
    static Nodo* m0;
    static int i, j, x, y, xdx, ydy;
    static char c;
    pqi=0;

    // reset the node maps
    for(y=0;y<m;y++)
    {
        for(x=0;x<n;x++)
        {
            closed_nodes_map[x][y]=0;
            open_nodes_map[x][y]=0;
        }
    }
    // create the start node and push into list of open nodes
    n0=new Nodo(xStart, yStart, 0, 0);
    n0->updatePriority(xFinish, yFinish);
    pq[pqi].push(*n0);
    //open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes map
    open_nodes_map[n0->getxPos()][n0->getyPos()] = n0->getPriority();
    // A* search
    while(!pq[pqi].empty())
    {
        // get the current node w/ the highest priority
        // from the list of open nodes
        n0=new Nodo( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
                     pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

        x=n0->getxPos(); y=n0->getyPos();

        pq[pqi].pop(); // remove the node from the open list
        open_nodes_map[x][y]=0;
        // mark it on the closed nodes map
        closed_nodes_map[x][y]=1;

        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if(x==xFinish && y==yFinish)
        {
            // generate the path from finish to start
            // by following the directions
            string path="";
            while(!(x==xStart && y==yStart))
            {
                j=dir_map[x][y];
                c='0'+(j+dir/2)%dir;
                path=c+path;
                x+=dx[j];
                y+=dy[j];
            }

            // garbage collection
            delete n0;
            // empty the leftover nodes
            while(!pq[pqi].empty()) pq[pqi].pop();
            return path;
        }

        // generate moves (child nodes) in all possible directions
        for(i=0;i<dir;i++)
        {
            xdx=x+dx[i]; ydy=y+dy[i];

            if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || ::map[xdx][ydy]==1
                || closed_nodes_map[xdx][ydy]==1))
            {
                // generate a child node
                m0=new Nodo( xdx, ydy, n0->getLevel(),
                             n0->getPriority());
                m0->nextLevel(i,dir); //Este cambio es para intentar arreglar el problema del movimiento diagonal
                //m0->nextLevel(i,dir,*n0);
                m0->updatePriority(xFinish, yFinish);

                // if it is not in the open list then add into that
                if(open_nodes_map[xdx][ydy]==0)
                {
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    pq[pqi].push(*m0);
                    // mark its parent node direction
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                }
                else if(open_nodes_map[xdx][ydy]>m0->getPriority())
                {
                    // update the priority info
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    // update the parent direction info
                    dir_map[xdx][ydy]=(i+dir/2)%dir;

                    // replace the node
                    // by emptying one pq to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(pq[pqi].top().getxPos()==xdx &&
                           pq[pqi].top().getyPos()==ydy))
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pq[pqi].pop(); // remove the wanted node

                    // empty the larger size pq to the smaller one
                    if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    while(!pq[pqi].empty())
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pqi=1-pqi;
                    pq[pqi].push(*m0); // add the better node instead
                }
                else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }
    return ""; // no route found
}



/*-------------------------------------------------------
 * 
 * 
 *        EMPIEZA EL MAIN
 * 
 * -----------------------------------------------------*/


int main(int argc, char **argv)
{
  int Rx = 0,Ry = 0,Fx = 0, Fy = 0;
  ros::init(argc, argv, "plannerAstar2");
  cout << "Introduzca la posición actual del robot" << endl;
  cout << "Coordenada x? ";
  cin >> Rx;
  cout << ". Coordenada y? ";
  cin >> Ry;
  cout << endl << "Introduzca la posición deseada" << endl;
  cout << "Coordenada x? ";
  cin >> Fx;
  cout << ". Coordenada y? ";
  cin >> Fy;
  

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<planeamiento::apunto>("mover_en_direccion");
  planeamiento::apunto srv;
  
  /* Crear mapa */
  char obstacle = 24;

    n = 20;
    m = 13;

    ::map = new int*[n];
    closed_nodes_map = new int*[n];
    open_nodes_map = new int*[n];
    dir_map = new int*[n];

    for (int i = 0; i<n;i++){
        ::map[i] = new int[m];
        closed_nodes_map[i] = new int[m];
        open_nodes_map[i] = new int[m];
        dir_map[i] = new int[m];
    }


    // create empty map
    for(int y=0;y<m;y++)
    {
        for(int x=0;x<n;x++) ::map[x][y]=0;
    }
    
    //Poner obstáculos
    for (int x=0;x<n;x++){
		::map[x][0] = 1;
		::map[x][m-1] = 1;
	}
	for (int y=0;y<m;y++){
		::map[0][y] = 1;
		::map[n-1][y] = 1;
	}
	for (int i=0;i<10;i++){ //El índice de la i aumentará el grosor del obstáculo (mejor i<5)
		::map[i][4] = 1;
        ::map[i][3] = 1;
        ::map[i][5] = 1;
        ::map[10][4] = 1;	
	}
    for (int i=10;i<20;i++){ //El índice de la i aumentará el grosor del obstáculo (mejor i<5)
        ::map[i][8] = 1;    
    }

        ::map[6][8] = 1;    
        ::map[13][4] = 1;  
    
    /* Mapa 1 creado */
  
   /* Obtener camino */
    cout<<"Map Size (X,Y): "<<n<<","<<m<<endl;
    cout<<"Start: "<<Rx<<","<<Ry<<endl;
    cout<<"Finish: "<<Fx<<","<<Fy<<endl;
    Nodo nodoInicial;
    nodoInicial.setXPos(Rx);
    nodoInicial.setYPos(Ry);
    // get the route
    clock_t start = clock();
    string route=pathFind(Rx, Ry, Fx, Fy);
    if(route=="") cout<<"An empty route generated!"<<endl;
    clock_t end = clock();
    double time_elapsed = double(end - start);
    cout<<"Time to calculate the route (ms): "<<time_elapsed<<endl;
    cout<<"Route:"<<endl;
    cout<<route<<endl<<endl;

    // follow the route on the map and display it
    if(route.length()>0)
    {
        int j; char c;
        int x=Rx;
        int y=Ry;
        ::map[x][y]=2;
        for(int i=0;i<route.length();i++)
        {
            c =route.at(i);
            j=atoi(&c);
            x=x+dx[j];
            y=y+dy[j];
            ::map[x][y]=3;
        }
        ::map[x][y]=4;



        // display the map with the route
        for(int y=0;y<m;y++)
        {
            for(int x=0;x<n;x++)
                if(::map[x][y]==0)
                    cout<<".";
                else if(::map[x][y]==1)

                    cout<<obstacle; //obstacle
                else if(::map[x][y]==2)
                    cout<<"S"; //start
                else if(::map[x][y]==3)
                    cout<<"R"; //route
                else if(::map[x][y]==4)
                    cout<<"F"; //finish
            cout<<endl;
        }
    }
  
  
  /* Enviar al robot todas las instrucciones una por una */
  
  for (int i=0;i<route.length();i++){
        stringstream ss;
        ss << route[i];
        string direccion_string = ss.str();
        srv.request.direccion = atoi(direccion_string.c_str());
        cout << "Enviando instrucción " << atoi(direccion_string.c_str()) << endl;
        if (client.call(srv))
			{
				ROS_INFO("Sum: %ld", (long int)srv.response.resultado);
			}
			else
			{
				ROS_ERROR("Failed to call service mover_en_direccion");
				return 1;
			}
  }
  
  

/* Liberar memoria */
  for (int i = 0; i<n;i++){
        delete[] ::map[i];
        delete[] closed_nodes_map[i];
        delete[] open_nodes_map[i];
        delete[] dir_map[i];
    }
    delete[] ::map;
    delete[] closed_nodes_map;
    delete[] open_nodes_map;
    delete[] dir_map;
  return 0;
}
