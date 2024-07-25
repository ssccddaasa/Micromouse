#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <iostream>
#include <vector>
#include <algorithm>
#include "API.h"





using namespace std;
#define MAZE_SIZE 16
#define MAX_DISTNACE 16 * 16
int max_queue_size = 0;


class Diractions
{
public:
    static const int NORTH = 0;
    static const int EAST = 1;
    static const int SOUTH = 2;
    static const int WEST = 3;
};






struct Robot
{
    short int x;
    short int y;
    short int dirc;
};



struct Cell
{
    short int x;
    short int y;
};





int manhattan_distance[MAZE_SIZE][MAZE_SIZE] = {
    {14, 13, 12, 11, 10, 9, 8, 7, 7, 8, 9, 10, 11, 12, 13, 14},
    {13, 12, 11, 10, 9, 8, 7, 6, 6, 7, 8, 9, 10, 11, 12, 13},
    {12, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11, 12},
    {11, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10, 11},
    {10, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9, 10},
    {9, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8, 9},
    {8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8},
    {7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7},
    {7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7},
    {8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8},
    {9, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8, 9},
    {10, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9, 10},
    {11, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10, 11},
    {12, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11, 12},
    {13, 12, 11, 10, 9, 8, 7, 6, 6, 7, 8, 9, 10, 11, 12, 13},
    {14, 13, 12, 11, 10, 9, 8, 7, 7, 8, 9, 10, 11, 12, 13, 14}
};

// The locations of the walls in the maze
int vertical_walls[MAZE_SIZE - 1][MAZE_SIZE] = {0};
// The locations of the walls in the maze
int horizontal_walls[MAZE_SIZE][MAZE_SIZE - 1] = {0};




Cell getMinCell(vector<Cell> adj_cells);

vector<Cell> get_accessable_cells(short int x, short int y);

void change_deriction(Cell current_cell, Cell min_cell);

queue<Cell> flood_fill_queue;


Robot myRobot = {0, 0, Diractions::NORTH};




int distance(short int x, short int y) {
    return manhattan_distance[x][y];
}




int distance (Cell cell) {
    return manhattan_distance[cell.x][cell.y];
}



void printFloodFillQueue(queue<Cell> q) {
    cout << "Flood Fill Queue: ";
    while (!q.empty()) {
        Cell cell = q.front();
        q.pop();
        cout << "(" << cell.x << ", " << cell.y << ") ";
    }
    cout << endl;
}








void update_walls()
{
    int x = myRobot.x;
    int y = myRobot.y;
    int dirc = myRobot.dirc;


    if (dirc == Diractions::NORTH)
    {
        if (x > 0)
        {
            if (API::wallLeft())
            {
                vertical_walls[x - 1][y] = API::wallLeft();
                API::setWall(x, y, 'w');
            }
        }
        if (x < 15)
        {
            if (API::wallRight())
            {
                vertical_walls[x][y] = API::wallRight();
                API::setWall(x, y, 'e');
            }
        }
        if (y < 15)
        {
            if (API::wallFront())
            {
                horizontal_walls[x][y] = API::wallFront();
                API::setWall(x, y, 'n');
            }
        }
    }


    else if (dirc == Diractions::EAST)
    {

        if (x < 15)
        {
            if (API::wallFront())
            {
                vertical_walls[x][y] = API::wallFront();
                API::setWall(x, y, 'e');
            }
        }
        if (y > 0)
        {
            if (API::wallRight())
            {
                horizontal_walls[x][y - 1] = API::wallRight();
                API::setWall(x, y, 's');
            }
        }
        if (y < 15)
        {
            if (API::wallLeft())
            {
                horizontal_walls[x][y] = API::wallLeft();
                API::setWall(x, y, 'n');
            }
        }
    }


    else if (dirc == Diractions::WEST)
    {
        if (x > 0)
        {
            if (API::wallFront())
            {
                vertical_walls[x - 1][y] = API::wallFront();
                API::setWall(x, y, 'w');
            }
        }
        if (y > 0)
        {
            if (API::wallLeft())
            {
                horizontal_walls[x][y - 1] = API::wallLeft();
                API::setWall(x, y, 's');
            }
        }
        if (y < 15)
        {
            if (API::wallRight())
            {
                horizontal_walls[x][y] = API::wallRight();
                API::setWall(x, y, 'n');
            }
        }
    }




    else if (dirc == Diractions::SOUTH)
    {
        if (x > 0)
        {
            if (API::wallRight())
            {
                vertical_walls[x - 1][y] = API::wallRight();
                API::setWall(x, y, 'w');
            }
        }
        if (x < 15)
        {
            if (API::wallLeft())
            {
                vertical_walls[x][y] = API::wallLeft();
                API::setWall(x, y, 'e');
            }
        }
        if (y > 0)
        {
            if (API::wallFront())
            {
                horizontal_walls[x][y - 1] = API::wallFront();
                API::setWall(x, y, 's');
            }
        }
    }



}
















void flood_fill(short int x, short int y)
{


    Cell current_cell = {x, y};
    if (distance(x, y) == 0)
    {
        return;
    }



    flood_fill_queue.push(current_cell);


    while (!flood_fill_queue.empty())
    {
        Cell next_cell = flood_fill_queue.front();
        flood_fill_queue.pop();
        vector<Cell> Avalabile_cells = get_accessable_cells(next_cell.x, next_cell.y);
        Cell min_element = getMinCell(Avalabile_cells);


        if (distance(next_cell) - 1 != distance(min_element))
        {

            if (flood_fill_queue.size() > max_queue_size)
            {
                max_queue_size = flood_fill_queue.size();
            }


            int min_neighbour = distance(min_element);
            API::setText(next_cell.x, next_cell.y, to_string(min_neighbour + 1));
            API::setColor(myRobot.x, myRobot.y, 'y');
            manhattan_distance[next_cell.x][next_cell.y] = min_neighbour + 1;


            for (int i = 0; i < Avalabile_cells.size(); i++)
            {
                flood_fill_queue.push(Avalabile_cells[i]);
            }
        }


    }


}









void init()
{
    API::setText(0, 0, "Start");
    for (int i = 0; i < MAZE_SIZE; i++)
    {
        for (int j = 0; j < MAZE_SIZE; j++)
        {
            API::setText(i, j, to_string(manhattan_distance[i][j]));
        }
    }
}












void find()
{
    while (true)
    {
        update_walls();
        
        Cell current_cell = {myRobot.x, myRobot.y};
        API::setColor(myRobot.x, myRobot.y, 'y');
        vector<Cell> Avalabile_Cells = get_accessable_cells(myRobot.x, myRobot.y);
        Cell min_Ava_Cell = getMinCell(Avalabile_Cells);



        if (distance(current_cell) > distance(min_Ava_Cell))
        {

            change_deriction(current_cell, min_Ava_Cell);
            API::moveForward();

            myRobot.x = min_Ava_Cell.x;
            myRobot.y = min_Ava_Cell.y;
        }


        else if (manhattan_distance[myRobot.x][myRobot.y] == 0)
        {
            cerr << "Max queue size: " << max_queue_size << endl;
            printFloodFillQueue(flood_fill_queue);
            return;
        }


        else
        {
            flood_fill(myRobot.x, myRobot.y);
        }


    }
}







Cell getMinCell(vector<Cell> cells)
{

    Cell min_cell = cells[0];
    for (int i = 1; i < cells.size(); i++)
    {
        if (distance(cells[i]) < distance(min_cell))
        {
            min_cell = cells[i];
        }
    }
    return min_cell;
}







vector<Cell> get_accessable_cells(short int x, short int y)
{
    vector<Cell> Avalabile_Cells;
    if (x > 0 && !vertical_walls[x - 1][y])
    {
        Cell left_cell;
        left_cell.x = x - 1;
        left_cell.y = y;
        Avalabile_Cells.push_back(left_cell);
    }


    if (x < 15 && !vertical_walls[x][y])
    {
        Cell right_cell;
        right_cell.x = x + 1;
        right_cell.y = y;
        Avalabile_Cells.push_back(right_cell);
    }


    if (y > 0 && !horizontal_walls[x][y - 1])
    {
        Cell bottom_cell;
        bottom_cell.x = x;
        bottom_cell.y = y - 1;
        Avalabile_Cells.push_back(bottom_cell);
    }


    if (y < 15 && !horizontal_walls[x][y]) // y = 11, x = 5
    {
        Cell top_cell;
        top_cell.x = x;
        top_cell.y = y + 1;
        Avalabile_Cells.push_back(top_cell);
    }


    return Avalabile_Cells;
}





void change_deriction(Cell current_cell, Cell min_cell)
{
    if (current_cell.x == min_cell.x)
    {
        if (current_cell.y < min_cell.y) // SHOULD GO TO NORTH
        {
            if (myRobot.dirc == Diractions::EAST)
            {
                API::turnLeft();
            }

            else if (myRobot.dirc == Diractions::SOUTH)
            {
                API::turnLeft();
                API::turnLeft();
            }

            else if (myRobot.dirc == Diractions::WEST)
            {
                API::turnRight();
            }
            myRobot.dirc = Diractions::NORTH;
        }



        else if (current_cell.y > min_cell.y) // SHOULD GO TO SOUTH
        {
            if (myRobot.dirc == Diractions::NORTH)
            {
                API::turnLeft();
                API::turnLeft();
            }

            else if (myRobot.dirc == Diractions::EAST)
            {
                API::turnRight();
            }

            else if (myRobot.dirc == Diractions::WEST)
            {
                API::turnLeft();
            }
            myRobot.dirc = Diractions::SOUTH;
        }
    }



    else if (current_cell.y == min_cell.y)
    {
        if (current_cell.x < min_cell.x) // SHOULD GO TO EAST
        {
            if (myRobot.dirc == Diractions::NORTH)
            {
                API::turnRight();
            }

            else if (myRobot.dirc == Diractions::SOUTH)
            {
                API::turnLeft();
            }

            else if (myRobot.dirc == Diractions::WEST)
            {
                API::turnLeft();
                API::turnLeft();
            }

            myRobot.dirc = Diractions::EAST;
        }



        else if (current_cell.x > min_cell.x) // SHOULD GO TO WEST
        {
            if (myRobot.dirc == Diractions::NORTH)
            {
                API::turnLeft();
            }

            else if (myRobot.dirc == Diractions::SOUTH)
            {
                API::turnRight();
            }

            else if (myRobot.dirc == Diractions::EAST)
            {
                API::turnLeft();
                API::turnLeft();
            }

            myRobot.dirc = Diractions::WEST;
        }
    }


}





int main(int argc, char *argv[])
{

    init();
    find();
}