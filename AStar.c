#include "AStar.h"
#include "IndexPriorityQueue.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

// A few forward declarations for functions needed for the distance metrics
static double max (double, double);
static double chebyshevDistance (struct coord, struct coord);

// Distance metrics for distance estimation and precise computation. You might
// want to change these to match your game mechanics.

// Chebyshev distance metric for distance estimation by default
static double estimateDistance (struct coord start, struct coord end)
{
	return chebyshevDistance (start, end);
}

// Since we only work on uniform-cost maps, this function only needs
// to see the coordinates, not the map itself.
// Euclidean geometry by default.
// Note that since we jump over points, we actually have to compute 
// the entire distance - despite the uniform cost we can't just collapse
// all costs to 1
static double preciseDistance (struct coord start, struct coord end)
{
	if (start.x - end.x != 0 && start.y - end.y != 0)
		return sqrt (pow (start.x - end.x, 2) + 
			     pow (start.y - end.y, 2)) ;
	else
		return abs (start.x - end.x) + abs (start.y - end.y);
}

// Below this point, not a lot that there should be much need to change!

static double chebyshevDistance (struct coord start, struct coord end)
{
	return fmax (abs (start.x - end.x), abs (start.y - end.y));
}

// The order of directions is: 
// N, NE, E, SE, S, SW, W, NW 
typedef short direction;

/* Coordinates are represented either as pairs of an x-coordinate and
   y-coordinate, or map indexes, as appropriate. getIndex and getCoord
   convert between the representations. */
static int getIndex (struct coord bounds, struct coord c)
{
	return c.x + c.y * bounds.x;
}

int astar_getIndexByWidth (int width, int x, int y)
{
	return x + y * width;
}

static struct coord getCoord (struct coord bounds, int c)
{
	struct coord rv = { c % bounds.x, c / bounds.x };
	return rv;
}

void astar_getCoordByWidth (int width, int node, int *x, int *y)
{
	*x = node % width;
	*y = node / width;
}


// is this coordinate contained within the map bounds?
static int contained (struct coord bounds, struct coord c)
{
	return c.x >= 0 && c.y >= 0 && c.x < bounds.x && c.y < bounds.y;
}

// is this coordinate within the map bounds, and also walkable?
static int isEnterable (const short *grid, struct coord bounds, struct coord coord)
{
	return contained (bounds, coord) && grid[getIndex (bounds, coord)];
}

static int directionIsDiagonal (direction dir)
{
	return (dir % 2) != 0;
}

// the coordinate one tile in the given direction
static struct coord adjustInDirection (struct coord c, int dir)
{
	// we want to implement "rotation" - that is, for instance, we can
	// subtract 2 from the direction "north" and get "east"
	// C's modulo operator doesn't quite behave the right way to do this,
	// but for our purposes this should be plenty
	switch ((dir + 65536) % 8) {
	case 0: return (struct coord) {c.x, c.y - 1};
	case 1: return (struct coord) {c.x + 1, c.y - 1};
	case 2: return (struct coord) {c.x + 1, c.y };
	case 3: return (struct coord) {c.x + 1, c.y + 1};
	case 4: return (struct coord) {c.x, c.y + 1};
	case 5: return (struct coord) {c.x - 1, c.y + 1};
	case 6: return (struct coord) {c.x - 1, c.y};
	case 7: return (struct coord) {c.x - 1, c.y - 1};
	}
}

// logical implication operator
static int implies (int a, int b)
{
	return a ? b : 1;	
}

/* Harabor's explanation of exactly how to determine when a cell has forced
   neighbours is a bit unclear IMO, but this is the best explanation I could
   figure out. I won't go through everything in the paper, just the extra
   insights above what I thought was immediately understandable that it took
   to actually implement this function.

   First, to introduce the problem, we're looking at the immedate neighbours
   of a cell on the grid, considering what tile we arrived from.

   ...  This is the basic situation we're looking at. Supposing the top left
   -X.  period is cell (0,0), we're coming in to cell (1, 1) from (0, 1).
   ...  

   ...  The other basic case, the diagonal case. All other cases are obviously
   .X.  derivable from these two cases by symmetry.
   /..

   The question is: Given that some tiles might have walls, *how many tiles
   are there that we can reach better by going through the center tile than
   any other way?* (it's ok if we can reach them *as well* through the center
   tile as some other way)

   In case there are no obstructions, the answers are simple: In the horizontal
   or vertical case, the cell directly ahead; in the diagonal case, the three
   cells ahead.

   The paper is pretty abstract about what happens when there *are* 
   obstructions, but fortunately the abstraction seems to collapse into some
   fairly simple practical cases:

   123  Position 4 is a natural neighbour (according to the paper's terminology)
   -X4  so we don't need to count it. Positions 1, 2, 5 and 6 are accessible
   567  without going through the center tile. This leaves positions 3 and 7.

   Considering position 3 (everything here also follows for 7 by symmetry):
   If 3 is obstructed, then it doesn't matter what's in position in 2.
   If 3 is free and 2 is obstructed, 3 is a forced neighbour.
   If 3 is free and 2 is free, 3 is pruned (not a forced neighbour)

   i.e. logically, 
   3 is not a forced neighbour iff (3 is obstructed) implies (2 is obstructed).

   Similar reasoning applies for the diagonal case, except with bigger angles.
   
 */
static int hasForcedNeighbours (const short *grid, struct coord bounds, struct coord coord, int dir)
{
#define ENTERABLE(n) isEnterable (grid,\
	                          bounds,\
	                          adjustInDirection (coord, dir + (n)))
	if (directionIsDiagonal (dir))
		return !implies (ENTERABLE (-2), ENTERABLE (-3)) ||
		       !implies (ENTERABLE (2), ENTERABLE (3));
	else 
		return !implies (ENTERABLE (-1), ENTERABLE (-2)) ||
		       !implies (ENTERABLE (1), ENTERABLE (2));
#undef ENTERABLE
}

// directly translated from "algorithm 2" in the paper
static int jump (const short *grid, struct coord bounds, int goal, int dir, int start)
{
	struct coord coord = adjustInDirection (getCoord (bounds, start), dir);
	int node = getIndex (bounds, coord);
	if (!isEnterable (grid, bounds, coord))
		return -1;

	if (node == goal)
		return node;

	if (hasForcedNeighbours (grid, bounds, coord, dir))
		return node;

	if (directionIsDiagonal (dir)) {
		int next = jump (grid, bounds, goal, dir - 1, node);
		if (next >= 0)
			return node;
		next = jump (grid, bounds, goal, dir + 1, node);
		if (next >= 0)
			return node;
	}
	return jump (grid, bounds, goal, dir, node);
}

static int nextNodeInSolution (struct coord bounds, 
			       int *cameFrom, 
			       int *target,
			       int node)
{
	struct coord c = getCoord (bounds, node);
	struct coord cTarget = getCoord (bounds, *target);

	if (c.x < cTarget.x) 
		c.x++;
	else if (c.x > cTarget.x)
		c.x--;

	if (c.y < cTarget.y) 
		c.y++;
	else if (c.y > cTarget.y)
		c.y--;

	node = getIndex (bounds, c);

	if (node == *target)
		*target = cameFrom[*target];

	return node;
}

// a bit more complex than the usual A* solution-recording method,
// due to the need to interpolate path chunks
static int *recordSolution (struct coord bounds,
			    int *cameFrom,
			    int *solLen,
			    int begin,
			    int end)
{
	int rvLen = 1;
	*solLen = 0;
	int target = end;
	int *rv = malloc (rvLen * sizeof (int));
	int i = end;

	for (;;) {
		i = nextNodeInSolution (bounds, cameFrom, &target, i);
		rv[*solLen] = i;
		(*solLen)++;
		if (*solLen >= rvLen) {
			rvLen *= 2;
			rv = realloc (rv, rvLen * sizeof (int));
			if (!rv)
				return NULL;
		}
		if (i == begin)
			break;
	}

	(*solLen)--; // don't include the starting tile
	return rv;
}


static direction directionOfMove (struct coord to, struct coord from)
{
	if (from.x == to.x) {
		if (from.y == to.y)
			return -1;
		else if (from.y < to.y)
			return 4;
		else // from.y > to.y
			return 0;
	}
	else if (from.x < to.x) {
		if (from.y == to.y)
			return 2;
		else if (from.y < to.y)
			return 3;
		else // from.y > to.y
			return 1;
	}
	else { // from.x > to.x
		if (from.y == to.y)
			return 6;
		else if (from.y < to.y)
			return 5;
		else // from.y > to.y
			return 7;
	}

}

static direction directionWeCameFrom (struct coord bounds, int node, int nodeFrom)
{
	if (nodeFrom == -1)
		return -1;

	return directionOfMove (getCoord (bounds, node), getCoord (bounds, nodeFrom));
}

static int isOptimalTurn (int dir, int dirFrom)
{
	if (dirFrom == -1) // allow going in any direction from the start
		return 1;

	if (dirFrom == dir) // allow continuing without turning
		return 1;

	// slightly wonky-looking math because C's % has the wrong
	// behaviour with negative numerators for our purposes
	if (directionIsDiagonal (dirFrom)) {
		if ((dirFrom + 7) % 8 == dir ||
		    (dirFrom + 6) % 8 == dir ||
		    (dirFrom + 1) % 8 == dir ||
		    (dirFrom + 2) % 8 == dir)
			return 1;
	}
	else
		if ((dirFrom + 7) % 8 == dir ||
		    (dirFrom + 1) % 8 == dir)
			return 1;

	return 0;
}

int *astar_compute (const short *grid, 
		    int *solLength, 
		    int boundX, 
		    int boundY, 
		    int start, 
		    int end)
{
	struct coord bounds = {boundX, boundY};
	int size = bounds.x * bounds.y;

	*solLength = -1;

	if (start >= size || start < 0 || end >= size || end < 0)
		return NULL;

	struct coord startCoord = getCoord (bounds, start);
	struct coord endCoord = getCoord (bounds, end);

	if (!contained (bounds, startCoord) || !contained (bounds, endCoord))
		return NULL;

	queue *open = createQueue();
	char closed [size];
	double gScores [size];
	int cameFrom [size];

	memset (closed, 0, sizeof(closed));


	gScores[start] = 0;
	cameFrom[start] = -1;

	insert (open, start, estimateDistance (startCoord, endCoord));

	while (open->size) {
		int node = findMin (open)->value; 
		struct coord nodeCoord = getCoord (bounds, node);
		if (nodeCoord.x == endCoord.x && nodeCoord.y == endCoord.y) {
			freeQueue (open);
			return recordSolution (bounds, cameFrom, solLength, start, node);
		}

		deleteMin (open);
		closed[node] = 1;

		direction from = directionWeCameFrom (bounds, node, cameFrom[node]);

		for (int i = 0; i < 8; i++)
		{
			// only jump in optimal directions
			if (!isOptimalTurn (i, from))
				continue;

			int newNode = jump (grid, bounds, end, i, node);
			struct coord newCoord = getCoord (bounds, newNode);

			// this'll also bail out if jump() returned -1
			if (!contained (bounds, newCoord))
				continue;

			if (closed[newNode])
				continue;
			
			if (!exists (open, newNode)) {
				cameFrom[newNode] = node;
				gScores[newNode] = gScores[node] + 
					preciseDistance (nodeCoord, newCoord);
				insert (open, newNode, gScores[newNode] + 
					estimateDistance (newCoord, endCoord));
			}
			else if (gScores[newNode] > 
				 gScores[node] + 
				 preciseDistance (nodeCoord, newCoord)) {
				cameFrom[newNode] = node;
				int oldGScore = gScores[newNode];
				gScores[newNode] = gScores[node] + 
					preciseDistance (nodeCoord, newCoord);
				changePriority (open, 
						newNode, 
						priorityOf (open, newNode) - oldGScore + gScores[newNode]);
			}
		}
	}
	freeQueue (open);
	return NULL;
}

// for testing vs. the optimised case; this function won't stick around for too long
int *astar_unopt_compute (const short *grid, 
		    int *solLength, 
		    int boundX, 
		    int boundY, 
		    int start, 
		    int end)
{
	struct coord bounds = {boundX, boundY};
	int size = bounds.x * bounds.y;

	*solLength = -1;

	if (start >= size || start < 0 || end >= size || end < 0)
		return NULL;

	struct coord startCoord = getCoord (bounds, start);
	struct coord endCoord = getCoord (bounds, end);

	if (!contained (bounds, startCoord) || !contained (bounds, endCoord))
		return NULL;

	queue *open = createQueue();
	char closed [size];
	double gScores [size];
	int cameFrom [size];

	memset (closed, 0, sizeof(closed));


	gScores[start] = 0;
	cameFrom[start] = -1;
	if (contained (bounds, startCoord))
		insert (open, start, estimateDistance (startCoord, endCoord));
	
	while (open->size) {
		int node = findMin (open)->value; 
		struct coord nodeCoord = getCoord (bounds, node);
		if (nodeCoord.x == endCoord.x && nodeCoord.y == endCoord.y) {
			freeQueue (open);
			return recordSolution (bounds, cameFrom, solLength, start, node);
		}

		deleteMin (open);
		closed[node] = 1;

		for (int i = 0; i < 8; i++)
		{
			struct coord newCoord = adjustInDirection (nodeCoord, i);
			int newNode = getIndex (bounds, newCoord);

			// this'll also bail out if jump() returned -1
			if (!contained (bounds, newCoord) || !grid[newNode])
				continue;

			if (closed[newNode])
				continue;
			
			if (!exists (open, newNode)) {
				cameFrom[newNode] = node;
				gScores[newNode] = gScores[node] + 
					preciseDistance (nodeCoord, newCoord);
				insert (open, newNode, gScores[newNode] + 
					estimateDistance (newCoord, endCoord));
			}
			else if (gScores[newNode] > 
				 gScores[node] + 
				 preciseDistance (nodeCoord, newCoord)) {
				cameFrom[newNode] = node;
				int oldGScore = gScores[newNode];
				gScores[newNode] = gScores[node] + 
					preciseDistance (nodeCoord, newCoord);
				changePriority (open, 
						newNode, 
						priorityOf (open, newNode) - oldGScore + gScores[newNode]);
			}
		}
	}
	freeQueue (open);
	return NULL;
}
