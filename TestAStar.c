#include "AStar.h"
#include <stdio.h>
#include <stdlib.h>

int main ()
{
	int width = 5;
	int height = 5;
	short grid[] = {1, 1, 1, 1, 1,
		        0, 0, 0, 0, 1,
		        1, 1, 1, 1, 1,
		        1, 0, 0, 0, 0,
		        1, 1, 1, 1, 1};
	
	int solLength = 0;
	int begin = astar_getIndexByWidth (width, 0, 0);
	int end = astar_getIndexByWidth (width, 4, 4);

	int* solution = astar_compute (grid, &solLength, width, height, begin, end);

	// print the coordinates of the solution
	// (remember: They're in reverse order in the array, and as usual
	// the starting point is not included in the solution but the ending
	// point is)
	printf ("solLength: %i\n", solLength);
	for (int i = solLength - 1; i >= 0; i--) {
		int x, y;
		astar_getCoordByWidth (width, solution[i], &x, &y);
		printf ("(%i,%i)\n", x, y);
	}
	printf ("\n---\n\n");
	// print the grid along with the solution path
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			int wasInSolution = 0;
			for (int i = 0; i < solLength; i++) {
				if ((x + width * y) == solution[i]) {
					printf ("%i", i % 10);
					wasInSolution = 1;
				}					
			}
			if (!wasInSolution)
				printf (grid[x + width*y] ? "." : "#");
		}
		printf ("\n");
	}
	free (solution);
}
