#include "AStar.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main (int argc, char **argv)
{
	if (argc != 6) {
		fprintf (stderr, "testAStar <mapfile> <startX> <startY> <goalX> <goalY>\n");
		fprintf (stderr, "(where <mapfile> is of the format used in http://www.aiide.org/benchmarks/)\n");
		exit (1);
	}
	int width;
	int height;

	FILE *mapFile = fopen (argv[1], "r");

	fscanf (mapFile, "type octile\nheight %i\nwidth %i\nmap\n", &height, &width);

	printf ("%i %i\n", width, height);
	short grid[width*height];
	memset (grid, 0, sizeof (short) * width * height);

	for (int i = 0; i < height - 1; i++) {
		char buf[width + 1]; // space for a newline
		int read = fread (buf, 1, width + 1, mapFile);
		for (int j = 0; j < width - 1; j++) {
			if (buf[j] == '.' || buf[j] == 'G')
				grid[width*i+j] = 1;
			else
				grid[width*i+j] = 0;
		}
	}

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			printf (grid[x + width * y] ? "." : "#");
		}
		puts ("");
	}

	int solLength = 0;
	int begin = astar_getIndexByWidth (width, atoi(argv[2]), atoi(argv[3]));
	int end = astar_getIndexByWidth (width, atoi(argv[4]), atoi(argv[5]));

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
