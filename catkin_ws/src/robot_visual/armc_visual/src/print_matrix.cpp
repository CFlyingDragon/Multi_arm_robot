#include <fstream>
#include "armc_visual/print_matrix.h"

void print_matrix(double *a, int dimi, int dimj, char *ch)
{
	FILE *fp;
	int i, j;
	fp = fopen(ch, "w");
	for (i = 0; i < dimi; i++)
	{
		for (j = 0; j < dimj; j++)
		{
			fprintf(fp, "%6.3f ", *(a + i*dimj + j));
			if (j == dimj - 1)
			{
				fprintf(fp, "\n");
			}
		}
	}
	fclose(fp);
}
