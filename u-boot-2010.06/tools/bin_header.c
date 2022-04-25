/*
 * convert binary file into a C Header file
 * based on bmp_logo.c
 *
 * Scott Esters (sesters@leapfrog.com)
 *
 */

#define _GNU_SOURCE
#include "compiler.h"

#define	BIN_HEADER_FILE	"SCREENS"

int main (int argc, char *argv[])
{
	FILE	*fp;
	int	ch;
	int	i, j;
	char	*cScreenName;
	char	*cBaseName;
	char	*cFileName;

	/* check for at least one FILENAME */
	if (argc < 2) {
		fprintf (stderr, "Usage: %s File\n", argv[0]);
		exit (EXIT_FAILURE);
	}

	printf ("/*\n"
		" * Automatically generated by \"tools/bin_header\"\n"
		" *\n"
		" * DO NOT EDIT\n"
		" *\n"
		" */\n\n\n"
		"#ifndef __%s__\n"
		"#define __%s__\n\n"
		"\n",
		BIN_HEADER_FILE, BIN_HEADER_FILE);

	i = 1;	/* point at first LABEL */

	while (i < argc) {
		cFileName = argv[i++];		/* get data filename */

		/*
		 * Convert filename to Array name
		 *   Drop path information, then mangle remaining
		 *   chars to valid C array name.
		 */

		/* get just filename */
		cBaseName = basename(cFileName);

		/* space for name */
		cScreenName = malloc(strlen(cBaseName) + 1);

		/* copy filename, mangle to legal C array name */
		for (j=0; j < strlen(cBaseName); j++) {
			if (isalnum(cBaseName[j]) || cBaseName[j] == '_')
				cScreenName[j] = cBaseName[j];
			else
				cScreenName[j] = '_';
		}


		if ((fp = fopen (cFileName, "rb")) == NULL) {
			perror (cFileName);
			exit (EXIT_FAILURE);
		}

		printf ("unsigned char %s[] = {\n", cScreenName);

		j = 0;
		while ((ch = fgetc(fp)) != EOF) {
			if ((j%8) == 0)
				putchar ('\t');
			printf ("0x%02X,%c", ch, ((j%8) == 7) ? '\n' : ' ');
			j++;
		}
		printf ("\n"
			"};\n\n");

		fclose (fp);
	}

	printf ("\n"
		"#endif /* __%s__ */\n",
		BIN_HEADER_FILE);

	return (0);
}
