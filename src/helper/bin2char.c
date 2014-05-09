/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
 #include <fcntl.h>
#endif

int main(int argc, char **argv)
{
	int c;
	unsigned int n;
	const char *name;

	if (argc == 1) {
		fprintf(stderr, "bin2char <varname>\n");
		fprintf(stderr, "read from standard input and write a char"
			" array out to standard output\n");
		exit(1);
	}

#ifdef _WIN32
	/* for win32 set stdin/stdout to binary mode */
	_setmode(_fileno(stdin), _O_BINARY);
	_setmode(_fileno(stdout), _O_BINARY);
#endif

	n = 0;
	name = argv[1];
	fprintf(stdout, "/* autogenerated from %s */\n", argv[0]);
	fprintf(stdout, "unsigned const char %s[] = {\n", name);
	while ((c = getc(stdin)) != EOF) {
		fprintf(stdout, "0x%02x,", c & 0xff);
		if ((++n % 16) == 0)
			fprintf(stdout, "\n");
	}
	fprintf(stdout, "0 /* terminate with a null */};\n");
	return 0;
}