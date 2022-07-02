
#include "stdio.h"


extern void TestMainEntry();
extern void TestPython();
extern void TestSIMD();

int main()
{
	printf("Running ...\n");
	TestMainEntry();
	printf("All Test Done\n");
	return 0;
}
