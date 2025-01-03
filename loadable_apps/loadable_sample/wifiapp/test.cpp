#include <tinyara/config.h>

#include <cstdio>
#include <debug.h>

#include <tinyara/init.h>

//***************************************************************************
// Definitions
//***************************************************************************

//***************************************************************************
// Private Classes
//***************************************************************************

//***************************************************************************
// Private Data
//***************************************************************************

//***************************************************************************
// Public Functions
//***************************************************************************

extern "C"
{
	void callback() {
		throw(5);
	}

	int mytest()
	{
		// Print the cpp version used
		//printf("c++ version used : %d\n", __cplusplus);
		
		extern int helloxx_main(int argc, char *argv[]);

		try {
			//throw (5);
			helloxx_main(1, (char**)callback);
		} catch (int myNum) {
			printf("caught %d thrown in common and caught in app\n", myNum);
		}

		try {
                        throw (5);
                } catch (int myNum) {
                        printf("caught %d in app\n", myNum);
                }

		return 0;
	}
}
