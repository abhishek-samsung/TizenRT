/* Copyright (C) 2012 Gregory Nutt
 *
 * This file is part of the uClibc++ Library.
 *
 * A replacement for __gnu_cxx::terminate
 */

#include <cstdlib>
#include <debug.h>
#include <stdio.h>
// This is a brain-dead replacement for __gnu_cxx::__verbose_terminate_handler
namespace __gnu_cxx
{
  void __verbose_terminate_handler()
  {
     printf("ERROR: bye Abhishek Terminating...\n");
     volatile int x = 0;
     for (int i = 0; i < 1000; i++) {
	x++;
     }
     abort();
  }
}
