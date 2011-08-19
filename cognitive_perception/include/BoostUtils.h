#ifndef BOOSTUTILS_H
#define BOOSTUTILS_H

#ifdef BOOST_THREAD
#include "boost/bind.hpp"
#define BOOST(A) A
#else
#define BOOST(A)
#endif


void Sleeping(long ms);

#endif //BOOSTUTILS_H


