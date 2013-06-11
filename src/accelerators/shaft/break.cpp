//
//  break.cpp
//  pbrt
//
//  Created by Bram Gotink on 10/06/13.
//
//

#include "break.h"

#if defined(SHAFT_ENABLE_BREAK)
namespace shaft { namespace breakp {

#if defined(SHAFT_BREAK_MANYPRIMS)
    void manyprims() {}
#endif


}}
#endif // defined(SHAFT_ENABLE_BREAK