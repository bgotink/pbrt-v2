//
//  break.h
//  pbrt
//
//  Created by Bram Gotink on 10/06/13.
//
//

#ifndef __pbrt__break__
#define __pbrt__break__

#define SHAFT_ENABLE_BREAK

#define SHAFT_BREAK_MANYPRIMS
#define SHAFT_BREAK_MANYPRIMS_TRESHOLD  10000

#if defined(SHAFT_ENABLE_BREAK)
namespace shaft { namespace breakp {

#if defined(SHAFT_BREAK_MANYPRIMS)
#   ifndef SHAFT_BREAK_MANYPRIMS_TRESHOLD
#       define SHAFT_BREAK_MANYPRIMS_TRESHOLD 10000
#   endif
    void manyprims();
#endif

}} // namespace breakp } namespace shaft }
#endif // defined(SHAFT_ENABLE_BREAK)

#endif /* defined(__pbrt__break__) */
