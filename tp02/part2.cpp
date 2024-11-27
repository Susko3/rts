#include <cassert>

#include "utils.h"
#include "preprocess.cpp"

int main()
{
    assert(set_realtime_priority());
    assert(pin_this_thread());
    assert(increase_clock_resolution());

    reset_clock_resolution();
}
