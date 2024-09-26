#include "utils.h"

void test_nagative_time(void)
{
    struct timespec a = {-8, 0}, b = {0, 900000}, r;
    timespec_diff(&a, &b, &r);
    print(&r);
    printf("\n");
}

int main(void)
{
    test_nagative_time();
}
