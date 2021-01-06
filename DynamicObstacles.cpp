#include "bit_star.h"
#include <iostream>

using namespace std;
int main(int argc, char *argv[])
{
    srand (0);// Random seed
    BIT_star bit;
    bit.parsing_map();// Paring the map
    int src = 0, t = 1;
    //bit.a_star(src, t);
    bit.bit_star(src, t);
    //src = bit.get_source();
    //cerr << src << endl;
    //exit(-1);
    //bit.dynamic_bit_star(src,t);
    return 0;
}
