#include "environmentoptions.h"

EnvironmentOptions::EnvironmentOptions()
{
    metrictype = CN_SP_MT_EUCL;
    allowsqueeze = false;
    allowdiagonal = true;
    cutcorners = false;
    searchtype = 0;
}

EnvironmentOptions::EnvironmentOptions(bool AS, bool AD, bool CC, int MT, int ST)
{
    metrictype = MT;
    allowsqueeze = AS;
    allowdiagonal = AD;
    cutcorners = CC;
    searchtype = ST;
}

