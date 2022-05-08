#include "mission.h"
#include <iostream>

Mission::Mission()
{
    logger = nullptr;
    fileName = nullptr;
}

Mission::Mission(const char *FileName)
{
    fileName = FileName;
    logger = nullptr;
}

Mission::~Mission()
{
    if (logger)
        delete logger;
}

bool Mission::getMap()
{
    return map.getMap(fileName);
}

bool Mission::getConfig()
{
    return config.getConfig(fileName);
}

bool Mission::createLog()
{
    if (logger != nullptr) delete logger;
    logger = new XmlLogger(config.LogParams[CN_LP_LEVEL]);
    return logger->getLog(fileName, config.LogParams);
}

void Mission::createEnvironmentOptions()
{
    options.cutcorners = config.SearchParams[CN_SP_CC];
    options.allowsqueeze = config.SearchParams[CN_SP_AS];
    options.allowdiagonal = config.SearchParams[CN_SP_AD];
    options.metrictype = config.SearchParams[CN_SP_MT];

}

void Mission::createSearch()
{
//might be helpful in case numerous algorithms are added
}

void Mission::startSearch()
{
    sr = search.startSearch(logger, map, options);
}

void Mission::startSeqSearch() {
    sr = seqsearch.startSeqSearch(logger, map, options);
}

void Mission::startDLiteSearch() {
    dsr = dlitesearch.StartDLiteSearch(logger, map, options);
}

void Mission::printSearchResultsToConsole()
{
    std::cout << "Path ";
    if (!dsr.pathfound)
        std::cout << "NOT ";
    std::cout << "found!" << std::endl;
    std::cout << "numberofsteps=" << dsr.numberofsteps << std::endl;
    std::cout << "nodescreated=" << dsr.nodescreated << std::endl;
    if (dsr.pathfound) {
        std::cout << "pathlength=" << dsr.pathlength << std::endl;
        std::cout << "pathlength_scaled=" << dsr.pathlength * map.getCellSize() << std::endl;
    }
    std::cout << "time=" << dsr.time << std::endl;
}

void Mission::saveSearchResultsToLog()
{
    logger->writeToLogSummary(dsr.numberofsteps, dsr.nodescreated, dsr.pathlength, dsr.time, map.getCellSize(), dsr.memory);
    if (dsr.pathfound) {
        logger->writeToLogPath(*dsr.lppath);
        logger->writeToLogHPpath(*dsr.hppath);
        logger->writeToLogMap(map, *dsr.lppath);
    } else
        logger->writeToLogNotFound();
    logger->saveLog();
}

DLiteSearchResult Mission::getSearchResult()
{
    return dsr;
}

