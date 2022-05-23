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
    sr = search.startSearch(map, options);
}

void Mission::startSeqSearch() {
    sr = seqsearch.startSeqSearch(map, options);
}

void Mission::startDLiteSearch() {
    dsr = dlitesearch.StartDLiteSearch(map, options);
}

void Mission::printSearchResultsToConsole()
{
    if (config.SearchParams[CN_SP_ST] == CN_SP_ST_SEQASTAR || config.SearchParams[CN_SP_ST] == CN_SP_ST_ASTAR) {
        std::cout << "Path ";
        if (!sr.pathfound)
            std::cout << "NOT ";
        std::cout << "found!" << std::endl;
        std::cout << "numberofsteps=" << sr.numberofsteps << std::endl;
        std::cout << "nodescreated=" << sr.nodescreated << std::endl;
        if (sr.pathfound) {
            std::cout << "pathlength=" << sr.pathlength << std::endl;
            std::cout << "pathlength_scaled=" << sr.pathlength * map.getCellSize() << std::endl;
        }
        std::cout << "time=" << sr.time << std::endl;
    } else {
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
}

void Mission::saveSearchResultsToLog()
{
    if (config.SearchParams[CN_SP_ST] == CN_SP_ST_SEQASTAR || config.SearchParams[CN_SP_ST] == CN_SP_ST_ASTAR) {
        logger->writeToLogSummary(sr.numberofsteps, sr.nodescreated, sr.pathlength, sr.time, map.getCellSize(), sr.memory);
        if (sr.pathfound) {
            logger->writeToLogPath(*sr.lppath);
            logger->writeToLogHPpath(*sr.hppath);
            logger->writeToLogMap(map, *sr.lppath);
        } else
            logger->writeToLogNotFound();
        logger->saveLog();
    } else {
        logger->writeToLogSummary(dsr.numberofsteps, dsr.nodescreated, dsr.pathlength, dsr.time, map.getCellSize(), dsr.memory);
        if (dsr.pathfound) {
            logger->writeToLogPathDLite(*dsr.lppath);
            logger->writeToLogHPpathDLite(*dsr.hppath);
            logger->writeToLogMapDLite(map, *dsr.lppath);
        } else
            logger->writeToLogNotFound();
        logger->saveLog();
    }
}

DLiteSearchResult Mission::getSearchResult()
{
    return dsr;
}

