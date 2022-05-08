#ifndef XMLLOGGER_H
#define	XMLLOGGER_H
#include "tinyxml2.h"
#include "ilogger.h"


//That's the class that flushes the data to the output XML


class XmlLogger {

public:
    XmlLogger(std::string loglevel) {this->loglevel = loglevel;}

    virtual ~XmlLogger() {};

    bool getLog(const char *FileName, const std::string *LogParams);

    void saveLog();

    template<typename T>
    void writeToLogMap(const Map &Map, const std::list<T> &path);

    //void writeToLogOpenClose(const typename &open, const typename &close);

    template<typename T>
    void writeToLogPath(const std::list<T> &path);

    template<typename T>
    void writeToLogHPpath(const std::list<T> &hppath);

    void writeToLogNotFound();

    void writeToLogSummary(unsigned int numberofsteps, unsigned int nodescreated, float length, double time, double cellSize, unsigned int memory);

private:
    std::string loglevel;
    std::string LogFileName;
    tinyxml2::XMLDocument doc;
};

#endif

