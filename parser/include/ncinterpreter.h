#ifndef NCINTERPRETER_H
#define NCINTERPRETER_H

#include <QString>
//#include <QTextDocument>
#include "ParserState.h"
#include "coordinate.h"
#include "itpparam.h"
#include "abstracttextbuffer.h"
#include "motiondata.h"
#include "ncitperrortype.h"

class NCInterpreter_p;
class CoreMacroVariable;
class NCInterpreter
{
public:
    NCInterpreter();
    ~NCInterpreter();
//    bool setFile(const QString &fileName, int startLine = 0);
//    bool setDocument(const QTextDocument &doc, QString fileName = "000", int startLine = 0);
//    bool setStringList(const QStringList &stringList, int startLine = 0);

    void setTextBuffer(AbstractTextBuffer *textBuffer, int startLine = 0);
    void setItpParam(const ItpParam &parameter);
    void setOriginMacPos(double x, double y, double z, double a, double b, double c);
    MacroVarUpdate autosetToolLenthComp(double z);
    void setMacroVariable(MacroVar cmv[]);

    ItpParam getItpParam();

    void nextLine();
    QList<pmc_data> motionData() const;
    ParserState state() const;
    NcKeyWord originalData() const;
    QString fileName() const;
    int lineNumber() const;
    bool isOver() const;
    void reset();
    bool showToollifeState();

    QList<double> toolLenthComp();

    QList<NCITPErrorType> errorList() const;
    QString errorStr() const;

    Coordinate& getCoordinate();

    void resetG92CoordnateOffset();

    NCITPVariable getItpInnerVar();

    void setGrade(int level);
    bool isM2M30() const;
    bool isProbeM6AllNeedVarUpdate() const;
private:
    NCInterpreter_p *m_itp;
};


#endif // NCINTERPRETER_H
