#ifndef ABSTRACTTEXTBUFFER_H
#define ABSTRACTTEXTBUFFER_H

#include <QtCore/QString>

class AbstractTextBuffer
{
public:
    AbstractTextBuffer();
    virtual ~AbstractTextBuffer();
    virtual int openFile(const QString &path) = 0;			// 打开文件并将内容加载入缓存区,返回文件位于缓存区中的第几位
    virtual QStringList getTextList(unsigned int n) const = 0;	// 获取指定文件内容
    virtual QString getFilePath(unsigned int n) const = 0;         //获取文件路径
};

#endif // ABSTRACTTEXTBUFFER_H
