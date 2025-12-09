#include "text_buffer.h"
#include <QDebug>
#include <QFile>
#include <QTextStream>

TextBuffer::TextBuffer()
{
}

TextBuffer::~TextBuffer()
{
}

int TextBuffer::openFile(const QString &path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << path << file.errorString();
        return -1;
    }
    QTextStream in(&file);
    TextData textData;
    textData.filePath = path;
    textData.content = in.readAll();
    m_buffer.push_back(textData);
    file.close();
    if (m_buffer.size() > static_cast<size_t>(std::numeric_limits<int>::max()))
    {
        return -1;
    }
    return static_cast<int>(m_buffer.size() - 1);
}

QStringList TextBuffer::getTextList(unsigned int n) const
{
    if (n >= m_buffer.size())
        return QStringList();
    return m_buffer[n].content.split("\n");
}

QString TextBuffer::getFilePath(unsigned int n) const
{
    if (n >= m_buffer.size())
        return QString();
    return m_buffer[n].filePath;
}

const QString &TextBuffer::getText(unsigned int n) const
{    
    if (n >= m_buffer.size())
        return m_emptyText;
    return m_buffer[n].content;
}

void TextBuffer::setFrontText(const QString &text)
{
    if (m_buffer.empty())
        return;
    m_buffer[0].content = text;
}

void TextBuffer::saveText()
{
    if (m_buffer.empty()) return;
    QFile file(m_buffer[0].filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        qDebug() << "write file error: " << file.errorString();
        return;
    }
    QTextStream out(&file);
    out << m_buffer[0].content;
    file.close();
}

void TextBuffer::clear()
{
    m_buffer.clear();
}

unsigned int TextBuffer::size() const
{
    return static_cast<unsigned int>(m_buffer.size());
}

QString TextBuffer::getNewFilePath()
{
    if(m_buffer.empty()) return QString();
    return  m_buffer.back().filePath;
}
