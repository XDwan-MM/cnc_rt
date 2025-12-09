#ifndef TEXT_BUFFER_H
#define TEXT_BUFFER_H

#include "abstracttextbuffer.h"
#include <vector>

/**
 * @brief NC文件文本缓存区
 * @details
 * 缓存区头的内容为主程序内容，其余为子程序内容
 * 只可修改和保存主程序内容
 */
class TextBuffer : public AbstractTextBuffer
{
public:
    struct TextData
    {
        QString filePath;
        QString content;
    };

    TextBuffer();
    ~TextBuffer() override;
    int openFile(const QString &path) override;             // 打开文件并将内容加载入缓存区,返回文件位于缓存区中的第几位
    QStringList getTextList(unsigned int n) const override;	// 获取指定文件内容
    QString getFilePath(unsigned int n) const override;     // 获取文件路径

    const QString &getText(unsigned int n) const;           // 获取指定文件内容
    void setFrontText(const QString &text);                 // 设置缓存区头内容
    void saveText();                                        // 将缓存区头内容存储文件
    void clear();                                           // 清空缓存区
    unsigned int size() const;                              // 获取缓存区大小
    QString getNewFilePath();

private:
    std::vector<TextBuffer::TextData> m_buffer;

    QString m_emptyText;
};

#endif // TEXT_BUFFER_H
