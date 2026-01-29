win32 {
    HEADERS += $$system(dir /s /b *.h)
} else {
    HEADERS += $$system(find . -type f -name "*.h")
}
