#ifndef TEST_ENV_H
#define TEST_ENV_H

#include <string>

void testEnvSetReady(bool ready, const std::string &message = std::string());
bool testEnvReady();
const std::string &testEnvMessage();

#endif // TEST_ENV_H
