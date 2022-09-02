#pragma once

class BasicOTA
{
private:
    static const char *ssid;
    static const char *password;

public:
    static void setupBasicOTA();
    static void handleBasicOTA();
};