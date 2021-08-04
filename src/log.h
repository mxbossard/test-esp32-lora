#include <Arduino.h>

#ifdef LOG_TO_SERIAL

#endif

#ifdef LOG_TO_SCREEN
#include <U8g2lib.h>
#include <CircularBuffer.h>
#endif

void _log(String message, bool newLine = true, bool screen = false) {
    #ifdef LOG_TO_SERIAL
    if (newLine) {
        Serial.println(message);
    } else {
        Serial.print(message);
    }
    Serial.flush();
    #endif

    #ifdef LOG_TO_SCREEN
    if (screen) {
        String lastMessage = loggingQueue[0];
        int8_t maxCharCount = u8g2.getDisplayWidth() / u8g2.getMaxCharWidth() * 1.5;
        char lastMessageEol = lastMessage.charAt(lastMessage.length() - 1);
        if (lastMessageEol == '\n') {
            message = message.substring(0, maxCharCount);
            if (newLine) {
                message += '\n';
            }
            loggingQueue.unshift(message);
        } else {
            String concat = loggingQueue.shift();
            message = concat + message;
            message = message.substring(0, maxCharCount);
            if (newLine) {
                message += '\n';
            }
            loggingQueue.unshift(message);
        }

        displayLoggingQueue();
    }
    #endif
}

void logln(String message, bool screen = true) {
    _log(message, true, screen);
}

void log(String message, bool screen = true) {
    _log(message, false, screen);
}

void _log(int message, char base = 10, bool newLine = true, bool screen = false) {
    #if defined(LOG_TO_SERIAL) || defined(LOG_TO_SCREEN)
    _log(String(message, base), newLine, screen);
    #endif
}

void logln(int message, char base = 10, bool screen = true) {
    _log(message, base, true, screen);
}

void log(int message, char base = 10, bool screen = true) {
    _log(message, base, false, screen);
}

void _log(unsigned int message, char base = 10, bool newLine = true, bool screen = false) {
    #if defined(LOG_TO_SERIAL) || defined(LOG_TO_SCREEN)
    _log(String(message, base), newLine, screen);
    #endif
}

void logln(unsigned int message, char base = 10, bool screen = true) {
    _log(message, base, true, screen);
}

void log(unsigned int message, char base = 10, bool screen = true) {
    _log(message, base, false, screen);
}

void _log(long message, char base = 10, bool newLine = true, bool screen = false) {
    #if defined(LOG_TO_SERIAL) || defined(LOG_TO_SCREEN)
    _log(String(message, base), newLine, screen);
    #endif
}

void logln(long message, char base = 10, bool screen = true) {
    _log(message, base, true, screen);
}

void log(long message, char base = 10, bool screen = true) {
    _log(message, base, false, screen);
}

void _log(unsigned long message, char base = 10, bool newLine = true, bool screen = false) {
    #if defined(LOG_TO_SERIAL) || defined(LOG_TO_SCREEN)
    _log(String(message, base), newLine, screen);
    #endif
}

void logln(unsigned long message, char base = 10, bool screen = true) {
    _log(message, base, true, screen);
}

void log(unsigned long message, char base = 10, bool screen = true) {
    _log(message, base, false, screen);
}
