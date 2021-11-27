#ifndef INPUT_H
#define INPUT_H

#pragma once

#include <linux/input-event-codes.h>

class Mouse {
public:
    Mouse();
    ~Mouse();

    bool open(const char *device);
    void close();

    bool read(int *x, int *y);

private:
    int fd;
};

extern "C" bool getKey(int key);

extern "C" bool getJoyButton(int button);
extern "C" int getJoyAxis(int axis);

#endif // INPUT_H
